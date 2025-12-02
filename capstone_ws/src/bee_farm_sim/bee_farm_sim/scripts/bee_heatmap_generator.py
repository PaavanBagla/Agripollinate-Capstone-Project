# scripts/heat_map_generator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import time

# Trying to use SciPy’s gaussian filter. If unavailable, use a tiny fallback blur.
try:
    from scipy.ndimage import gaussian_filter
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False
    def gaussian_filter(x, sigma):
        """
        Fallback blur filter if SciPy isn’t installed.
        This uses a basic 3x3 averaging kernel just so the heatmap
        still gets *some* smoothing.
        """
        kernel = np.ones((3, 3)) / 9.0
        from scipy.signal import convolve2d
        return convolve2d(x, kernel, mode='same', boundary='fill', fillvalue=0)

class BeeHeatmapGenerator(Node):
    """
    This node subscribes to /scan (LiDAR) data, identifies clusters of points
    that represent bees, and generates:
    - A real-time heatmap (decays over time)
    - A cumulative pollination heatmap (keeps growing)

    Clusters that remain in roughly the same place for longer than
    'pollination_time_threshold' seconds are counted as a pollinating bee.
    """
    def __init__(self):
        super().__init__('bee_heatmap_generator')

        # -------------------------------
        # ROS subscriber setup
        # -------------------------------
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # -------------------------------
        # Heatmap settings
        # -------------------------------
        self.grid_size = 400                 # Heatmap is 400x400 cells
        self.resolution = 0.05               # Each cell = 5cm (20m x 20m area)
        self.decay = 0.95                    # Real-time heat fades each frame
        self.max_clip = 50                   # Max heat per cell for visualization

        # Two heatmaps:
        # 1. self.heatmap -> real-time fading heatmap
        # 2. self.cumulative_heatmap -> long-term pollination map
        self.heatmap = np.zeros((self.grid_size, self.grid_size))
        self.cumulative_heatmap = np.zeros((self.grid_size, self.grid_size))

        # -------------------------------
        # Clustering + tracking settings
        # -------------------------------
        self.active_clusters = {}  # cluster_id -> {'pos': (x, y), 'first_seen': t, 'last_seen': t, 'counted': False}
        self.next_cluster_id = 1
        self.pollination_time_threshold = 2.0  # Seconds until cluster counted
        self.match_radius = 0.3              # Max distance to match cluster across frames
        self.bee_count = 0                   # Number of pollinating bees detected

        # How large each detected point spreads on the heatmap
        self.spread = 1  # 1 = 3x3 neighborhood; increase for bigger points
        self.point_weight = 10 # Heat added per cell

        # -------------------------------
        # Live Matplotlib visualization
        # -------------------------------
        plt.ion()
        self.fig, (self.ax_real, self.ax_cumulative) = plt.subplots(1, 2, figsize=(12, 6))
        
        # Real-time heatmap image
        self.im_real = self.ax_real.imshow(
            self.heatmap, cmap='hot', origin='lower',
            extent=[-self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2,
                    -self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2],
            vmin=0, vmax=self.max_clip
        )
        self.ax_real.set_title("Real-Time Bee Heatmap")
        self.ax_real.set_xlabel("X (m)")
        self.ax_real.set_ylabel("Y (m)")
        
        # Cumulative heatmap image
        self.im_cumulative = self.ax_cumulative.imshow(
            self.cumulative_heatmap, cmap='hot', origin='lower',
            extent=[-self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2,
                    -self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2],
            vmin=0, vmax=self.max_clip
        )
        self.ax_cumulative.set_title("Cumulative Pollination Heatmap")
        self.ax_cumulative.set_xlabel("X (m)")
        self.ax_cumulative.set_ylabel("Y (m)")

        plt.show(block=False)
        self.get_logger().info("Bee heatmap detector started — listening to /scan")

    # -------------------------------------------------------------------------
    # Callback: Reads LaserScan → clusters points → updates heatmaps
    # -------------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        # Convert ranges + angles → cartesian (x, y)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        mask = np.isfinite(ranges)
        if not np.any(mask):
            return

        xs = ranges[mask] * np.cos(angles[mask])
        ys = ranges[mask] * np.sin(angles[mask])
        points = np.column_stack((xs, ys))

        # -------------------------------
        # DBSCAN clustering to detect bees
        # -------------------------------
        if len(points) < 5:
            return # Too few points for meaningful clustering
        
        db = DBSCAN(eps=0.2, min_samples=3).fit(points)
        labels = db.labels_

        # Compute center point of each cluster
        cluster_centers = []
        for lbl in set(labels):
            if lbl == -1:
                continue # -1 = noise
            cluster_pts = points[labels == lbl]
            cluster_centers.append(cluster_pts.mean(axis=0))

        current_time = time.time()

        # -------------------------------
        # Match clusters to existing ones
        # (so we can track them across frames)
        # -------------------------------
        for center in cluster_centers:
            matched = False
            for cid, cdata in self.active_clusters.items():
                # Check if this cluster is close to an existing one
                if np.linalg.norm(center - cdata['pos']) < self.match_radius:
                    cdata['pos'] = center
                    cdata['last_seen'] = current_time
                    matched = True
                    break
            # If not matched, create a new cluster entry
            if not matched:
                self.active_clusters[self.next_cluster_id] = {
                    'pos': center,
                    'first_seen': current_time,
                    'last_seen': current_time,
                    'counted': False
                }
                self.next_cluster_id += 1

        # -------------------------------
        # Remove clusters that disappeared
        # -------------------------------
        to_delete = [cid for cid, cdata in self.active_clusters.items()
                     if current_time - cdata['last_seen'] > 1.0]
        for cid in to_delete:
            del self.active_clusters[cid]

        # -------------------------------
        # Add bee heat to heatmaps
        # -------------------------------
        local_real = np.zeros_like(self.heatmap)
        for cid, cdata in self.active_clusters.items():
            duration = current_time - cdata['first_seen']
            # Only considered "pollination" if stationary long enough
            if duration >= self.pollination_time_threshold:
                x, y = cdata['pos']
                # Convert world coordinates → heatmap cell indices
                gx = int((x + (self.grid_size * self.resolution / 2)) / self.resolution)
                gy = int((y + (self.grid_size * self.resolution / 2)) / self.resolution)

                # Spread heat over a small neighborhood to make it visible
                for dx in range(-self.spread, self.spread + 1):
                    for dy in range(-self.spread, self.spread + 1):
                        gx_sp = gx + dx
                        gy_sp = gy + dy
                        if 0 <= gx_sp < self.grid_size and 0 <= gy_sp < self.grid_size:
                            local_real[gy_sp, gx_sp] += self.point_weight
                            self.cumulative_heatmap[gy_sp, gx_sp] += self.point_weight

                # Only count a bee once
                if not cdata['counted']:
                    self.bee_count += 1
                    cdata['counted'] = True

        # -------------------------------
        # Apply smoothing + decay
        # -------------------------------
        blurred_real = gaussian_filter(local_real, sigma=1.5)
        self.heatmap *= self.decay           # Fade old heat
        self.heatmap += blurred_real         # Add new heat
        self.heatmap = np.clip(self.heatmap, 0, self.max_clip)
        self.cumulative_heatmap = np.clip(self.cumulative_heatmap, 0, self.max_clip)

        # -------------------------------
        # Update live plots
        # -------------------------------
        self.ax_real.clear()
        self.im_real = self.ax_real.imshow(
            self.heatmap, cmap='hot', origin='lower',
            extent=[-self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2,
                    -self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2],
            vmin=0, vmax=self.max_clip
        )
        self.ax_real.set_title("Real-Time Bee Heatmap")
        self.ax_real.set_xlabel("X (m)")
        self.ax_real.set_ylabel("Y (m)")

        self.ax_cumulative.clear()
        self.im_cumulative = self.ax_cumulative.imshow(
            self.cumulative_heatmap, cmap='hot', origin='lower',
            extent=[-self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2,
                    -self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2],
            vmin=0, vmax=self.max_clip
        )
        self.ax_cumulative.set_title(f"Cumulative Pollination Heatmap — Pollinating Bees: {self.bee_count}")
        self.ax_cumulative.set_xlabel("X (m)")
        self.ax_cumulative.set_ylabel("Y (m)")

        # Draw cluster IDs on top of real-time map for debugging
        for cid, cdata in self.active_clusters.items():
            x, y = cdata['pos']
            self.ax_real.text(x, y, str(cid), color='cyan', fontsize=8, ha='center', va='center')

        self.fig.canvas.flush_events()
        plt.pause(0.001)

# -------------------------------------------------------------------------
# Standard ROS2 Node Startup
# -------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = BeeHeatmapGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Keep final heatmaps visible after shutdown
        plt.ioff()
        plt.show()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
