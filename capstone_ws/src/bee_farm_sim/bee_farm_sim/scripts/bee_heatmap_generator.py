# scripts/heat_map_generator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import time

try:
    from scipy.ndimage import gaussian_filter
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False
    def gaussian_filter(x, sigma):
        # very small fallback: simple 3x3 average
        kernel = np.ones((3, 3)) / 9.0
        from scipy.signal import convolve2d
        return convolve2d(x, kernel, mode='same', boundary='fill', fillvalue=0)

class BeeHeatmapGenerator(Node):
    def __init__(self):
        super().__init__('bee_heatmap_generator')

        # ROS setup
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Heatmap parameters
        self.grid_size = 400
        self.resolution = 0.05  # 0.05 m per cell -> 20x20m area
        self.decay = 0.95
        self.max_clip = 50

        # Real-time decaying heatmap
        self.heatmap = np.zeros((self.grid_size, self.grid_size))
        # Cumulative pollination heatmap
        self.cumulative_heatmap = np.zeros((self.grid_size, self.grid_size))

        # Cluster tracking
        self.active_clusters = {}  # cluster_id -> {'pos': (x, y), 'first_seen': t, 'last_seen': t, 'counted': False}
        self.next_cluster_id = 1
        self.pollination_time_threshold = 2.0  # seconds
        self.match_radius = 0.3  # meters
        self.bee_count = 0

        # Size of cluster points
        self.spread = 1  # 1 = 3x3 neighborhood; increase for bigger points
        self.point_weight = 10

        # Matplotlib live view with two subplots
        plt.ion()
        self.fig, (self.ax_real, self.ax_cumulative) = plt.subplots(1, 2, figsize=(12, 6))
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

    def scan_callback(self, msg: LaserScan):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        mask = np.isfinite(ranges)
        if not np.any(mask):
            return

        xs = ranges[mask] * np.cos(angles[mask])
        ys = ranges[mask] * np.sin(angles[mask])
        points = np.column_stack((xs, ys))

        # --- DBSCAN clustering ---
        if len(points) < 5:
            return
        db = DBSCAN(eps=0.2, min_samples=3).fit(points)
        labels = db.labels_

        # --- Collect cluster centers ---
        cluster_centers = []
        for lbl in set(labels):
            if lbl == -1:
                continue
            cluster_pts = points[labels == lbl]
            cluster_centers.append(cluster_pts.mean(axis=0))

        current_time = time.time()

        # --- Match clusters to active_clusters ---
        for center in cluster_centers:
            matched = False
            for cid, cdata in self.active_clusters.items():
                if np.linalg.norm(center - cdata['pos']) < self.match_radius:
                    cdata['pos'] = center
                    cdata['last_seen'] = current_time
                    matched = True
                    break
            if not matched:
                self.active_clusters[self.next_cluster_id] = {
                    'pos': center,
                    'first_seen': current_time,
                    'last_seen': current_time,
                    'counted': False
                }
                self.next_cluster_id += 1

        # --- Remove old clusters ---
        to_delete = [cid for cid, cdata in self.active_clusters.items()
                     if current_time - cdata['last_seen'] > 1.0]
        for cid in to_delete:
            del self.active_clusters[cid]

        # --- Update heatmaps with spread points ---
        local_real = np.zeros_like(self.heatmap)
        for cid, cdata in self.active_clusters.items():
            duration = current_time - cdata['first_seen']
            if duration >= self.pollination_time_threshold:
                x, y = cdata['pos']
                gx = int((x + (self.grid_size * self.resolution / 2)) / self.resolution)
                gy = int((y + (self.grid_size * self.resolution / 2)) / self.resolution)

                # Spread the heat over a small neighborhood
                for dx in range(-self.spread, self.spread + 1):
                    for dy in range(-self.spread, self.spread + 1):
                        gx_sp = gx + dx
                        gy_sp = gy + dy
                        if 0 <= gx_sp < self.grid_size and 0 <= gy_sp < self.grid_size:
                            local_real[gy_sp, gx_sp] += self.point_weight
                            self.cumulative_heatmap[gy_sp, gx_sp] += self.point_weight

                if not cdata['counted']:
                    self.bee_count += 1
                    cdata['counted'] = True

        # --- Smooth and decay real-time heatmap ---
        blurred_real = gaussian_filter(local_real, sigma=1.5)
        self.heatmap *= self.decay
        self.heatmap += blurred_real
        self.heatmap = np.clip(self.heatmap, 0, self.max_clip)
        self.cumulative_heatmap = np.clip(self.cumulative_heatmap, 0, self.max_clip)

        # --- Update plots ---
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

        # Draw cluster IDs on real-time heatmap
        for cid, cdata in self.active_clusters.items():
            x, y = cdata['pos']
            self.ax_real.text(x, y, str(cid), color='cyan', fontsize=8, ha='center', va='center')

        self.fig.canvas.flush_events()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = BeeHeatmapGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
