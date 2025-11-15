# scripts/heat_map_generator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

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
        
        # Heatmap grid
        self.grid_size = 400
        self.resolution = 0.05  # 0.05 m per cell -> 20x20m area
        self.decay = 0.95
        self.max_clip = 50
        self.heatmap = np.zeros((self.grid_size, self.grid_size))

        # Matplotlib live view
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.im = self.ax.imshow(
            self.heatmap, cmap='hot', origin='lower',
            extent=[-self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2,
                    -self.grid_size*self.resolution/2,
                    self.grid_size*self.resolution/2],
            vmin=0, vmax=self.max_clip
        )
        self.cbar = self.fig.colorbar(self.im, ax=self.ax)
        self.cbar.set_label("Bee Heat Intensity")
        self.ax.set_title("Bee Detection Heatmap (DBSCAN Clustering)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
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

        # --- Collect cluster centers and weights ---
        cluster_centers = []
        cluster_strengths = []
        for lbl in set(labels):
            if lbl == -1:
                continue
            cluster_pts = points[labels == lbl]
            cluster_centers.append(cluster_pts.mean(axis=0))
            cluster_strengths.append(len(cluster_pts))

        # --- Build local heatmap from clusters ---
        local = np.zeros_like(self.heatmap)
        for (cx, cy), strength in zip(cluster_centers, cluster_strengths):
            gx = int((cx + (self.grid_size * self.resolution / 2)) / self.resolution)
            gy = int((cy + (self.grid_size * self.resolution / 2)) / self.resolution)
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                local[gy, gx] += strength * 10  # weight clusters

        # --- Smooth and update accumulated heatmap ---
        blurred = gaussian_filter(local, sigma=1.5)
        self.heatmap *= self.decay
        self.heatmap += blurred
        self.heatmap = np.clip(self.heatmap, 0, self.max_clip)

        # --- Update plot ---
        self.im.set_data(self.heatmap)
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