# scripts/heat_map_generator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

# optional: use gaussian smoothing if scipy is available
try:
    from scipy.ndimage import gaussian_filter
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False
    def gaussian_filter(x, sigma):
        # very small fallback: simple 3x3 average (avoids scipy dependency)
        kernel = np.ones((3, 3)) / 9.0
        from scipy.signal import convolve2d as _conv  # will still error if scipy missing
        return _conv(x, kernel, mode='same', boundary='fill', fillvalue=0)

class HeatmapGenerator(Node):
    def __init__(self):
        super().__init__('heatmap_generator')

        # --- Params you can tune ---
        self.topic = '/scan'
        self.grid_size = 400            # cells per side (400 -> 20 m if resolution=0.05)
        self.resolution = 0.05          # meters per cell
        self.decay = 0.98               # per-frame multiplicative decay (less than 1)
        self.smooth_sigma = 1.5         # gaussian smoothing sigma (0 = no smoothing)
        self.max_clip = 100.0           # clip maximum heat value for color scaling
        self.debug = False              # set True to get printed debug info

        # Derived
        half_m = (self.grid_size * self.resolution) / 2.0
        self.x_min, self.x_max = -half_m, half_m
        self.y_min, self.y_max = -half_m, half_m

        # Main heatmap (accumulated)
        self.heatmap = np.zeros((self.grid_size, self.grid_size), dtype=float)

        # ROS subscription
        self.subscription = self.create_subscription(LaserScan, self.topic, self.scan_callback, 10)

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.im = self.ax.imshow(
            self.heatmap,
            cmap='hot',
            origin='lower',
            interpolation='nearest',
            extent=[self.x_min, self.x_max, self.y_min, self.y_max],
            vmin=0.0, vmax=self.max_clip
        )
        self.ax.set_title("LiDAR Heatmap")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.cbar = self.fig.colorbar(self.im, ax=self.ax)
        self.cbar.set_label('Heat (hits)')
        plt.show(block=False)

        self.get_logger().info(f"Heatmap node started. listening to {self.topic}. grid_size={self.grid_size}, resolution={self.resolution} m/cell")

    def scan_callback(self, msg: LaserScan):
        # --- 1) Build angles and ranges safely ---
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # filter invalid readings
        valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(valid_mask):
            # nothing to add this frame; still apply decay so old heat fades slowly
            self.heatmap *= self.decay
            # update display and return
            self._update_display()
            if self.debug:
                self.get_logger().info("No valid ranges in this scan.")
            return

        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        # Cartesian points in lidar frame
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        if self.debug:
            self.get_logger().info(f"valid hits: {len(xs)} | x range: {xs.min():.2f}..{xs.max():.2f} | y range: {ys.min():.2f}..{ys.max():.2f}")

        # --- 2) Bin points into 2D histogram that matches heatmap grid ---
        # numpy.histogram2d expects x values first, then y.
        bins = (self.grid_size, self.grid_size)
        H, xedges, yedges = np.histogram2d(
            xs, ys,
            bins=bins,
            range=[[self.x_min, self.x_max], [self.y_min, self.y_max]]
        )

        # H has shape (nx_bins, ny_bins). We want (rows, cols) = (y bins, x bins)
        # So transpose H to match our heatmap indexing [row(y), col(x)]
        local = H.T  # now local.shape == (grid_size, grid_size)

        # --- 3) Smooth local hits so clusters become blocks/blobs ---
        if self.smooth_sigma > 0:
            if _HAS_SCIPY:
                local_blurred = gaussian_filter(local, sigma=self.smooth_sigma)
            else:
                # fallback: small (no-op) blur using a very small kernel implemented with numpy
                # (this is cheap and avoids SciPy if not present)
                kernel = np.array([[0.05, 0.1, 0.05],
                                   [0.1,  0.4, 0.1 ],
                                   [0.05, 0.1, 0.05]])
                # simple convolution:
                from scipy.signal import convolve2d
                try:
                    local_blurred = convolve2d(local, kernel, mode='same', boundary='fill', fillvalue=0)
                except Exception:
                    # as last resort, skip smoothing
                    local_blurred = local
        else:
            local_blurred = local

        # --- 4) Update accumulated heatmap with decay and addition ---
        # decay old heat
        self.heatmap *= self.decay
        # add this frame's contribution (scale up to emphasize clusters)
        self.heatmap += local_blurred * 5.0

        # clip to prevent runaway values and to keep colormap stable
        self.heatmap = np.clip(self.heatmap, 0.0, self.max_clip)

        # --- 5) Update visualization ---
        self._update_display()

    def _update_display(self):
        # update the image data and refresh
        self.im.set_data(self.heatmap)
        # optional: update color limits adaptively (comment/uncomment as needed)
        # self.im.set_clim(0, max(1.0, np.max(self.heatmap)))
        self.fig.canvas.flush_events()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = HeatmapGenerator()
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
