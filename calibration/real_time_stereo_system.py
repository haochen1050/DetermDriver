import cv2
import numpy as np
import pyrealsense2 as rs
from picamera2 import Picamera2
import pickle
import threading
import time
import open3d as o3d

class RealTimeStereoSystem:
    def __init__(self):
        self.rgb_frame = None
        self.depth_frame = None
        self.point_cloud = None
        self.rgb_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.pc_lock = threading.Lock()
        
        # Load calibration data
        self.load_calibrations()
        
    def load_calibrations(self):
        """Load all calibration data"""
        try:
            # Load stereo calibration
            with open('stereo_calibration.pkl', 'rb') as f:
                self.stereo_calib = pickle.load(f)
            
            # Extract calibration parameters
            self.R = self.stereo_calib['rotation_matrix']
            self.T = self.stereo_calib['translation_vector']
            self.Q = self.stereo_calib['disparity_to_depth_mapping_matrix']
            self.map1_rgb = self.stereo_calib['rectification_map_1_rgb']
            self.map2_rgb = self.stereo_calib['rectification_map_2_rgb']
            self.map1_depth = self.stereo_calib['rectification_map_1_depth']
            self.map2_depth = self.stereo_calib['rectification_map_2_depth']
            self.baseline = self.stereo_calib['baseline']
            
            print("Stereo calibration loaded successfully")
            print(f"Baseline: {self.baseline:.2f} mm")
            
        except FileNotFoundError:
            print("Stereo calibration file not found!")
            print("Please run stereo_calibration.py first")
            return False
            
        return True
        
    def setup_cameras(self):
        """Initialize both cameras"""
        # Setup RGB camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        
        # Setup RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        
        # Get depth sensor for setting parameters
        device = self.pipeline.get_active_profile().get_device()
        depth_sensor = device.first_depth_sensor()
        
        # Set depth parameters for better performance
        if depth_sensor.supports(rs.option.visual_preset):
            depth_sensor.set_option(rs.option.visual_preset, 4)  # High accuracy preset
            
        print("Cameras initialized successfully")
        
    def capture_rgb_frames(self):
        """Capture and process RGB frames"""
        while self.capturing:
            try:
                # Capture frame
                frame = self.picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                # Apply rectification
                rectified_rgb = cv2.remap(frame_bgr, self.map1_rgb, self.map2_rgb, cv2.INTER_LINEAR)
                
                with self.rgb_lock:
                    self.rgb_frame = rectified_rgb.copy()
                    
            except Exception as e:
                print(f"RGB capture error: {e}")
                
            time.sleep(0.01)
            
    def capture_depth_frames(self):
        """Capture and process depth frames"""
        while self.capturing:
            try:
                # Get frames
                frames = self.pipeline.wait_for_frames(timeout_ms=100)
                depth_frame = frames.get_depth_frame()
                
                if depth_frame:
                    # Apply filters for better depth quality
                    depth_frame = self.apply_depth_filters(depth_frame)
                    
                    # Convert to numpy array
                    depth_image = np.asanyarray(depth_frame.get_data())
                    
                    # Apply rectification
                    rectified_depth = cv2.remap(depth_image, self.map1_depth, self.map2_depth, cv2.INTER_LINEAR)
                    
                    with self.depth_lock:
                        self.depth_frame = rectified_depth.copy()
                        
            except Exception as e:
                print(f"Depth capture error: {e}")
                
            time.sleep(0.01)
            
    def apply_depth_filters(self, depth_frame):
        """Apply filters to improve depth data quality"""
        # Decimation filter - reduces depth frame density
        decimation = rs.decimation_filter()
        decimation.set_option(rs.option.filter_magnitude, 1)
        
        # Spatial filter - edge-preserving smoothing
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, 2)
        spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        spatial.set_option(rs.option.filter_smooth_delta, 20)
        
        # Temporal filter - reduces noise
        temporal = rs.temporal_filter()
        temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
        temporal.set_option(rs.option.filter_smooth_delta, 20)
        
        # Apply filters
        depth_frame = decimation.process(depth_frame)
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        
        return depth_frame
        
    def generate_point_cloud(self):
        """Generate point cloud from depth data"""
        while self.capturing:
            try:
                with self.depth_lock:
                    current_depth = self.depth_frame.copy() if self.depth_frame is not None else None
                    
                with self.rgb_lock:
                    current_rgb = self.rgb_frame.copy() if self.rgb_frame is not None else None
                    
                if current_depth is not None and current_rgb is not None:
                    # Create point cloud
                    points_3d = self.depth_to_points(current_depth)
                    
                    # Color the points
                    colors = current_rgb.reshape(-1, 3) / 255.0
                    
                    # Filter out invalid points
                    valid_mask = (points_3d[:, 2] > 0) & (points_3d[:, 2] < 5000)  # 5m max distance
                    valid_points = points_3d[valid_mask]
                    valid_colors = colors[valid_mask]
                    
                    # Create Open3D point cloud
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(valid_points)
                    pcd.colors = o3d.utility.Vector3dVector(valid_colors)
                    
                    with self.pc_lock:
                        self.point_cloud = pcd
                        
            except Exception as e:
                print(f"Point cloud generation error: {e}")
                
            time.sleep(0.05)  # Generate at 20 FPS
            
    def depth_to_points(self, depth_image):
        """Convert depth image to 3D points"""
        height, width = depth_image.shape
        
        # Create coordinate grids
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        u = u.flatten()
        v = v.flatten()
        depth = depth_image.flatten()
        
        # Convert to 3D points using camera intrinsics
        # Using RealSense intrinsics from calibration
        fx = self.stereo_calib['depth_camera_matrix'][0, 0]
        fy = self.stereo_calib['depth_camera_matrix'][1, 1]
        cx = self.stereo_calib['depth_camera_matrix'][0, 2]
        cy = self.stereo_calib['depth_camera_matrix'][1, 2]
        
        # Convert pixel coordinates to 3D points
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        points_3d = np.column_stack([x, y, z])
        
        # Transform to RGB camera coordinate system
        # Apply rotation and translation from stereo calibration
        points_3d_transformed = np.dot(points_3d, self.R.T) + self.T.T
        
        return points_3d_transformed
        
    def display_results(self):
        """Display real-time results"""
        print("Real-time stereo system running...")
        print("Controls:")
        print("  'q' - Quit")
        print("  's' - Save current point cloud")
        print("  'r' - Reset view")
        
        # Initialize Open3D visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window("Point Cloud", width=800, height=600)
        
        # Add coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
        vis.add_geometry(coord_frame)
        
        pcd_added = False
        frame_count = 0
        
        while self.capturing:
            try:
                # Get current frames
                with self.rgb_lock:
                    current_rgb = self.rgb_frame.copy() if self.rgb_frame is not None else None
                    
                with self.depth_lock:
                    current_depth = self.depth_frame.copy() if self.depth_frame is not None else None
                    
                with self.pc_lock:
                    current_pc = self.point_cloud
                    
                # Display RGB and depth images
                if current_rgb is not None:
                    cv2.imshow('RGB Camera (Rectified)', current_rgb)
                    
                if current_depth is not None:
                    # Convert depth to colormap for display
                    depth_colormap = cv2.applyColorMap(
                        cv2.convertScaleAbs(current_depth, alpha=0.03), cv2.COLORMAP_JET
                    )
                    cv2.imshow('Depth Camera (Rectified)', depth_colormap)
                
                # Update point cloud visualization
                if current_pc is not None and len(current_pc.points) > 0:
                    if not pcd_added:
                        vis.add_geometry(current_pc)
                        pcd_added = True
                    else:
                        vis.update_geometry(current_pc)
                    
                    vis.poll_events()
                    vis.update_renderer()
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.capturing = False
                    break
                elif key == ord('s') and current_pc is not None:
                    filename = f"point_cloud_{int(time.time())}.ply"
                    o3d.io.write_point_cloud(filename, current_pc)
                    print(f"Point cloud saved as {filename}")
                elif key == ord('r'):
                    vis.reset_view_point(True)
                
                frame_count += 1
                if frame_count % 30 == 0:  # Print FPS every 30 frames
                    print(f"Processing frame {frame_count}")
                    
            except Exception as e:
                print(f"Display error: {e}")
                
            time.sleep(0.01)
        
        vis.destroy_window()
        cv2.destroyAllWindows()
        
    def run(self):
        """Run the real-time stereo system"""
        if not self.load_calibrations():
            return
            
        self.setup_cameras()
        
        # Start capture threads
        self.capturing = True
        
        rgb_thread = threading.Thread(target=self.capture_rgb_frames)
        depth_thread = threading.Thread(target=self.capture_depth_frames)
        pc_thread = threading.Thread(target=self.generate_point_cloud)
        
        rgb_thread.start()
        depth_thread.start()
        pc_thread.start()
        
        # Start display
        self.display_results()
        
        # Wait for threads to finish
        rgb_thread.join()
        depth_thread.join()
        pc_thread.join()
        
        # Cleanup
        self.picam2.stop()
        self.pipeline.stop()
        
        print("System shutdown complete")

def main():
    system = RealTimeStereoSystem()
    system.run()

if __name__ == "__main__":
    main()