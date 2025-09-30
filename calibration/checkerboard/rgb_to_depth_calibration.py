import cv2
import numpy as np
import pyrealsense2 as rs
from picamera2 import Picamera2
import pickle
import os
import json
import argparse
from datetime import datetime
import glob
import time

class DepthColorCalibrator:
    def __init__(self):
        self.pattern_size = (11, 8)
        self.square_size = 30.0  # mm
        self.objpoints = []
        self.imgpoints_color = []
        self.depth_points = []  # 3D points from depth sensor
        self.images_collected = 0
        self.config_dir = "../../config"
        
        # Will store RGB camera calibration
        self.color_mtx = None
        self.color_dist = None
        
        # Ensure config directory exists
        os.makedirs(self.config_dir, exist_ok=True)
        os.makedirs('calibration_images', exist_ok=True)
        
    def setup_cameras(self):
        """Initialize both cameras"""
        # Setup RGB camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        
        # Setup RealSense with both depth and infrared
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable depth and infrared streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        
        profile = self.pipeline.start(self.config)
        
        # Get camera intrinsics
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()
        
        ir_profile = rs.video_stream_profile(profile.get_stream(rs.stream.infrared, 1))
        self.ir_intrinsics = ir_profile.get_intrinsics()
        
        # Get extrinsics between depth and infrared (factory calibrated)
        self.depth_to_ir_extrinsics = depth_profile.get_extrinsics_to(ir_profile)
        
        print("RealSense Camera Information:")
        print(f"Depth intrinsics: fx={self.depth_intrinsics.fx:.2f}, fy={self.depth_intrinsics.fy:.2f}")
        print(f"IR intrinsics: fx={self.ir_intrinsics.fx:.2f}, fy={self.ir_intrinsics.fy:.2f}")
        print(f"Depth to IR transform available: {self.depth_to_ir_extrinsics}")
        
        # Configure depth sensor
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        
        # Enable emitter for better depth measurement
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1)
            
        # Load RGB camera calibration
        self.load_color_calibration()
        
    def load_color_calibration(self):
        """Load RGB camera calibration"""
        calibration_paths = [
            'rgb_camera_calibration.pkl',
            os.path.join(self.config_dir, 'rgb_camera_calibration.pkl')
        ]
        
        for path in calibration_paths:
            try:
                with open(path, 'rb') as f:
                    rgb_calib = pickle.load(f)
                    self.color_mtx = rgb_calib['camera_matrix']
                    self.color_dist = rgb_calib['distortion_coefficients']
                    print(f"✅ RGB camera calibration loaded from {path}")
                    return
            except FileNotFoundError:
                continue
            except Exception as e:
                print(f"❌ Error loading RGB calibration from {path}: {e}")
                continue
        
        raise FileNotFoundError("RGB camera calibration not found. Run rgb_camera_calibration.py first.")
        
    def load_existing_calibration_data(self, image_dir="calibration_images"):
        """Load existing calibration images and process them"""
        print(f"Loading existing calibration data from {image_dir}...")
        
        # Find all depth-color calibration image sets
        color_files = sorted(glob.glob(os.path.join(image_dir, "depth_color_*_color.jpg")))
        ir_files = sorted(glob.glob(os.path.join(image_dir, "depth_color_*_ir.jpg")))
        depth_files = sorted(glob.glob(os.path.join(image_dir, "depth_color_*_depth.png")))
        
        if not (color_files and ir_files and depth_files):
            print("No existing depth-color calibration images found.")
            return False
            
        print(f"Found {len(color_files)} calibration sets")
        successful_pairs = 0
        
        # Setup RealSense for depth processing (needed for 3D point calculation)
        try:
            self.setup_cameras()
            time.sleep(2)  # Let cameras stabilize
        except Exception as e:
            print(f"Error setting up cameras for processing: {e}")
            return False
        
        for i, (color_file, ir_file, depth_file) in enumerate(zip(color_files, ir_files, depth_files)):
            print(f"Processing calibration pair {i+1}/{len(color_files)}...")
            
            # Load images
            color_img = cv2.imread(color_file)
            ir_img = cv2.imread(ir_file, cv2.IMREAD_GRAYSCALE)
            depth_img = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)
            
            if color_img is None or ir_img is None or depth_img is None:
                print(f"  ❌ Could not load images for pair {i+1}")
                continue
            
            # Find checkerboard corners in both images
            color_gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
            ret_color, corners_color = cv2.findChessboardCorners(color_gray, self.pattern_size, None)
            ret_ir, corners_ir = cv2.findChessboardCorners(ir_img, self.pattern_size, None)
            
            if not (ret_color and ret_ir):
                print(f"  ❌ Could not find patterns in pair {i+1}")
                continue
            
            # Refine corners
            corners_color = cv2.cornerSubPix(color_gray, corners_color, (11, 11), (-1, -1),
                                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            corners_ir = cv2.cornerSubPix(ir_img, corners_ir, (11, 11), (-1, -1),
                                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            
            # Extract 3D points from depth image (simulate depth frame)
            points_3d = []
            valid_corners_ir = []
            
            for corner in corners_ir:
                u, v = int(corner[0][0]), int(corner[0][1])
                if 0 <= u < depth_img.shape[1] and 0 <= v < depth_img.shape[0]:
                    depth_value = depth_img[v, u] / 1000.0  # Convert to meters
                    if depth_value > 0:
                        # Convert pixel + depth to 3D point
                        point_3d = rs.rs2_deproject_pixel_to_point(
                            self.depth_intrinsics, [u, v], depth_value
                        )
                        points_3d.append(point_3d)
                        valid_corners_ir.append(corner)
            
            if len(points_3d) >= len(corners_ir) * 0.9:  # At least 90% valid
                points_3d = np.array(points_3d, dtype=np.float32)
                valid_corners_ir = np.array(valid_corners_ir, dtype=np.float32)
                
                self.depth_points.append(points_3d)
                self.imgpoints_color.append(corners_color)
                successful_pairs += 1
                print(f"  ✅ Pair {i+1} processed successfully")
            else:
                print(f"  ❌ Insufficient valid depth points in pair {i+1}")
        
        self.images_collected = successful_pairs
        
        # Clean up cameras
        if hasattr(self, 'picam2'):
            self.picam2.stop()
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()
            
        print(f"\nSuccessfully loaded {successful_pairs} calibration pairs")
        return successful_pairs > 0
            
    def get_3d_checkerboard_points_from_depth(self, corners_ir, depth_frame):
        """Extract 3D coordinates of checkerboard corners using depth data"""
        if corners_ir is None or depth_frame is None:
            return None, None
            
        points_3d = []
        valid_points = []
        
        for corner in corners_ir:
            # Get pixel coordinates in IR image
            u, v = int(corner[0][0]), int(corner[0][1])
            
            # Get depth value at this pixel
            if 0 <= u < depth_frame.get_width() and 0 <= v < depth_frame.get_height():
                depth_value = depth_frame.get_distance(u, v)  # in meters
                
                if depth_value > 0:  # Valid depth
                    # Convert pixel + depth to 3D point using depth camera intrinsics
                    point_3d = rs.rs2_deproject_pixel_to_point(
                        self.depth_intrinsics, [u, v], depth_value
                    )
                    points_3d.append(point_3d)
                    valid_points.append(corner)
                    
        if len(points_3d) == len(corners_ir):
            return np.array(points_3d, dtype=np.float32), np.array(valid_points, dtype=np.float32)
        else:
            print(f"Warning: Only {len(points_3d)}/{len(corners_ir)} corners have valid depth")
            return None, None
            
    def collect_calibration_data(self):
        """Collect calibration data using checkerboard pattern"""
        print("\n=== Depth-Color Camera Calibration ===")
        print("Instructions:")
        print("- Position checkerboard where both cameras can see it clearly")
        print("- Ensure good depth measurement (avoid reflective surfaces)")
        print("- Pattern should be 0.5m - 2m from cameras")
        print("- Press 's' when pattern detected in BOTH cameras with valid depth")
        print("- Collect 20-25 high-quality pairs")
        print("- Press 'q' to finish")
        
        while True:
            # Capture frames
            color_frame = self.picam2.capture_array()
            
            # Get RealSense frames
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            ir_frame = frames.get_infrared_frame(1)
            
            if not depth_frame or not ir_frame:
                continue
                
            ir_image = np.asanyarray(ir_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Enhance IR image for corner detection
            ir_enhanced = cv2.equalizeHist(ir_image)
            
            # Find checkerboard corners
            color_gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
            ret_color, corners_color = cv2.findChessboardCorners(color_gray, self.pattern_size, None)
            ret_ir, corners_ir = cv2.findChessboardCorners(ir_enhanced, self.pattern_size, None)
            
            # Create displays - CONVERT TO BGR FOR PROPER DISPLAY
            display_color = cv2.cvtColor(color_frame, cv2.COLOR_BGRA2BGR)  # Convert XRGB to BGR
            display_ir = cv2.cvtColor(ir_enhanced, cv2.COLOR_GRAY2BGR)
            
            # Get 3D points from depth if IR corners found
            points_3d = None
            valid_depth_ratio = 0.0
            
            if ret_ir:
                points_3d, valid_corners_ir = self.get_3d_checkerboard_points_from_depth(corners_ir, depth_frame)
                if points_3d is not None:
                    valid_depth_ratio = len(points_3d) / len(corners_ir)
                    corners_ir = valid_corners_ir  # Use only corners with valid depth
                    
            # Draw corners and status
            if ret_color:
                cv2.drawChessboardCorners(display_color, self.pattern_size, corners_color, ret_color)
                cv2.putText(display_color, "Color: Pattern found", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_color, "Color: No pattern", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
            if ret_ir and points_3d is not None:
                cv2.drawChessboardCorners(display_ir, self.pattern_size, corners_ir, ret_ir)
                cv2.putText(display_ir, f"IR+Depth: {valid_depth_ratio:.1%} valid", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            elif ret_ir:
                cv2.drawChessboardCorners(display_ir, self.pattern_size, corners_ir, ret_ir)
                cv2.putText(display_ir, "IR: Pattern found, no depth", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                cv2.putText(display_ir, "IR: No pattern", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
            # Create depth visualization
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )
            
            # Create a blank image with same dimensions as depth_colormap
            blank_image = np.zeros_like(depth_colormap)
            
            # Ensure all images have the same dimensions and channels (3 channels)
            if display_color.shape[:2] != display_ir.shape[:2]:
                display_ir = cv2.resize(display_ir, (display_color.shape[1], display_color.shape[0]))
            if depth_colormap.shape[:2] != display_color.shape[:2]:
                depth_colormap = cv2.resize(depth_colormap, (display_color.shape[1], display_color.shape[0]))
                blank_image = cv2.resize(blank_image, (display_color.shape[1], display_color.shape[0]))
            
            # Combine all views
            top_row = np.hstack([display_color, display_ir])
            bottom_row = np.hstack([depth_colormap, blank_image])
            combined = np.vstack([top_row, bottom_row])
            
            # Status message
            can_save = (ret_color and ret_ir and points_3d is not None and 
                       valid_depth_ratio > 0.9)  # At least 90% corners have valid depth
                       
            if can_save:
                status_msg = f"HIGH QUALITY! Press 's' to save ({self.images_collected}/20)"
                color = (0, 255, 0)
            else:
                status_msg = f"Need both patterns with depth ({self.images_collected}/20)"
                color = (0, 0, 255)
                
            cv2.putText(combined, status_msg, (10, combined.shape[0] - 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(combined, "s=save, q=quit, r=reset", 
                       (10, combined.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow('Depth-Color Calibration (Color | IR+Depth | Depth)', combined)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') and can_save:
                # Refine corners
                corners_color = cv2.cornerSubPix(color_gray, corners_color, (11, 11), (-1, -1),
                                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                corners_ir = cv2.cornerSubPix(ir_enhanced, corners_ir, (11, 11), (-1, -1),
                                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                
                # Store data for calibration
                # We use the 3D points from depth as "object points"
                self.depth_points.append(points_3d)  # 3D points from depth sensor
                self.imgpoints_color.append(corners_color)  # 2D points in color image
                self.images_collected += 1
                
                # Save images for reference - convert back to original format for saving
                save_color_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGRA2BGR)
                os.makedirs('calibration_images', exist_ok=True)
                cv2.imwrite(f'calibration_images/depth_color_{self.images_collected:03d}_color.jpg', save_color_frame)
                cv2.imwrite(f'calibration_images/depth_color_{self.images_collected:03d}_ir.jpg', ir_enhanced)
                cv2.imwrite(f'calibration_images/depth_color_{self.images_collected:03d}_depth.png', depth_image)
                
                print(f"Saved pair {self.images_collected} (depth ratio: {valid_depth_ratio:.1%})")
                
            elif key == ord('r'):
                self.depth_points = []
                self.imgpoints_color = []
                self.images_collected = 0
                print("Reset calibration data")
                
            elif key == ord('q'):
                if self.images_collected >= 15:
                    break
                else:
                    print(f"Need at least 15 pairs, you have {self.images_collected}")
        
        self.picam2.stop()
        self.pipeline.stop()
        cv2.destroyAllWindows()
        
    def save_calibration_results(self, R, rvec, tvec, mean_error, std_error, errors, baseline, rotation_angle, all_depth_points, all_color_points):
        """Save calibration results in multiple formats"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 1. Save comprehensive calibration data as PKL
        calibration_data = {
            'rotation_matrix': R,
            'rotation_vector': rvec,
            'translation_vector': tvec,
            'color_camera_matrix': self.color_mtx,
            'color_distortion': self.color_dist,
            'depth_camera_intrinsics': {
                'fx': self.depth_intrinsics.fx,
                'fy': self.depth_intrinsics.fy,
                'cx': self.depth_intrinsics.ppx,
                'cy': self.depth_intrinsics.ppy,
                'coeffs': self.depth_intrinsics.coeffs
            },
            'ir_camera_intrinsics': {
                'fx': self.ir_intrinsics.fx,
                'fy': self.ir_intrinsics.fy,
                'cx': self.ir_intrinsics.ppx,
                'cy': self.ir_intrinsics.ppy,
                'coeffs': self.ir_intrinsics.coeffs
            },
            'reprojection_error_mean': mean_error,
            'reprojection_error_std': std_error,
            'reprojection_errors': errors,
            'baseline_distance': baseline,
            'rotation_angle_degrees': rotation_angle,
            'num_calibration_points': len(all_depth_points),
            'calibration_pairs': len(self.depth_points),
            'pattern_size': self.pattern_size,
            'square_size': self.square_size,
            'timestamp': timestamp
        }
        
        # Save locally and in config
        pkl_paths = [
            'rgb_to_depth_calibration.pkl',
            os.path.join(self.config_dir, 'rgb_to_depth_calibration.pkl')
        ]
        
        for pkl_path in pkl_paths:
            with open(pkl_path, 'wb') as f:
                pickle.dump(calibration_data, f)
                
        # 2. Save as XML file (OpenCV format)
        xml_paths = [
            'rgb_to_depth_calibration.xml',
            os.path.join(self.config_dir, 'rgb_to_depth_calibration.xml')
        ]
        
        for xml_path in xml_paths:
            fs = cv2.FileStorage(xml_path, cv2.FILE_STORAGE_WRITE)
            fs.write("rotation_matrix", R)
            fs.write("rotation_vector", rvec)
            fs.write("translation_vector", tvec)
            fs.write("color_camera_matrix", self.color_mtx)
            fs.write("color_distortion", self.color_dist)
            fs.write("depth_fx", self.depth_intrinsics.fx)
            fs.write("depth_fy", self.depth_intrinsics.fy)
            fs.write("depth_cx", self.depth_intrinsics.ppx)
            fs.write("depth_cy", self.depth_intrinsics.ppy)
            fs.write("reprojection_error_mean", mean_error)
            fs.write("reprojection_error_std", std_error)
            fs.write("baseline_distance", baseline)
            fs.write("rotation_angle_degrees", rotation_angle)
            fs.write("num_calibration_points", len(all_depth_points))
            fs.write("calibration_pairs", len(self.depth_points))
            fs.write("pattern_size_width", self.pattern_size[0])
            fs.write("pattern_size_height", self.pattern_size[1])
            fs.write("square_size", self.square_size)
            fs.write("timestamp", timestamp)
            fs.release()
        
        # 3. Save as JSON file
        json_data = {
            'rotation_matrix': R.tolist(),
            'rotation_vector': rvec.tolist(),
            'translation_vector': tvec.tolist(),
            'color_camera_matrix': self.color_mtx.tolist(),
            'color_distortion': self.color_dist.tolist(),
            'depth_camera_intrinsics': {
                'fx': float(self.depth_intrinsics.fx),
                'fy': float(self.depth_intrinsics.fy),
                'cx': float(self.depth_intrinsics.ppx),
                'cy': float(self.depth_intrinsics.ppy),
                'coeffs': [float(c) for c in self.depth_intrinsics.coeffs]
            },
            'ir_camera_intrinsics': {
                'fx': float(self.ir_intrinsics.fx),
                'fy': float(self.ir_intrinsics.fy),
                'cx': float(self.ir_intrinsics.ppx),
                'cy': float(self.ir_intrinsics.ppy),
                'coeffs': [float(c) for c in self.ir_intrinsics.coeffs]
            },
            'reprojection_error_mean': float(mean_error),
            'reprojection_error_std': float(std_error),
            'baseline_distance': float(baseline),
            'rotation_angle_degrees': float(rotation_angle),
            'num_calibration_points': len(all_depth_points),
            'calibration_pairs': len(self.depth_points),
            'pattern_size': list(self.pattern_size),
            'square_size': float(self.square_size),
            'timestamp': timestamp,
            'calibration_type': 'Depth_Color_Stereo'
        }
        
        json_paths = [
            'rgb_to_depth_calibration.json',
            os.path.join(self.config_dir, 'rgb_to_depth_calibration.json')
        ]
        
        for json_path in json_paths:
            with open(json_path, 'w') as f:
                json.dump(json_data, f, indent=2)
        
        # 4. Save detailed summary as TXT file
        summary = f"""Depth-Color Camera Calibration Summary
=========================================
Timestamp: {timestamp}
Calibration Type: Depth-Color Stereo (RealSense D435i + Raspberry Pi Camera)

Pattern Information:
- Pattern Size: {self.pattern_size[0]} x {self.pattern_size[1]} corners
- Square Size: {self.square_size} mm
- Number of Calibration Pairs: {len(self.depth_points)}
- Total Calibration Points: {len(all_depth_points)}

Camera Information:
Depth Camera (RealSense):
- Focal Length: fx={self.depth_intrinsics.fx:.2f}, fy={self.depth_intrinsics.fy:.2f}
- Principal Point: cx={self.depth_intrinsics.ppx:.2f}, cy={self.depth_intrinsics.ppy:.2f}

Color Camera (Raspberry Pi):
- Camera Matrix:
{self.color_mtx}
- Distortion Coefficients: {self.color_dist.flatten()}

Calibration Results:
- Mean Reprojection Error: {mean_error:.4f} ± {std_error:.4f} pixels
- Maximum Reprojection Error: {np.max(errors):.4f} pixels
- Baseline Distance: {baseline*1000:.2f} mm
- Rotation Angle: {rotation_angle:.2f} degrees

Extrinsic Parameters (Depth → Color):
Rotation Matrix:
{R}

Translation Vector (meters):
{tvec.flatten()}

Rotation Vector:
{rvec.flatten()}

Quality Assessment:
- Mean Error < 1.0 pixel: {'✅ Excellent' if mean_error < 1.0 else ('⚠️ Acceptable' if mean_error < 2.0 else '❌ Poor')}
- Standard Deviation < 0.5: {'✅ Consistent' if std_error < 0.5 else ('⚠️ Moderate' if std_error < 1.0 else '❌ Inconsistent')}
- Number of Pairs >= 20: {'✅ Good' if len(self.depth_points) >= 20 else ('⚠️ Minimal' if len(self.depth_points) >= 15 else '❌ Insufficient')}

Transformation Summary:
- The depth camera is positioned {baseline*1000:.1f}mm from the color camera
- The depth camera is rotated {rotation_angle:.1f}° relative to the color camera
- Translation: X={tvec[0][0]*1000:.1f}mm, Y={tvec[1][0]*1000:.1f}mm, Z={tvec[2][0]*1000:.1f}mm

Usage Notes:
- Use this calibration to transform 3D points from depth camera coordinates to color camera coordinates
- Apply: P_color = R * P_depth + T
- For projecting depth points to color image: use projectPoints() with color camera matrix

Files Generated:
- rgb_to_depth_calibration.pkl (local & config/)
- rgb_to_depth_calibration.xml (local & config/)
- rgb_to_depth_calibration.json (local & config/)
- rgb_to_depth_calibration_summary.txt (local & config/)

Error Statistics:
- Minimum Error: {np.min(errors):.4f} pixels
- 25th Percentile: {np.percentile(errors, 25):.4f} pixels
- Median Error: {np.median(errors):.4f} pixels
- 75th Percentile: {np.percentile(errors, 75):.4f} pixels
- 95th Percentile: {np.percentile(errors, 95):.4f} pixels

Note: All calibration files have been saved both locally and in the ../../config directory.
"""
        
        summary_paths = [
            'rgb_to_depth_calibration_summary.txt',
            os.path.join(self.config_dir, 'rgb_to_depth_calibration_summary.txt')
        ]
        
        for summary_path in summary_paths:
            with open(summary_path, 'w') as f:
                f.write(summary)
        
        print(f"\n✅ Depth-Color calibration results saved in multiple formats:")
        print(f"   - PKL files: local & {self.config_dir}/")
        print(f"   - XML files: local & {self.config_dir}/")  
        print(f"   - JSON files: local & {self.config_dir}/")
        print(f"   - Summary: local & {self.config_dir}/")
        
    def perform_rgb_to_depth_calibration(self):
        """Perform calibration between depth sensor coordinate system and color camera"""
        if len(self.depth_points) < 15:
            print("Not enough calibration pairs!")
            return False
            
        print(f"Performing depth-color calibration with {len(self.depth_points)} pairs...")
        
        # Convert depth points and image points to the format needed by solvePnP
        all_depth_points = []
        all_color_points = []
        
        for i in range(len(self.depth_points)):
            depth_pts = self.depth_points[i]  # 3D points in depth camera coordinate system
            color_pts = self.imgpoints_color[i]  # 2D points in color camera
            
            # Convert to meters (RealSense gives points in meters)
            depth_pts_m = depth_pts.reshape(-1, 3).astype(np.float32)
            color_pts_px = color_pts.reshape(-1, 2).astype(np.float32)
            
            all_depth_points.extend(depth_pts_m)
            all_color_points.extend(color_pts_px)
            
        all_depth_points = np.array(all_depth_points, dtype=np.float32)
        all_color_points = np.array(all_color_points, dtype=np.float32)
        
        print(f"Total points for calibration: {len(all_depth_points)}")
        
        # Solve for the transformation from depth camera to color camera coordinate system
        # This gives us the rotation and translation between the two cameras
        success, rvec, tvec = cv2.solvePnP(
            all_depth_points, all_color_points, 
            self.color_mtx, self.color_dist
        )
        
        if success:
            # Convert rotation vector to rotation matrix
            R, _ = cv2.Rodrigues(rvec)
            
            # Calculate reprojection error
            projected_points, _ = cv2.projectPoints(
                all_depth_points, rvec, tvec, self.color_mtx, self.color_dist
            )
            projected_points = projected_points.reshape(-1, 2)
            
            errors = np.sqrt(np.sum((all_color_points - projected_points) ** 2, axis=1))
            mean_error = np.mean(errors)
            std_error = np.std(errors)
            
            print(f"✅ Depth-Color calibration successful!")
            print(f"Reprojection error: {mean_error:.3f} ± {std_error:.3f} pixels")
            print(f"Max error: {np.max(errors):.3f} pixels")
            
            # Calculate additional metrics
            baseline = np.linalg.norm(tvec)
            rotation_angle = np.arccos((np.trace(R) - 1) / 2) * 180 / np.pi
            
            print(f"Baseline distance: {baseline*1000:.2f} mm")
            print(f"Rotation angle: {rotation_angle:.2f} degrees")
            
            # Save comprehensive calibration results
            self.save_calibration_results(R, rvec, tvec, mean_error, std_error, errors, 
                                        baseline, rotation_angle, all_depth_points, all_color_points)
            
            return True
        else:
            print("❌ Depth-color calibration failed!")
            return False

def main():
    parser = argparse.ArgumentParser(description='Depth-Color Camera Calibration')
    parser.add_argument('--load-images', action='store_true', 
                       help='Load existing calibration images instead of capturing new ones')
    parser.add_argument('--image-dir', default='calibration_images',
                       help='Directory containing existing calibration images')
    
    args = parser.parse_args()
    
    os.makedirs('calibration_images', exist_ok=True)
    
    try:
        calibrator = DepthColorCalibrator()
        
        if args.load_images:
            # Load existing images and process them
            if calibrator.load_existing_calibration_data(args.image_dir):
                calibrator.perform_rgb_to_depth_calibration()
            else:
                print("Failed to load existing calibration data. Exiting.")
        else:
            # Capture new images
            calibrator.setup_cameras()
            calibrator.collect_calibration_data()
            calibrator.perform_rgb_to_depth_calibration()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("Make sure you have run: python3 rgb_camera_calibration.py")

if __name__ == "__main__":
    main()