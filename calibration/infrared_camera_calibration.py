import cv2
import numpy as np
import pyrealsense2 as rs
import pickle
import os

class InfraredCameraCalibrator:
    def __init__(self):
        self.pattern_size = (10, 7)  # Internal corners
        self.square_size = 25.0  # mm
        self.objpoints = []
        self.imgpoints = []
        self.images_collected = 0
        
    def setup_camera(self):
        """Initialize RealSense infrared camera"""
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure infrared stream (using infrared 1)
        # Use 640x480 resolution for consistency with RGB camera
        self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        
        # Also enable depth for later use
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Start pipeline
        profile = self.pipeline.start(self.config)
        
        # Get infrared camera intrinsics
        ir_profile = rs.video_stream_profile(profile.get_stream(rs.stream.infrared, 1))
        self.ir_intrinsics = ir_profile.get_intrinsics()
        
        # Get depth camera intrinsics for reference
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()
        
        print(f"RealSense D421 Camera Information:")
        print(f"Infrared Camera Intrinsics:")
        print(f"  Width: {self.ir_intrinsics.width}")
        print(f"  Height: {self.ir_intrinsics.height}")
        print(f"  fx: {self.ir_intrinsics.fx:.2f}")
        print(f"  fy: {self.ir_intrinsics.fy:.2f}")
        print(f"  cx: {self.ir_intrinsics.ppx:.2f}")
        print(f"  cy: {self.ir_intrinsics.ppy:.2f}")
        print(f"  Distortion: {self.ir_intrinsics.coeffs}")
        
        # Set infrared emitter for better pattern visibility
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        
        # Turn on infrared emitter for better visibility
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1)
            print("Infrared emitter enabled")
            
        # Adjust infrared exposure for better pattern detection
        if depth_sensor.supports(rs.option.exposure):
            # Set exposure to auto or a good value
            depth_sensor.set_option(rs.option.exposure, 8500)  # Adjust as needed
            print("Infrared exposure set")
        
    def prepare_object_points(self):
        """Prepare 3D object points"""
        self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
    def collect_calibration_images(self):
        """Collect calibration images from infrared camera"""
        print("Infrared Camera Calibration (RealSense D421)")
        print("Instructions:")
        print("- The checkerboard pattern should be clearly visible in IR")
        print("- Place checkerboard at different distances (0.5m - 3m)")
        print("- Try different angles and orientations")
        print("- Press 's' when pattern is detected to save")
        print("- Collect 15-20 images")
        print("- Press 'q' to finish, 'r' to reset")
        print("- Press 'e' to toggle emitter on/off")
        
        emitter_enabled = True
        
        while True:
            # Get frames
            frames = self.pipeline.wait_for_frames()
            ir_frame = frames.get_infrared_frame(1)  # Get infrared 1
            depth_frame = frames.get_depth_frame()
            
            if not ir_frame:
                continue
                
            # Convert infrared frame to numpy array
            ir_image = np.asanyarray(ir_frame.get_data())
            
            # Enhance infrared image for better visualization
            # Apply histogram equalization
            ir_enhanced = cv2.equalizeHist(ir_image)
            
            # Convert to 3-channel for display and corner detection
            ir_display = cv2.cvtColor(ir_enhanced, cv2.COLOR_GRAY2BGR)
            
            # Find chessboard corners on the enhanced IR image
            ret, corners = cv2.findChessboardCorners(ir_enhanced, self.pattern_size, 
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                                   cv2.CALIB_CB_NORMALIZE_IMAGE)
            
            # Draw corners if found
            if ret:
                # Refine corners on the original (non-enhanced) image
                corners = cv2.cornerSubPix(ir_image, corners, (11, 11), (-1, -1),
                                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                
                # Draw corners on display image
                cv2.drawChessboardCorners(ir_display, self.pattern_size, corners, ret)
                cv2.putText(ir_display, f"Pattern found! Press 's' to save ({self.images_collected}/15)", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(ir_display, f"No pattern detected ({self.images_collected}/15)", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Add emitter status
            emitter_status = "ON" if emitter_enabled else "OFF"
            cv2.putText(ir_display, f"IR Emitter: {emitter_status}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.putText(ir_display, "s=save, q=quit, r=reset, e=toggle emitter", 
                       (10, ir_display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Show depth reference (optional)
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
                )
                
                # Combine IR and depth for reference
                combined = np.hstack([ir_display, depth_colormap])
                cv2.imshow('Infrared Camera Calibration (IR + Depth Reference)', combined)
            else:
                cv2.imshow('Infrared Camera Calibration', ir_display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') and ret:
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners)
                self.images_collected += 1
                print(f"Saved image {self.images_collected}")
                
                # Save images for reference
                cv2.imwrite(f'calibration_images/ir_calib_{self.images_collected}.jpg', ir_display)
                cv2.imwrite(f'calibration_images/ir_raw_{self.images_collected}.png', ir_image)
                
            elif key == ord('r'):
                self.objpoints = []
                self.imgpoints = []
                self.images_collected = 0
                print("Reset calibration data")
                
            elif key == ord('e'):
                # Toggle emitter
                device = self.pipeline.get_active_profile().get_device()
                depth_sensor = device.first_depth_sensor()
                if depth_sensor.supports(rs.option.emitter_enabled):
                    emitter_enabled = not emitter_enabled
                    depth_sensor.set_option(rs.option.emitter_enabled, 1 if emitter_enabled else 0)
                    print(f"Infrared emitter {'enabled' if emitter_enabled else 'disabled'}")
                
            elif key == ord('q'):
                if self.images_collected >= 10:
                    break
                else:
                    print(f"Need at least 10 images, you have {self.images_collected}")
        
        self.pipeline.stop()
        cv2.destroyAllWindows()
        
    def calibrate_camera(self):
        """Perform infrared camera calibration"""
        if len(self.objpoints) < 10:
            print("Not enough calibration images!")
            return None
            
        print("Performing infrared camera calibration...")
        
        # Get image dimensions
        img_shape = (640, 480)
        
        # Use RealSense intrinsics as initial guess
        initial_camera_matrix = np.array([
            [self.ir_intrinsics.fx, 0, self.ir_intrinsics.ppx],
            [0, self.ir_intrinsics.fy, self.ir_intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        initial_dist_coeffs = np.array(self.ir_intrinsics.coeffs, dtype=np.float32)
        
        print("Using RealSense intrinsics as initial guess:")
        print(f"Initial camera matrix:\n{initial_camera_matrix}")
        print(f"Initial distortion:\n{initial_dist_coeffs}")
        
        # Calibrate camera with initial guess
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, img_shape, 
            initial_camera_matrix, initial_dist_coeffs,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS
        )
        
        if ret:
            # Calculate reprojection error
            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error
            
            mean_error /= len(self.objpoints)
            print(f"Mean reprojection error: {mean_error:.3f} pixels")
            
            # Save calibration results
            calibration_data = {
                'camera_matrix': mtx,
                'distortion_coefficients': dist,
                'rotation_vectors': rvecs,
                'translation_vectors': tvecs,
                'reprojection_error': mean_error,
                'image_shape': img_shape,
                'realsense_ir_intrinsics': {
                    'width': self.ir_intrinsics.width,
                    'height': self.ir_intrinsics.height,
                    'fx': self.ir_intrinsics.fx,
                    'fy': self.ir_intrinsics.fy,
                    'cx': self.ir_intrinsics.ppx,
                    'cy': self.ir_intrinsics.ppy,
                    'coeffs': self.ir_intrinsics.coeffs
                },
                'realsense_depth_intrinsics': {
                    'width': self.depth_intrinsics.width,
                    'height': self.depth_intrinsics.height,
                    'fx': self.depth_intrinsics.fx,
                    'fy': self.depth_intrinsics.fy,
                    'cx': self.depth_intrinsics.ppx,
                    'cy': self.depth_intrinsics.ppy
                }
            }
            
            with open('depth_camera_calibration.pkl', 'wb') as f:
                pickle.dump(calibration_data, f)
            
            print("Infrared camera calibration completed and saved!")
            print(f"Refined camera matrix:\n{mtx}")
            print(f"Refined distortion coefficients:\n{dist}")
            print(f"Difference from RealSense intrinsics:")
            print(f"  fx: {mtx[0,0]:.2f} vs {self.ir_intrinsics.fx:.2f} (diff: {mtx[0,0] - self.ir_intrinsics.fx:.2f})")
            print(f"  fy: {mtx[1,1]:.2f} vs {self.ir_intrinsics.fy:.2f} (diff: {mtx[1,1] - self.ir_intrinsics.fy:.2f})")
            print(f"  cx: {mtx[0,2]:.2f} vs {self.ir_intrinsics.ppx:.2f} (diff: {mtx[0,2] - self.ir_intrinsics.ppx:.2f})")
            print(f"  cy: {mtx[1,2]:.2f} vs {self.ir_intrinsics.ppy:.2f} (diff: {mtx[1,2] - self.ir_intrinsics.ppy:.2f})")
            
            return mtx, dist
        else:
            print("Infrared camera calibration failed!")
            return None

def main():
    # Create directory for calibration images
    os.makedirs('calibration_images', exist_ok=True)
    
    calibrator = InfraredCameraCalibrator()
    calibrator.setup_camera()
    calibrator.prepare_object_points()
    calibrator.collect_calibration_images()
    calibrator.calibrate_camera()

if __name__ == "__main__":
    main()