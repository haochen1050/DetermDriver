import cv2
import numpy as np
from picamera2 import Picamera2
import pickle
import os
import json
import argparse
from datetime import datetime
import glob

class RGBCameraCalibrator:
    def __init__(self):
        self.pattern_size = (11, 8)  # Internal corners
        self.square_size = 30.0  # mm - measure your actual pattern
        self.objpoints = []  # 3D points
        self.imgpoints = []  # 2D points
        self.images_collected = 0
        self.config_dir = "../../config"
        
        # Ensure config directory exists
        os.makedirs(self.config_dir, exist_ok=True)
        os.makedirs('calibration_images', exist_ok=True)
        
    def setup_camera(self):
        """Initialize Pi camera"""
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        
    def prepare_object_points(self):
        """Prepare 3D object points"""
        self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
    def load_existing_images(self, image_dir="calibration_images"):
        """Load and process existing calibration images"""
        print(f"Loading existing images from {image_dir}...")
        
        # Find all calibration images
        image_files = glob.glob(os.path.join(image_dir, "rgb_calib_*.jpg"))
        image_files.sort()
        
        if not image_files:
            print("No existing calibration images found.")
            return False
            
        print(f"Found {len(image_files)} existing images")
        successful_detections = 0
        
        for img_path in image_files:
            print(f"Processing {os.path.basename(img_path)}...")
            
            # Load image
            img = cv2.imread(img_path)
            if img is None:
                print(f"  ❌ Could not load {img_path}")
                continue
                
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            
            if ret:
                # Refine corners
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                
                self.objpoints.append(self.objp.copy())
                self.imgpoints.append(corners)
                successful_detections += 1
                print(f"  ✅ Pattern detected and added")
            else:
                print(f"  ❌ No pattern detected")
        
        self.images_collected = successful_detections
        print(f"\nSuccessfully loaded {successful_detections} calibration images")
        return successful_detections > 0
        
    def collect_calibration_images(self):
        """Collect calibration images interactively"""
        print("RGB Camera Calibration")
        print("Instructions:")
        print("- Move checkerboard to different positions and angles")
        print("- Press 's' when pattern is detected to save image")
        print("- Collect 15-20 images from different views")
        print("- Press 'q' to finish collection")
        print("- Press 'r' to reset collection")
        
        while True:
            # Capture frame
            frame = self.picam2.capture_array()
            # frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            
            # Draw corners if found
            display_frame = frame.copy()
            if ret:
                # Refine corners
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                
                # Draw corners
                cv2.drawChessboardCorners(display_frame, self.pattern_size, corners, ret)
                cv2.putText(display_frame, f"Pattern found! Press 's' to save ({self.images_collected}/15)", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, f"No pattern detected ({self.images_collected}/15)", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Display instructions
            cv2.putText(display_frame, "s=save, q=quit, r=reset", 
                       (10, display_frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow('RGB Camera Calibration', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') and ret:
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners)
                self.images_collected += 1
                print(f"Saved image {self.images_collected}")
                
                # Save the image for reference
                cv2.imwrite(f'calibration_images/rgb_calib_{self.images_collected:03d}.jpg', frame)
                
            elif key == ord('r'):
                self.objpoints = []
                self.imgpoints = []
                self.images_collected = 0
                print("Reset calibration data")
                
            elif key == ord('q'):
                if self.images_collected >= 10:
                    break
                else:
                    print(f"Need at least 10 images, you have {self.images_collected}")
        
        self.picam2.stop()
        cv2.destroyAllWindows()
        
    def save_calibration_results(self, mtx, dist, rvecs, tvecs, mean_error, img_shape):
        """Save calibration results in multiple formats"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 1. Save as PKL file (local and config)
        calibration_data = {
            'camera_matrix': mtx,
            'distortion_coefficients': dist,
            'rotation_vectors': rvecs,
            'translation_vectors': tvecs,
            'reprojection_error': mean_error,
            'image_shape': img_shape,
            'pattern_size': self.pattern_size,
            'square_size': self.square_size,
            'num_images': len(self.objpoints),
            'timestamp': timestamp
        }
        
        # Save locally
        with open('rgb_camera_calibration.pkl', 'wb') as f:
            pickle.dump(calibration_data, f)
        
        # Save in config directory
        with open(os.path.join(self.config_dir, 'rgb_camera_calibration.pkl'), 'wb') as f:
            pickle.dump(calibration_data, f)
            
        # 2. Save as XML file (OpenCV format)
        xml_path_local = 'rgb_camera_calibration.xml'
        xml_path_config = os.path.join(self.config_dir, 'rgb_camera_calibration.xml')
        
        for xml_path in [xml_path_local, xml_path_config]:
            fs = cv2.FileStorage(xml_path, cv2.FILE_STORAGE_WRITE)
            fs.write("camera_matrix", mtx)
            fs.write("distortion_coefficients", dist)
            fs.write("reprojection_error", mean_error)
            fs.write("image_width", img_shape[0])
            fs.write("image_height", img_shape[1])
            fs.write("pattern_size_width", self.pattern_size[0])
            fs.write("pattern_size_height", self.pattern_size[1])
            fs.write("square_size", self.square_size)
            fs.write("num_images", len(self.objpoints))
            fs.write("timestamp", timestamp)
            fs.release()
        
        # 3. Save as JSON file
        json_data = {
            'camera_matrix': mtx.tolist(),
            'distortion_coefficients': dist.tolist(),
            'reprojection_error': float(mean_error),
            'image_shape': list(img_shape),
            'pattern_size': list(self.pattern_size),
            'square_size': float(self.square_size),
            'num_images': len(self.objpoints),
            'timestamp': timestamp,
            'calibration_type': 'RGB_Camera'
        }
        
        json_path_local = 'rgb_camera_calibration.json'
        json_path_config = os.path.join(self.config_dir, 'rgb_camera_calibration.json')
        
        for json_path in [json_path_local, json_path_config]:
            with open(json_path, 'w') as f:
                json.dump(json_data, f, indent=2)
        
        # 4. Save summary as TXT file
        summary = f"""RGB Camera Calibration Summary
=====================================
Timestamp: {timestamp}
Calibration Type: RGB Camera (Raspberry Pi)

Pattern Information:
- Pattern Size: {self.pattern_size[0]} x {self.pattern_size[1]} corners
- Square Size: {self.square_size} mm
- Number of Images Used: {len(self.objpoints)}

Image Information:
- Resolution: {img_shape[0]} x {img_shape[1]} pixels

Calibration Results:
- Mean Reprojection Error: {mean_error:.4f} pixels

Camera Matrix:
{mtx}

Distortion Coefficients:
{dist.flatten()}

Quality Assessment:
- Reprojection Error < 0.5 pixels: {'✅ Excellent' if mean_error < 0.5 else '❌ Poor'}
- Number of Images >= 15: {'✅ Good' if len(self.objpoints) >= 15 else '⚠️ Minimal'}

Camera Parameters:
- Focal Length X (fx): {mtx[0,0]:.2f} pixels
- Focal Length Y (fy): {mtx[1,1]:.2f} pixels  
- Principal Point X (cx): {mtx[0,2]:.2f} pixels
- Principal Point Y (cy): {mtx[1,2]:.2f} pixels

Files Generated:
- rgb_camera_calibration.pkl (local & config/)
- rgb_camera_calibration.xml (local & config/)
- rgb_camera_calibration.json (local & config/)
- rgb_camera_calibration_summary.txt (local & config/)

Note: All calibration files have been saved both locally and in the ../../config directory.
"""
        
        summary_path_local = 'rgb_camera_calibration_summary.txt'
        summary_path_config = os.path.join(self.config_dir, 'rgb_camera_calibration_summary.txt')
        
        for summary_path in [summary_path_local, summary_path_config]:
            with open(summary_path, 'w') as f:
                f.write(summary)
        
        print(f"\n✅ Calibration results saved in multiple formats:")
        print(f"   - PKL files: local & {self.config_dir}/")
        print(f"   - XML files: local & {self.config_dir}/")  
        print(f"   - JSON files: local & {self.config_dir}/")
        print(f"   - Summary: local & {self.config_dir}/")
        
    def calibrate_camera(self):
        """Perform camera calibration"""
        if len(self.objpoints) < 10:
            print("Not enough calibration images!")
            return None
            
        print("Performing camera calibration...")
        
        # Get image dimensions
        img_shape = (640, 480)
        
        # Calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, img_shape, None, None
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
            
            # Save calibration results in all formats
            self.save_calibration_results(mtx, dist, rvecs, tvecs, mean_error, img_shape)
            
            print("RGB camera calibration completed and saved!")
            print(f"Camera matrix:\n{mtx}")
            print(f"Distortion coefficients:\n{dist}")
            
            return mtx, dist
        else:
            print("Camera calibration failed!")
            return None

def main():
    parser = argparse.ArgumentParser(description='RGB Camera Calibration')
    parser.add_argument('--load-images', action='store_true', 
                       help='Load existing calibration images instead of capturing new ones')
    parser.add_argument('--image-dir', default='calibration_images',
                       help='Directory containing existing calibration images')
    
    args = parser.parse_args()
    
    # Create directory for calibration images
    os.makedirs('calibration_images', exist_ok=True)
    
    calibrator = RGBCameraCalibrator()
    calibrator.prepare_object_points()
    
    if args.load_images:
        # Load existing images
        if calibrator.load_existing_images(args.image_dir):
            calibrator.calibrate_camera()
        else:
            print("Failed to load existing images. Exiting.")
    else:
        # Capture new images
        calibrator.setup_camera()
        calibrator.collect_calibration_images()
        calibrator.calibrate_camera()

if __name__ == "__main__":
    main()