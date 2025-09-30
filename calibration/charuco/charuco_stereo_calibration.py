#!/usr/bin/env python3
"""
Simplified RGB-Depth Stereo Calibration with ChArUco Boards 
Raspberry Pi Camera v3 (RGB) + RealSense D421 (IR1/Depth)
Manual board parameter input - no config file needed
Modified to save results in ../../config folder with specified naming

Author: Assistant (Simplified Version)
Date: 2025
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from picamera2 import Picamera2
import pickle
import os
import threading
import time
import json
import xml.etree.ElementTree as ET
from xml.dom import minidom

# Handle different OpenCV versions consistently
try:
    import cv2.aruco as aruco
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        NEW_ARUCO_API = True
    else:
        NEW_ARUCO_API = False
except ImportError:
    NEW_ARUCO_API = False

class RGBDepthStereoCalibrator:
    """
    Simplified stereo calibration between Raspberry Pi Camera v3 (RGB) and RealSense D421 (IR1/Depth)
    using ChArUco boards with manual parameter input.
    """
    
    def __init__(self, squares_x, squares_y, square_length_mm, marker_length_mm, dict_name='DICT_5X5_250'):
        """
        Initialize calibrator with manual board parameters
        
        Args:
            squares_x: Number of squares in X direction
            squares_y: Number of squares in Y direction  
            square_length_mm: Square size in millimeters
            marker_length_mm: Marker size in millimeters
            dict_name: ArUco dictionary name (default: DICT_5X5_250)
        """
        # Board configuration from user input
        self.squares_x = squares_x
        self.squares_y = squares_y
        self.square_length = square_length_mm / 1000.0  # Convert to meters
        self.marker_length = marker_length_mm / 1000.0  # Convert to meters
        self.dict_name = dict_name
        
        print(f"📏 Board Configuration:")
        print(f"   Grid size: {self.squares_x} x {self.squares_y} squares")
        print(f"   Square size: {square_length_mm}mm")
        print(f"   Marker size: {marker_length_mm}mm") 
        print(f"   Dictionary: {self.dict_name}")
        
        # Calibration objects (created in create_board_objects)
        self.board = None
        self.aruco_dict = None
        self.detector_params = None
        
        # Data storage
        self.all_charuco_corners_rgb = []
        self.all_charuco_ids_rgb = []
        self.all_charuco_corners_ir = []
        self.all_charuco_ids_ir = []
        self.images_collected = 0
        
        # Threading for synchronized capture
        self.rgb_frame = None
        self.ir_frame = None
        self.depth_frame = None
        self.capturing = False
        self.frame_lock = threading.Lock()
        self.last_capture_time = 0
        
        # Camera parameters
        self.rgb_mtx = None
        self.rgb_dist = None
        self.ir_intrinsics = None
        
        # Create config directory structure
        self.config_dir = os.path.join('..', '..', 'config')
        os.makedirs(self.config_dir, exist_ok=True)
        print(f"📁 Config directory: {os.path.abspath(self.config_dir)}")
        
    def create_board_objects(self):
        """Create ChArUco board and detector objects"""
        try:
            # Create ArUco dictionary
            if NEW_ARUCO_API:
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dict_name))
            else:
                self.aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, self.dict_name))
            
            # Create ChArUco board
            if NEW_ARUCO_API:
                self.board = cv2.aruco.CharucoBoard(
                    (self.squares_x, self.squares_y), 
                    self.square_length, self.marker_length, 
                    self.aruco_dict
                )
            else:
                self.board = cv2.aruco.CharucoBoard_create(
                    self.squares_x, self.squares_y, 
                    self.square_length, self.marker_length, 
                    self.aruco_dict
                )
            
            # Create detector parameters optimized for both RGB and IR
            if NEW_ARUCO_API:
                self.detector_params = cv2.aruco.DetectorParameters()
            else:
                self.detector_params = cv2.aruco.DetectorParameters_create()
            
            # Optimize for both RGB and IR detection
            self.detector_params.adaptiveThreshWinSizeMin = 3
            self.detector_params.adaptiveThreshWinSizeMax = 23
            self.detector_params.adaptiveThreshWinSizeStep = 10
            self.detector_params.adaptiveThreshConstant = 7
            self.detector_params.minMarkerPerimeterRate = 0.03
            self.detector_params.maxMarkerPerimeterRate = 4.0
            self.detector_params.polygonalApproxAccuracyRate = 0.05
            self.detector_params.minCornerDistanceRate = 0.05
            self.detector_params.minDistanceToBorder = 3
            self.detector_params.minMarkerDistanceRate = 0.05
            self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector_params.cornerRefinementWinSize = 5
            self.detector_params.cornerRefinementMaxIterations = 30
            self.detector_params.cornerRefinementMinAccuracy = 0.1
            
            print("✅ ChArUco board and detector objects created")
            return True
            
        except Exception as e:
            print(f"❌ Error creating board objects: {e}")
            return False

    def setup_cameras(self):
        """Initialize both cameras with optimal settings"""
        print("🔄 Setting up cameras...")
        
        # Setup Raspberry Pi Camera v3
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": (640, 480)},  # Match IR resolution
                controls={
                    "ExposureTime": 15000,  # Fixed exposure for stability
                    "AnalogueGain": 1.5,    # Slight gain for better pattern detection
                    "AwbEnable": False,     # Disable auto white balance
                    "ColourGains": (1.4, 1.5)  # Manual white balance
                }
            )
            self.picam2.configure(config)
            self.picam2.start()
            print("✅ Raspberry Pi Camera v3 initialized")
        except Exception as e:
            print(f"❌ Failed to initialize RGB camera: {e}")
            raise
        
        # Setup RealSense D421
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams - IR1 for stereo calibration, depth for reference
            self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Start with custom settings for better IR pattern detection
            profile = self.pipeline.start(self.config)
            
            # Get IR camera intrinsics
            ir_stream = profile.get_stream(rs.stream.infrared, 1)
            self.ir_intrinsics = ir_stream.as_video_stream_profile().get_intrinsics()
            
            # Configure RealSense for better ChArUco detection
            device = profile.get_device()
            if device.supports(rs.camera_info.name):
                print(f"RealSense Device: {device.get_info(rs.camera_info.name)}")
                
            # Adjust IR settings for ChArUco detection
            depth_sensor = device.first_depth_sensor()
            if depth_sensor.supports(rs.option.laser_power):
                # Lower laser power for better pattern visibility
                depth_sensor.set_option(rs.option.laser_power, 50)  # Reduced from 100
            if depth_sensor.supports(rs.option.exposure):
                depth_sensor.set_option(rs.option.exposure, 8500)
            if depth_sensor.supports(rs.option.gain):
                depth_sensor.set_option(rs.option.gain, 16)
                
            print("✅ RealSense D421 initialized")
        except Exception as e:
            print(f"❌ Failed to initialize RealSense: {e}")
            raise
            
        print(f"IR1 Camera: {self.ir_intrinsics.width}x{self.ir_intrinsics.height}")
        print(f"IR1 fx={self.ir_intrinsics.fx:.1f}, fy={self.ir_intrinsics.fy:.1f}")
        print(f"IR1 cx={self.ir_intrinsics.ppx:.1f}, cy={self.ir_intrinsics.ppy:.1f}")
        
        # Load RGB calibration
        self.load_rgb_calibration()
        
        # Allow cameras to stabilize
        time.sleep(3)
        
    def load_rgb_calibration(self):
        """Load pre-computed RGB calibration from config directory first, then current directory"""
        try:
            # Try config directory first with new naming
            config_rgb_path = os.path.join(self.config_dir, 'rgb_calibration.pkl')
            if os.path.exists(config_rgb_path):
                with open(config_rgb_path, 'rb') as f:
                    rgb_calib = pickle.load(f)
                    print("✅ RGB calibration loaded from config directory")
            else:
                # Fallback to current directory with original naming
                try:
                    with open('rgb_camera_calibration_charuco.pkl', 'rb') as f:
                        rgb_calib = pickle.load(f)
                        print("✅ RGB ChArUco calibration loaded from current directory")
                except FileNotFoundError:
                    with open('rgb_camera_calibration.pkl', 'rb') as f:
                        rgb_calib = pickle.load(f)
                        print("✅ RGB standard calibration loaded from current directory")
                    
            self.rgb_mtx = rgb_calib['camera_matrix']
            self.rgb_dist = rgb_calib['distortion_coefficients']
            print(f"RGB fx={self.rgb_mtx[0,0]:.1f}, fy={self.rgb_mtx[1,1]:.1f}")
            
            # Validate resolution compatibility
            if 'image_shape' in rgb_calib:
                rgb_shape = rgb_calib['image_shape']
                print(f"RGB calibration resolution: {rgb_shape[0]}x{rgb_shape[1]}")
                if rgb_shape != (640, 480):
                    print("⚠️  Warning: RGB calibration resolution doesn't match current setup")
                    print("   You may need to scale the camera matrix")
                    
        except FileNotFoundError:
            print("❌ RGB calibration not found! Run RGB calibration script first!")
            print("   Looking for: ../../config/rgb_calibration.pkl or rgb_camera_calibration_charuco.pkl")
            raise
    
    def detect_charuco_board(self, image, camera_type="RGB"):
        """Detect ChArUco board in image with improved preprocessing"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # Enhanced preprocessing for IR images
        if camera_type == "IR":
            # Multi-step preprocessing for better IR detection
            # 1. Noise reduction
            gray = cv2.bilateralFilter(gray, 5, 50, 50)
            
            # 2. Contrast enhancement
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            gray = clahe.apply(gray)
            
            # 3. Additional denoising
            gray = cv2.medianBlur(gray, 3)
        
        # Detect ArUco markers
        try:
            if NEW_ARUCO_API:
                detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        except Exception as e:
            print(f"Warning: ArUco detection failed for {camera_type}: {e}")
            return False, None, None, None, None
        
        # If markers found, interpolate ChArUco corners
        if len(corners) > 0:
            try:
                # Handle the interpolation result based on OpenCV version
                result = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
                
                if len(result) == 3:
                    # Format: (ret_count, charuco_corners, charuco_ids)
                    ret, charuco_corners, charuco_ids = result
                    
                    # ret is the number of detected corners (int)
                    if isinstance(ret, int) and ret >= 6 and charuco_corners is not None:
                        return True, charuco_corners, charuco_ids, corners, ids
                else:
                    # Format: (charuco_corners, charuco_ids) - unlikely but handle it
                    charuco_corners, charuco_ids = result
                    if charuco_corners is not None and len(charuco_corners) >= 6:
                        return True, charuco_corners, charuco_ids, corners, ids
                    
            except Exception as e:
                print(f"Warning: ChArUco interpolation failed for {camera_type}: {e}")
                pass
        
        return False, None, None, corners if len(corners) > 0 else None, ids if len(corners) > 0 else None
            
    def capture_frames_threaded(self):
        """Capture frames from both cameras with improved synchronization"""
        def capture_rgb():
            while self.capturing:
                try:
                    frame = self.picam2.capture_array()
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    with self.frame_lock:
                        self.rgb_frame = frame_bgr.copy()
                        self.last_capture_time = time.time()
                except Exception as e:
                    print(f"RGB capture error: {e}")
                time.sleep(0.033)  # ~30fps
                
        def capture_realsense():
            while self.capturing:
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=100)
                    ir_frame = frames.get_infrared_frame(1)  # IR1
                    depth_frame = frames.get_depth_frame()
                    
                    if ir_frame and depth_frame:
                        ir_image = np.asanyarray(ir_frame.get_data())
                        depth_image = np.asanyarray(depth_frame.get_data())
                        
                        with self.frame_lock:
                            self.ir_frame = ir_image.copy()
                            self.depth_frame = depth_image.copy()
                            
                except RuntimeError:
                    continue
                except Exception as e:
                    print(f"RealSense capture error: {e}")
                time.sleep(0.033)  # ~30fps
        
        # Start capture threads
        self.capturing = True
        rgb_thread = threading.Thread(target=capture_rgb, daemon=True)
        rs_thread = threading.Thread(target=capture_realsense, daemon=True)
        
        rgb_thread.start()
        rs_thread.start()
        
        return rgb_thread, rs_thread
        
    def collect_stereo_calibration_data(self):
        """Collect synchronized stereo calibration data with improved UI"""
        print("\n" + "="*70)
        print("RGB ↔ IR1 Stereo Calibration with ChArUco (SIMPLIFIED)")
        print("="*70)
        print("Setup:")
        print("- RGB: Raspberry Pi Camera v3")
        print("- IR1: RealSense D421 infrared camera")
        print("- Pattern: ChArUco board")
        print("- Coordinate System: Depth = IR1")
        print("")
        print("IMPORTANT:")
        print("- Use the SAME ChArUco board from RGB calibration")
        print("- Ensure pattern is visible in BOTH cameras simultaneously")
        print("- Good, even lighting - avoid strong IR interference")
        print("- Move pattern slowly for better detection")
        print("")
        print("Instructions:")
        print("- Move pattern to different positions, angles, and distances")
        print("- Cover entire field of view including corners")
        print("- Wait for GREEN status before saving")
        print("- Press 's' when both patterns are clearly detected")
        print("- Collect 15+ synchronized image pairs for best results")
        print("- Press 'q' when finished, 'r' to reset")
        print("="*70)
        
        # Start capture threads
        rgb_thread, rs_thread = self.capture_frames_threaded()
        
        try:
            while True:
                # Get latest frames
                with self.frame_lock:
                    rgb = self.rgb_frame.copy() if self.rgb_frame is not None else None
                    ir = self.ir_frame.copy() if self.ir_frame is not None else None
                    depth = self.depth_frame.copy() if self.depth_frame is not None else None
                    
                if rgb is None or ir is None:
                    time.sleep(0.1)
                    continue
                    
                # Detect ChArUco patterns
                ret_rgb, charuco_corners_rgb, charuco_ids_rgb, marker_corners_rgb, marker_ids_rgb = \
                    self.detect_charuco_board(rgb, "RGB")
                ret_ir, charuco_corners_ir, charuco_ids_ir, marker_corners_ir, marker_ids_ir = \
                    self.detect_charuco_board(ir, "IR")
                
                # Prepare display
                disp_rgb = rgb.copy()
                disp_ir = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
                
                # Draw detection results for RGB
                if ret_rgb:
                    if marker_corners_rgb is not None and marker_ids_rgb is not None:
                        if NEW_ARUCO_API:
                            cv2.aruco.drawDetectedMarkers(disp_rgb, marker_corners_rgb, marker_ids_rgb)
                        else:
                            cv2.aruco.drawDetectedMarkers(disp_rgb, marker_corners_rgb, marker_ids_rgb)
                    
                    if NEW_ARUCO_API:
                        cv2.aruco.drawDetectedCornersCharuco(disp_rgb, charuco_corners_rgb, charuco_ids_rgb, (0, 255, 0))
                    else:
                        cv2.aruco.drawDetectedCornersCharuco(disp_rgb, charuco_corners_rgb, charuco_ids_rgb, (0, 255, 0))
                    
                    cv2.putText(disp_rgb, f"RGB: ✓ {len(charuco_corners_rgb)} corners", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(disp_rgb, "RGB: ✗ No ChArUco", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    # Show ArUco markers even if ChArUco detection failed
                    if marker_corners_rgb is not None and marker_ids_rgb is not None:
                        if NEW_ARUCO_API:
                            cv2.aruco.drawDetectedMarkers(disp_rgb, marker_corners_rgb, marker_ids_rgb)
                        else:
                            cv2.aruco.drawDetectedMarkers(disp_rgb, marker_corners_rgb, marker_ids_rgb)
                        cv2.putText(disp_rgb, f"Found {len(marker_ids_rgb)} markers", (10, 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
                    
                # Draw detection results for IR
                if ret_ir:
                    if marker_corners_ir is not None and marker_ids_ir is not None:
                        if NEW_ARUCO_API:
                            cv2.aruco.drawDetectedMarkers(disp_ir, marker_corners_ir, marker_ids_ir)
                        else:
                            cv2.aruco.drawDetectedMarkers(disp_ir, marker_corners_ir, marker_ids_ir)
                    
                    if NEW_ARUCO_API:
                        cv2.aruco.drawDetectedCornersCharuco(disp_ir, charuco_corners_ir, charuco_ids_ir, (0, 255, 0))
                    else:
                        cv2.aruco.drawDetectedCornersCharuco(disp_ir, charuco_corners_ir, charuco_ids_ir, (0, 255, 0))
                    
                    cv2.putText(disp_ir, f"IR1: ✓ {len(charuco_corners_ir)} corners", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(disp_ir, "IR1: ✗ No ChArUco", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    # Show ArUco markers even if ChArUco detection failed
                    if marker_corners_ir is not None and marker_ids_ir is not None:
                        if NEW_ARUCO_API:
                            cv2.aruco.drawDetectedMarkers(disp_ir, marker_corners_ir, marker_ids_ir)
                        else:
                            cv2.aruco.drawDetectedMarkers(disp_ir, marker_corners_ir, marker_ids_ir)
                        cv2.putText(disp_ir, f"Found {len(marker_ids_ir)} markers", (10, 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
                
                # Check for corresponding corners
                common_corners = 0
                if ret_rgb and ret_ir:
                    ids_rgb = charuco_ids_rgb.flatten()
                    ids_ir = charuco_ids_ir.flatten()
                    common_ids = np.intersect1d(ids_rgb, ids_ir)
                    common_corners = len(common_ids)
                
                # Combine displays - only RGB and IR
                combined = np.hstack([disp_rgb, disp_ir])
                
                # Status and instructions
                both_detected = ret_rgb and ret_ir and common_corners >= 6
                if both_detected:
                    status = f"🎯 READY! {common_corners} common corners - Press 's' to save pair {self.images_collected + 1}/15"
                    color = (0, 255, 0)
                else:
                    missing = []
                    if not ret_rgb: missing.append("RGB")
                    if not ret_ir: missing.append("IR1")
                    if ret_rgb and ret_ir and common_corners < 6:
                        missing.append(f"Only {common_corners} common corners")
                    status = f"⏳ Waiting: {', '.join(missing) if missing else 'Unknown'} ({self.images_collected}/15)"
                    color = (0, 100, 255)
                    
                cv2.putText(combined, status, (10, combined.shape[0] - 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                cv2.putText(combined, "Controls: s=save | q=quit | r=reset", 
                           (10, combined.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Progress bar
                progress = min(self.images_collected / 15.0, 1.0)
                bar_width = 400
                cv2.rectangle(combined, (10, combined.shape[0] - 90), 
                             (10 + int(bar_width * progress), combined.shape[0] - 80), (0, 255, 0), -1)
                cv2.rectangle(combined, (10, combined.shape[0] - 90), 
                             (10 + bar_width, combined.shape[0] - 80), (255, 255, 255), 1)
                
                cv2.imshow('RGB-IR1 ChArUco Stereo Calibration (SIMPLIFIED)', combined)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s') and both_detected:
                    # Save calibration data
                    self.all_charuco_corners_rgb.append(charuco_corners_rgb)
                    self.all_charuco_ids_rgb.append(charuco_ids_rgb)
                    self.all_charuco_corners_ir.append(charuco_corners_ir)
                    self.all_charuco_ids_ir.append(charuco_ids_ir)
                    self.images_collected += 1
                    
                    # Save reference images
                    os.makedirs('calibration_images', exist_ok=True)
                    cv2.imwrite(f'calibration_images/stereo_charuco_rgb_{self.images_collected:02d}.jpg', rgb)
                    cv2.imwrite(f'calibration_images/stereo_charuco_ir_{self.images_collected:02d}.jpg', ir)
                    if depth is not None:
                        np.save(f'calibration_images/stereo_charuco_depth_{self.images_collected:02d}.npy', depth)
                    
                    print(f"✅ Saved stereo pair {self.images_collected}/15")
                    print(f"   RGB: {len(charuco_corners_rgb)} corners, IR: {len(charuco_corners_ir)} corners, Common: {common_corners}")
                    
                elif key == ord('r'):
                    self.all_charuco_corners_rgb = []
                    self.all_charuco_ids_rgb = []
                    self.all_charuco_corners_ir = []
                    self.all_charuco_ids_ir = []
                    self.images_collected = 0
                    print("🔄 Reset calibration data")
                    
                elif key == ord('q'):
                    if self.images_collected >= 10:
                        break
                    else:
                        print(f"❌ Need at least 10 pairs, you have {self.images_collected}")
                        
        finally:
            # Cleanup
            self.capturing = False
            self.picam2.stop()
            self.pipeline.stop()
            cv2.destroyAllWindows()
                
    def perform_stereo_calibration(self):
        """Perform RGB ↔ IR1 stereo calibration with improved corner matching"""
        if len(self.all_charuco_corners_rgb) < 10:
            print("❌ Insufficient data for stereo calibration!")
            return False
            
        print(f"\n🔄 Performing improved ChArUco stereo calibration with {len(self.all_charuco_corners_rgb)} image pairs...")
        
        # Prepare IR camera matrix from RealSense intrinsics
        ir_mtx = np.array([
            [self.ir_intrinsics.fx, 0, self.ir_intrinsics.ppx],
            [0, self.ir_intrinsics.fy, self.ir_intrinsics.ppy],
            [0, 0, 1]
        ])
        ir_dist = np.array(self.ir_intrinsics.coeffs)
        
        # Image size
        img_shape = (640, 480)
        
        print("Camera parameters:")
        print(f"RGB fx={self.rgb_mtx[0,0]:.1f}, fy={self.rgb_mtx[1,1]:.1f}")
        print(f"IR1 fx={ir_mtx[0,0]:.1f}, fy={ir_mtx[1,1]:.1f}")
        
        # Prepare object points and image points for stereo calibration
        # Improved matching of corresponding corners
        objpoints = []
        imgpoints_rgb = []
        imgpoints_ir = []
        
        valid_pairs = 0
        total_common_corners = 0
        
        for i in range(len(self.all_charuco_corners_rgb)):
            # Get ChArUco IDs for this image pair
            ids_rgb = self.all_charuco_ids_rgb[i].flatten()
            ids_ir = self.all_charuco_ids_ir[i].flatten()
            
            # Find common corner IDs
            common_ids = np.intersect1d(ids_rgb, ids_ir)
            
            if len(common_ids) >= 6:  # Need at least 6 corresponding points
                # Get indices of common IDs in each array
                rgb_indices = np.array([np.where(ids_rgb == id)[0][0] for id in common_ids])
                ir_indices = np.array([np.where(ids_ir == id)[0][0] for id in common_ids])
                
                # Extract corresponding corners
                corners_rgb = self.all_charuco_corners_rgb[i][rgb_indices]
                corners_ir = self.all_charuco_corners_ir[i][ir_indices]
                
                # Get corresponding 3D object points
                if NEW_ARUCO_API:
                    objpts = self.board.getChessboardCorners()[common_ids]
                else:
                    objpts = self.board.chessboardCorners[common_ids]
                
                objpoints.append(objpts)
                imgpoints_rgb.append(corners_rgb)
                imgpoints_ir.append(corners_ir)
                
                valid_pairs += 1
                total_common_corners += len(common_ids)
                
                print(f"   Pair {i+1}: {len(common_ids)} common corners")
        
        if valid_pairs < 10:
            print(f"❌ Not enough valid pairs for stereo calibration! Got {valid_pairs}, need 10+")
            return False
            
        print(f"Using {valid_pairs} image pairs with {total_common_corners} total corresponding corners")
        print(f"Average corners per pair: {total_common_corners/valid_pairs:.1f}")
        
        # Stereo calibration with improved flags
        try:
            retval, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
                objpoints, imgpoints_rgb, imgpoints_ir,
                self.rgb_mtx, self.rgb_dist, ir_mtx, ir_dist,
                img_shape,
                flags=cv2.CALIB_FIX_INTRINSIC,  # Use pre-calibrated intrinsics
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
            )
        except Exception as e:
            print(f"❌ Stereo calibration failed: {e}")
            return False
        
        if retval > 0:
            print(f"✅ ChArUco stereo calibration successful!")
            print(f"   RMS reprojection error: {retval:.3f} pixels")
            
            # Quality assessment
            if retval < 0.4:
                quality = "EXCELLENT"
            elif retval < 0.8:
                quality = "GOOD"
            elif retval < 1.5:
                quality = "ACCEPTABLE"
            else:
                quality = "POOR - Consider recalibrating"
            
            print(f"   Quality: {quality}")
            
            # Calculate baseline (distance between cameras)
            baseline = np.linalg.norm(T)
            print(f"   Baseline distance: {baseline*1000:.1f} mm")
            
            # Compute rectification for epipolar geometry validation
            try:
                R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
                    self.rgb_mtx, self.rgb_dist, ir_mtx, ir_dist,
                    img_shape, R, T,
                    flags=cv2.CALIB_ZERO_DISPARITY,
                    alpha=0
                )
                
                print(f"   Rectification computed successfully")
                print(f"   ROI RGB: {roi1}, ROI IR: {roi2}")
                
                rectification_data = {
                    'R1': R1, 'R2': R2, 'P1': P1, 'P2': P2, 'Q': Q,
                    'roi1': roi1, 'roi2': roi2
                }
            except Exception as e:
                print(f"⚠️  Warning: Rectification computation failed: {e}")
                rectification_data = None
            
            # Save comprehensive calibration results
            stereo_calibration_data = {
                'rgb_to_ir_rotation': R,
                'rgb_to_ir_translation': T,
                'essential_matrix': E,
                'fundamental_matrix': F,
                'rms_error': retval,
                'baseline_mm': baseline * 1000,
                'pairs_used': valid_pairs,
                'total_corners': total_common_corners,
                
                # Camera intrinsics
                'rgb_camera_matrix': self.rgb_mtx,
                'rgb_distortion': self.rgb_dist,
                'ir_camera_matrix': ir_mtx,
                'ir_distortion': ir_dist,
                
                # RealSense intrinsics (raw)
                'ir_intrinsics_raw': {
                    'fx': self.ir_intrinsics.fx,
                    'fy': self.ir_intrinsics.fy,
                    'ppx': self.ir_intrinsics.ppx,
                    'ppy': self.ir_intrinsics.ppy,
                    'coeffs': self.ir_intrinsics.coeffs,
                    'model': str(self.ir_intrinsics.model)
                },
                
                # Rectification data
                'rectification': rectification_data,
                
                # ChArUco board configuration
                'board_config': {
                    'squares_x': self.squares_x,
                    'squares_y': self.squares_y,
                    'square_length': self.square_length,
                    'marker_length': self.marker_length,
                    'dict_name': self.dict_name
                },
                
                # Calibration metadata
                'image_size': img_shape,
                'calibration_method': 'ChArUco_Stereo_Simplified',
                'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
                'opencv_version': cv2.__version__,
                'opencv_api': 'NEW' if NEW_ARUCO_API else 'OLD'
            }
            
            # Save calibration files with specified naming in config directory
            config_pkl_path = os.path.join(self.config_dir, 'rgb_to_depth_calibration.pkl')
            config_xml_path = os.path.join(self.config_dir, 'rgb_to_depth_calibration.xml')
            
            # Save to config directory
            with open(config_pkl_path, 'wb') as f:
                pickle.dump(stereo_calibration_data, f)
            
            # Save XML format to config directory
            self.save_stereo_calibration_xml(stereo_calibration_data, config_xml_path)
            
            # Save human-readable summary to config directory
            config_summary_path = os.path.join(self.config_dir, 'rgb_to_depth_calibration_summary.txt')
            self.save_stereo_calibration_summary(stereo_calibration_data, config_summary_path)
            
            # Also save to current directory for backward compatibility
            with open('rgb_depth_stereo_calibration_charuco.pkl', 'wb') as f:
                pickle.dump(stereo_calibration_data, f)
            
            self.save_stereo_calibration_xml(stereo_calibration_data, 'rgb_depth_stereo_calibration_charuco.xml')
            self.save_stereo_calibration_summary(stereo_calibration_data, 'rgb_depth_stereo_calibration_summary.txt')
            
            print("✅ Saved to config directory:")
            print(f"   {config_pkl_path}")
            print(f"   {config_xml_path}")
            print(f"   {config_summary_path}")
            print("✅ Saved to current directory:")
            print("   rgb_depth_stereo_calibration_charuco.pkl")
            print("   rgb_depth_stereo_calibration_charuco.xml")
            print("   rgb_depth_stereo_calibration_summary.txt")
            print("\nTransformation RGB → IR1 (= Depth coordinate system):")
            print("Rotation matrix:")
            print(R)
            print(f"Translation vector (mm): [{T[0,0]*1000:.2f}, {T[1,0]*1000:.2f}, {T[2,0]*1000:.2f}]")
            
            # Calculate and display transformation summary
            rotation_angles = cv2.Rodrigues(R)[0].flatten() * 180 / np.pi
            print(f"Rotation angles (degrees): [{rotation_angles[0]:.2f}, {rotation_angles[1]:.2f}, {rotation_angles[2]:.2f}]")
            
            return True
        else:
            print("❌ ChArUco stereo calibration failed!")
            return False
    
    def save_stereo_calibration_xml(self, calib_data, filename):
        """Save stereo calibration data in comprehensive XML format"""
        root = ET.Element("StereoCalibration")
        
        # Metadata
        metadata = ET.SubElement(root, "Metadata")
        ET.SubElement(metadata, "CalibrationMethod").text = calib_data['calibration_method']
        ET.SubElement(metadata, "CalibrationDate").text = calib_data['calibration_date']
        ET.SubElement(metadata, "Camera1").text = "Raspberry Pi Camera v3 (RGB)"
        ET.SubElement(metadata, "Camera2").text = "RealSense D421 IR1"
        ET.SubElement(metadata, "ImageWidth").text = str(calib_data['image_size'][0])
        ET.SubElement(metadata, "ImageHeight").text = str(calib_data['image_size'][1])
        ET.SubElement(metadata, "PairsUsed").text = str(calib_data['pairs_used'])
        ET.SubElement(metadata, "TotalCorners").text = str(calib_data['total_corners'])
        ET.SubElement(metadata, "RMSError").text = f"{calib_data['rms_error']:.6f}"
        ET.SubElement(metadata, "BaselineDistance").text = f"{calib_data['baseline_mm']:.3f}"
        ET.SubElement(metadata, "OpenCVVersion").text = calib_data['opencv_version']
        ET.SubElement(metadata, "OpenCVAPI").text = calib_data['opencv_api']
        
        # ChArUco Board Configuration
        board_config = ET.SubElement(root, "CharucoBoardConfig")
        ET.SubElement(board_config, "SquaresX").text = str(calib_data['board_config']['squares_x'])
        ET.SubElement(board_config, "SquaresY").text = str(calib_data['board_config']['squares_y'])
        ET.SubElement(board_config, "SquareLength").text = f"{calib_data['board_config']['square_length']:.6f}"
        ET.SubElement(board_config, "MarkerLength").text = f"{calib_data['board_config']['marker_length']:.6f}"
        ET.SubElement(board_config, "ArucoDictionary").text = calib_data['board_config']['dict_name']
        
        # RGB Camera Parameters
        rgb_camera = ET.SubElement(root, "RGBCamera")
        
        rgb_matrix = ET.SubElement(rgb_camera, "CameraMatrix")
        rgb_matrix.set("type_id", "opencv-matrix")
        rgb_matrix.set("rows", "3")
        rgb_matrix.set("cols", "3")
        rgb_matrix.set("dt", "d")
        mtx_flat = calib_data['rgb_camera_matrix'].flatten()
        rgb_matrix.text = '\n' + ' '.join([f"{val:.6f}" for val in mtx_flat]) + '\n'
        
        rgb_dist = ET.SubElement(rgb_camera, "DistortionCoefficients")
        rgb_dist.set("type_id", "opencv-matrix")
        rgb_dist.set("rows", "1")
        rgb_dist.set("cols", str(len(calib_data['rgb_distortion'].flatten())))
        rgb_dist.set("dt", "d")
        dist_flat = calib_data['rgb_distortion'].flatten()
        rgb_dist.text = '\n' + ' '.join([f"{val:.8f}" for val in dist_flat]) + '\n'
        
        # IR Camera Parameters
        ir_camera = ET.SubElement(root, "IRCamera")
        
        ir_matrix = ET.SubElement(ir_camera, "CameraMatrix")
        ir_matrix.set("type_id", "opencv-matrix")
        ir_matrix.set("rows", "3")
        ir_matrix.set("cols", "3")
        ir_matrix.set("dt", "d")
        ir_mtx_flat = calib_data['ir_camera_matrix'].flatten()
        ir_matrix.text = '\n' + ' '.join([f"{val:.6f}" for val in ir_mtx_flat]) + '\n'
        
        ir_dist = ET.SubElement(ir_camera, "DistortionCoefficients")
        ir_dist.set("type_id", "opencv-matrix")
        ir_dist.set("rows", "1")
        ir_dist.set("cols", str(len(calib_data['ir_distortion'].flatten())))
        ir_dist.set("dt", "d")
        ir_dist_flat = calib_data['ir_distortion'].flatten()
        ir_dist.text = '\n' + ' '.join([f"{val:.8f}" for val in ir_dist_flat]) + '\n'
        
        # Stereo Extrinsics
        extrinsics = ET.SubElement(root, "StereoExtrinsics")
        
        rotation = ET.SubElement(extrinsics, "RotationMatrix")
        rotation.set("type_id", "opencv-matrix")
        rotation.set("rows", "3")
        rotation.set("cols", "3")
        rotation.set("dt", "d")
        R_flat = calib_data['rgb_to_ir_rotation'].flatten()
        rotation.text = '\n' + ' '.join([f"{val:.8f}" for val in R_flat]) + '\n'
        
        translation = ET.SubElement(extrinsics, "TranslationVector")
        translation.set("type_id", "opencv-matrix")
        translation.set("rows", "3")
        translation.set("cols", "1")
        translation.set("dt", "d")
        T_flat = calib_data['rgb_to_ir_translation'].flatten()
        translation.text = '\n' + ' '.join([f"{val:.8f}" for val in T_flat]) + '\n'
        
        essential = ET.SubElement(extrinsics, "EssentialMatrix")
        essential.set("type_id", "opencv-matrix")
        essential.set("rows", "3")
        essential.set("cols", "3")
        essential.set("dt", "d")
        E_flat = calib_data['essential_matrix'].flatten()
        essential.text = '\n' + ' '.join([f"{val:.8f}" for val in E_flat]) + '\n'
        
        fundamental = ET.SubElement(extrinsics, "FundamentalMatrix")
        fundamental.set("type_id", "opencv-matrix")
        fundamental.set("rows", "3")
        fundamental.set("cols", "3")
        fundamental.set("dt", "d")
        F_flat = calib_data['fundamental_matrix'].flatten()
        fundamental.text = '\n' + ' '.join([f"{val:.8f}" for val in F_flat]) + '\n'
        
        # Rectification data if available
        if calib_data['rectification'] is not None:
            rectification = ET.SubElement(root, "Rectification")
            rect_data = calib_data['rectification']
            
            for matrix_name in ['R1', 'R2', 'P1', 'P2', 'Q']:
                if matrix_name in rect_data:
                    matrix_elem = ET.SubElement(rectification, matrix_name)
                    matrix_elem.set("type_id", "opencv-matrix")
                    
                    matrix = rect_data[matrix_name]
                    matrix_elem.set("rows", str(matrix.shape[0]))
                    matrix_elem.set("cols", str(matrix.shape[1]))
                    matrix_elem.set("dt", "d")
                    
                    matrix_flat = matrix.flatten()
                    matrix_elem.text = '\n' + ' '.join([f"{val:.8f}" for val in matrix_flat]) + '\n'
        
        # Format and save XML
        rough_string = ET.tostring(root, 'unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        
        with open(filename, 'w') as f:
            f.write(pretty_xml)

    def save_stereo_calibration_summary(self, calib_data, filename):
        """Save comprehensive human-readable stereo calibration summary"""
        with open(filename, 'w') as f:
            f.write("RGB-to-Depth Stereo Calibration Summary\n")
            f.write("=" * 50 + "\n\n")
            
            # Basic info
            f.write(f"Camera Setup: {calib_data['calibration_method']}\n")
            f.write(f"Calibration Date: {calib_data['calibration_date']}\n")
            f.write(f"OpenCV Version: {calib_data['opencv_version']}\n")
            f.write(f"Method: ChArUco Stereo Calibration\n\n")
            
            # Camera setup
            f.write("Camera Configuration:\n")
            f.write("  RGB Camera: Raspberry Pi Camera v3\n")
            f.write("  Depth Camera: RealSense D421 IR1\n")
            f.write("  Coordinate System: Depth = IR1 (RealSense standard)\n")
            f.write("  Transformation: RGB → Depth coordinate system\n\n")
            
            # ChArUco board configuration
            f.write("ChArUco Board Configuration:\n")
            f.write(f"  Size: {calib_data['board_config']['squares_x']}x{calib_data['board_config']['squares_y']} squares\n")
            f.write(f"  Square size: {calib_data['board_config']['square_length']*1000:.1f}mm\n")
            f.write(f"  Marker size: {calib_data['board_config']['marker_length']*1000:.1f}mm\n")
            f.write(f"  Dictionary: {calib_data['board_config']['dict_name']}\n\n")
            
            # Calibration results
            f.write("Stereo Calibration Results:\n")
            f.write(f"  Image pairs used: {calib_data['pairs_used']}\n")
            f.write(f"  Total corresponding corners: {calib_data['total_corners']}\n")
            f.write(f"  Average corners per pair: {calib_data['total_corners']/calib_data['pairs_used']:.1f}\n")
            f.write(f"  RMS reprojection error: {calib_data['rms_error']:.3f} pixels\n")
            
            # Quality assessment
            if calib_data['rms_error'] < 0.4:
                quality = "EXCELLENT"
            elif calib_data['rms_error'] < 0.8:
                quality = "GOOD"
            elif calib_data['rms_error'] < 1.5:
                quality = "ACCEPTABLE"
            else:
                quality = "POOR - Consider recalibrating"
            f.write(f"  Quality assessment: {quality}\n")
            f.write(f"  Baseline distance: {calib_data['baseline_mm']:.1f}mm\n\n")
            
            # Transformation summary
            f.write("Coordinate Transformation (RGB → Depth):\n")
            R = calib_data['rgb_to_ir_rotation']
            T = calib_data['rgb_to_ir_translation']
            
            # Convert rotation matrix to Euler angles for easier interpretation
            import cv2
            rotation_angles = cv2.Rodrigues(R)[0].flatten() * 180 / np.pi
            
            f.write(f"  Translation (mm): [{T[0,0]*1000:.2f}, {T[1,0]*1000:.2f}, {T[2,0]*1000:.2f}]\n")
            f.write(f"  Rotation angles (degrees): [{rotation_angles[0]:.2f}, {rotation_angles[1]:.2f}, {rotation_angles[2]:.2f}]\n")
            f.write(f"  Physical interpretation:\n")
            f.write(f"    X: {T[0,0]*1000:.1f}mm {'right' if T[0,0] > 0 else 'left'} (RGB relative to depth)\n")
            f.write(f"    Y: {T[1,0]*1000:.1f}mm {'down' if T[1,0] > 0 else 'up'}\n")
            f.write(f"    Z: {T[2,0]*1000:.1f}mm {'forward' if T[2,0] > 0 else 'backward'}\n\n")
            
            # Camera parameters
            f.write("RGB Camera Parameters:\n")
            f.write(f"  Image resolution: {calib_data['image_size'][0]}x{calib_data['image_size'][1]}\n")
            rgb_mtx = calib_data['rgb_camera_matrix']
            f.write(f"  Focal length: fx={rgb_mtx[0,0]:.1f}, fy={rgb_mtx[1,1]:.1f} pixels\n")
            f.write(f"  Principal point: cx={rgb_mtx[0,2]:.1f}, cy={rgb_mtx[1,2]:.1f}\n")
            
            # Calculate FOV for RGB
            rgb_fov_x = 2 * np.arctan(calib_data['image_size'][0] / (2 * rgb_mtx[0,0])) * 180 / np.pi
            rgb_fov_y = 2 * np.arctan(calib_data['image_size'][1] / (2 * rgb_mtx[1,1])) * 180 / np.pi
            f.write(f"  Field of view: {rgb_fov_x:.1f}° × {rgb_fov_y:.1f}°\n\n")
            
            f.write("Depth Camera Parameters (RealSense IR1):\n")
            f.write(f"  Image resolution: {calib_data['image_size'][0]}x{calib_data['image_size'][1]}\n")
            ir_mtx = calib_data['ir_camera_matrix']
            f.write(f"  Focal length: fx={ir_mtx[0,0]:.1f}, fy={ir_mtx[1,1]:.1f} pixels\n")
            f.write(f"  Principal point: cx={ir_mtx[0,2]:.1f}, cy={ir_mtx[1,2]:.1f}\n")
            
            # Calculate FOV for IR
            ir_fov_x = 2 * np.arctan(calib_data['image_size'][0] / (2 * ir_mtx[0,0])) * 180 / np.pi
            ir_fov_y = 2 * np.arctan(calib_data['image_size'][1] / (2 * ir_mtx[1,1])) * 180 / np.pi
            f.write(f"  Field of view: {ir_fov_x:.1f}° × {ir_fov_y:.1f}°\n")
            f.write(f"  Distortion model: {calib_data['ir_intrinsics_raw']['model']}\n\n")
            
            # Matrices for reference
            f.write("RGB Camera Matrix:\n")
            for i in range(3):
                f.write(f"  [{rgb_mtx[i,0]:10.3f} {rgb_mtx[i,1]:10.3f} {rgb_mtx[i,2]:10.3f}]\n")
            
            f.write("\nRGB Distortion Coefficients:\n")
            rgb_dist = calib_data['rgb_distortion'].flatten()
            f.write(f"  [")
            for i, d in enumerate(rgb_dist):
                f.write(f"{d:8.5f}")
                if i < len(rgb_dist) - 1:
                    f.write(", ")
            f.write("]\n\n")
            
            f.write("Depth Camera Matrix:\n")
            for i in range(3):
                f.write(f"  [{ir_mtx[i,0]:10.3f} {ir_mtx[i,1]:10.3f} {ir_mtx[i,2]:10.3f}]\n")
            
            f.write("\nDepth Distortion Coefficients:\n")
            ir_dist = calib_data['ir_distortion'].flatten()
            f.write(f"  [")
            for i, d in enumerate(ir_dist):
                f.write(f"{d:8.5f}")
                if i < len(ir_dist) - 1:
                    f.write(", ")
            f.write("]\n\n")
            
            f.write("Transformation Matrices:\n")
            f.write("Rotation Matrix (RGB → Depth):\n")
            for i in range(3):
                f.write(f"  [{R[i,0]:10.6f} {R[i,1]:10.6f} {R[i,2]:10.6f}]\n")
            
            f.write("\nTranslation Vector (RGB → Depth, meters):\n")
            f.write(f"  [{T[0,0]:10.6f}]\n")
            f.write(f"  [{T[1,0]:10.6f}]\n")
            f.write(f"  [{T[2,0]:10.6f}]\n\n")
            
            # Usage notes
            f.write("Usage Notes for C++ Applications:\n")
            f.write("1. Coordinate transformation: P_depth = R * P_rgb + T\n")
            f.write("2. For point clouds: Use depth coordinate system as reference\n")
            f.write("3. For RGB overlay: Transform RGB pixels to depth coordinates\n")
            f.write("4. Rectification data available for stereo rectification\n")
            f.write("5. All matrices are in OpenCV format (row-major)\n\n")
            
            # File locations
            f.write("Calibration Files:\n")
            f.write("  Binary data: rgb_to_depth_calibration.pkl\n")
            f.write("  XML format: rgb_to_depth_calibration.xml\n")
            f.write("  This summary: rgb_to_depth_calibration_summary.txt\n")
            f.write("  Reference images: calibration_images/stereo_charuco_*\n")

def get_board_parameters():
    """Get ChArUco board parameters from user input"""
    print("\n📏 ChArUco Board Parameter Input")
    print("=" * 40)
    print("Please measure your ChArUco board and enter the parameters:")
    print("(Make sure to use the SAME board from RGB calibration)")
    print("")
    
    while True:
        try:
            squares_x = int(input("Number of squares in X direction (e.g., 12): "))
            squares_y = int(input("Number of squares in Y direction (e.g., 9): "))
            
            if squares_x < 4 or squares_y < 4:
                print("❌ Board must have at least 4x4 squares")
                continue
                
            square_length_mm = float(input("Square size in millimeters (e.g., 30.0): "))
            marker_length_mm = float(input("Marker size in millimeters (e.g., 22.5): "))
            
            if square_length_mm <= 0 or marker_length_mm <= 0:
                print("❌ Sizes must be positive numbers")
                continue
                
            if marker_length_mm >= square_length_mm:
                print("❌ Marker size must be smaller than square size")
                continue
            
            print("\nAvailable ArUco dictionaries:")
            print("1. DICT_4X4_50   - 4x4 markers, 50 unique markers")
            print("2. DICT_5X5_100  - 5x5 markers, 100 unique markers") 
            print("3. DICT_5X5_250  - 5x5 markers, 250 unique markers (recommended)")
            print("4. DICT_6X6_250  - 6x6 markers, 250 unique markers")
            print("5. DICT_7X7_250  - 7x7 markers, 250 unique markers")
            
            dict_choice = input("Choose dictionary (1-5, default=3): ").strip()
            if dict_choice == "" or dict_choice == "3":
                dict_name = "DICT_5X5_250"
            elif dict_choice == "1":
                dict_name = "DICT_4X4_50"
            elif dict_choice == "2":
                dict_name = "DICT_5X5_100"
            elif dict_choice == "4":
                dict_name = "DICT_6X6_250"
            elif dict_choice == "5":
                dict_name = "DICT_7X7_250"
            else:
                print("Invalid choice, using DICT_5X5_250")
                dict_name = "DICT_5X5_250"
            
            # Confirmation
            print(f"\n📋 Configuration Summary:")
            print(f"   Grid: {squares_x} x {squares_y} squares")
            print(f"   Square size: {square_length_mm}mm")
            print(f"   Marker size: {marker_length_mm}mm")
            print(f"   Dictionary: {dict_name}")
            
            confirm = input("\nIs this correct? (Y/n): ").lower()
            if confirm != 'n':
                return squares_x, squares_y, square_length_mm, marker_length_mm, dict_name
            
        except ValueError:
            print("❌ Please enter valid numbers")
        except KeyboardInterrupt:
            print("\n🛑 Cancelled by user")
            return None

def main():
    print("Simplified RGB-Depth ChArUco Stereo Calibration")
    print("Raspberry Pi Camera v3 + RealSense D421")
    print("=" * 60)
    print(f"OpenCV version: {cv2.__version__}")
    print(f"ArUco API: {'NEW' if NEW_ARUCO_API else 'OLD'}")
    
    # Create output directory
    os.makedirs('calibration_images', exist_ok=True)
    
    # Check for RGB calibration first
    config_rgb_path = os.path.join('..', '..', 'config', 'rgb_calibration.pkl')
    if not os.path.exists(config_rgb_path) and not os.path.exists('rgb_camera_calibration_charuco.pkl') and not os.path.exists('rgb_camera_calibration.pkl'):
        print("\n❌ RGB camera calibration file not found!")
        print("   Please run RGB calibration first:")
        print("   python3 rgb_camera_calibration_simplified.py")
        print(f"   Looking for: {config_rgb_path}")
        print("   Or: rgb_camera_calibration_charuco.pkl")
        print("   Or: rgb_camera_calibration.pkl")
        return False
    
    # Get board parameters from user  
    params = get_board_parameters()
    if params is None:
        return False
    
    squares_x, squares_y, square_length_mm, marker_length_mm, dict_name = params
    
    try:
        calibrator = RGBDepthStereoCalibrator(squares_x, squares_y, square_length_mm, marker_length_mm, dict_name)
        
        # Create board objects
        print("\n🔧 Creating calibration objects...")
        if not calibrator.create_board_objects():
            print("\n❌ Failed to create calibration objects!")
            return False
        
        print("\n📋 IMPORTANT CHECKLIST:")
        print("☐ RGB calibration completed (../../config/rgb_calibration.pkl exists)")
        print("☐ Same ChArUco board used for RGB calibration")
        print("☐ Board mounted on flat, rigid surface")
        print("☐ Good, even lighting conditions")
        print("☐ Both cameras connected and working")
        print("☐ Board visible in both RGB and IR cameras")
        
        ready = input("\nIs your setup ready for stereo calibration? (Y/n): ").lower()
        if ready == 'n':
            print("Please prepare your setup and run this script again.")
            return False
        
        # Setup cameras
        print("\n📷 Setting up cameras...")
        calibrator.setup_cameras()
        
        # Collect calibration data
        print("\n📸 Collecting stereo calibration data...")
        calibrator.collect_stereo_calibration_data()
        
        # Perform calibration
        print("\n🔢 Performing stereo calibration...")
        if calibrator.perform_stereo_calibration():
            print("\n" + "="*70)
            print("🎉 SIMPLIFIED CHARUCO STEREO CALIBRATION COMPLETE!")
            print("="*70)
            print("Files created in ../../config/:")
            print("✅ rgb_to_depth_calibration.pkl - Stereo calibration data")
            print("✅ rgb_to_depth_calibration.xml - XML format")
            print("\nFiles also created in current directory for compatibility:")
            print("✅ rgb_depth_stereo_calibration_charuco.pkl")
            print("✅ rgb_depth_stereo_calibration_charuco.xml")
            print("✅ calibration_images/ - Reference images")
            print("")
            print("The calibration provides transformation from RGB to IR1/Depth coordinate system")
            print("Next steps:")
            print("1. Use the calibration data for RGB-depth alignment")
            print("2. Test with real-time depth-RGB fusion applications")
            print("3. Implement point cloud generation with RGB overlay")
            print("4. Validate calibration accuracy with test patterns")
            return True
            
        else:
            print("\n❌ Calibration failed!")
            print("Troubleshooting tips:")
            print("- Ensure ChArUco board is identical to RGB calibration")
            print("- Use good, even lighting for both RGB and IR detection")
            print("- Avoid strong IR light sources (sunlight, halogen lamps)")
            print("- Check that board is flat and high-contrast")
            print("- Try different board positions covering entire field of view")
            print("- Ensure RGB calibration was completed first and files exist")
            print("- Verify camera connections and permissions")
            return False
            
    except KeyboardInterrupt:
        print("\n🛑 Calibration interrupted by user")
        return False
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        print("\nTroubleshooting:")
        print("- RGB calibration file exists (run RGB calibration first)")
        print("- Camera connections and permissions")
        print("- RealSense SDK installed correctly")
        print("- Board parameters match your actual board")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)