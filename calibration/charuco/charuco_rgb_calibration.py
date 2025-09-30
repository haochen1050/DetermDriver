#!/usr/bin/env python3
"""
Simplified RGB Camera Calibration with ChArUco Boards
Manual board parameter input - no config file needed
Modified to save results in ../../config folder with specified naming

Author: Assistant (Simplified Version)
Date: 2025
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import pickle
import os
import json
import xml.etree.ElementTree as ET
from xml.dom import minidom
import time

# Handle different OpenCV versions consistently
try:
    import cv2.aruco as aruco
    
    # Check OpenCV version
    cv_version = cv2.__version__
    cv_major = int(cv_version.split('.')[0])
    cv_minor = int(cv_version.split('.')[1])
    
    # Determine API version more reliably
    if hasattr(cv2.aruco, 'CharucoDetector'):
        NEW_ARUCO_API = "NEWEST"  # OpenCV 4.7+
        print(f"Using NEWEST ArUco API (CharucoDetector) - OpenCV {cv_version}")
    elif hasattr(cv2.aruco, 'ArucoDetector'):
        NEW_ARUCO_API = "NEW"     # OpenCV 4.5-4.11
        print(f"Using NEW ArUco API (ArucoDetector) - OpenCV {cv_version}")
    elif hasattr(cv2.aruco, 'getPredefinedDictionary'):
        NEW_ARUCO_API = "STANDARD"  # OpenCV 4.0-4.4
        print(f"Using STANDARD ArUco API - OpenCV {cv_version}")
    else:
        NEW_ARUCO_API = "OLD"     # OpenCV 3.x
        print(f"Using OLD ArUco API - OpenCV {cv_version}")
        
except ImportError:
    NEW_ARUCO_API = None
    print("❌ ArUco module not available")

class RGBCameraCalibrator:
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
        self.charuco_detector = None  # For newest API
        self.aruco_detector = None    # For new API
        
        # Data storage
        self.all_charuco_corners = []
        self.all_charuco_ids = []
        self.images_collected = 0
        
        # Camera resolution for calibration - optimized for Pi Camera v3
        self.calibration_resolution = (1536, 864)  # High resolution for accuracy
        
        # Create config directory structure
        self.config_dir = os.path.join('..', '..', 'config')
        os.makedirs(self.config_dir, exist_ok=True)
        print(f"📁 Config directory: {os.path.abspath(self.config_dir)}")
        
    def create_board_objects(self):
        """Create ChArUco board and detector objects from configuration"""
        try:
            if NEW_ARUCO_API is None:
                print("❌ Your OpenCV version doesn't support ArUco module")
                print("   Please install: pip install opencv-contrib-python>=4.6.0")
                return False
            
            # Get dictionary constant
            try:
                dict_id = getattr(cv2.aruco, self.dict_name)
            except AttributeError:
                print(f"❌ Dictionary {self.dict_name} not available in this OpenCV version")
                return False
            
            # Create ArUco dictionary - handle all API versions
            if NEW_ARUCO_API in ["NEWEST", "NEW", "STANDARD"]:
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            else:  # OLD
                self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)
            
            # Create ChArUco board - handle all API versions
            if NEW_ARUCO_API in ["NEWEST", "NEW", "STANDARD"]:
                # OpenCV 4.0+ including 4.11
                self.board = cv2.aruco.CharucoBoard(
                    (self.squares_x, self.squares_y), 
                    self.square_length, self.marker_length, 
                    self.aruco_dict
                )
            else:  # OLD
                # OpenCV 3.x
                self.board = cv2.aruco.CharucoBoard_create(
                    self.squares_x, self.squares_y, 
                    self.square_length, self.marker_length, 
                    self.aruco_dict
                )
            
            # Create detector parameters
            if NEW_ARUCO_API in ["NEWEST", "NEW", "STANDARD"]:
                self.detector_params = cv2.aruco.DetectorParameters()
            else:  # OLD
                self.detector_params = cv2.aruco.DetectorParameters_create()
            
            # Optimize detector parameters for better detection
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
            
            # Create specialized detectors based on API version
            if NEW_ARUCO_API == "NEWEST" and hasattr(cv2.aruco, 'CharucoDetector'):
                # OpenCV 4.7+ - Try to use CharucoDetector
                try:
                    charuco_params = cv2.aruco.CharucoParameters()
                    refine_params = cv2.aruco.RefineParameters() if hasattr(cv2.aruco, 'RefineParameters') else None
                    
                    # Try different constructor signatures
                    if refine_params is not None:
                        self.charuco_detector = cv2.aruco.CharucoDetector(
                            self.board, charuco_params, self.detector_params, refine_params
                        )
                    else:
                        self.charuco_detector = cv2.aruco.CharucoDetector(
                            self.board, charuco_params, self.detector_params
                        )
                    print("✅ Using CharucoDetector (OpenCV 4.7+)")
                    
                except Exception as e:
                    print(f"⚠️  CharucoDetector creation failed: {e}")
                    print("   Falling back to standard detection method")
                    self.charuco_detector = None
                    
            elif NEW_ARUCO_API in ["NEW", "STANDARD"]:
                # Create ArucoDetector for OpenCV 4.5+
                try:
                    self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
                    print("✅ Using ArucoDetector (OpenCV 4.5+)")
                except:
                    print("⚠️  ArucoDetector not available, using legacy detection")
                    self.aruco_detector = None
            
            print("✅ ChArUco board and detector objects created")
            print(f"   Using {NEW_ARUCO_API} ArUco API")
            return True
            
        except Exception as e:
            print(f"❌ Error creating board objects: {e}")
            import traceback
            traceback.print_exc()
            return False
        
    def setup_camera(self):
        """Initialize Pi Camera v3 with optimal settings and correct color format"""
        try:
            self.picam2 = Picamera2()
            
            # Configure for high quality calibration with correct color format
            config = self.picam2.create_preview_configuration(
                main={"format": 'RGB888', "size": self.calibration_resolution},
                controls={
                    "ExposureTime": 10000,  # Fixed exposure for stability
                    "AnalogueGain": 1.0,    # Low gain for best image quality
                    "AwbEnable": True,      # Enable auto white balance initially
                    "AeEnable": False,      # Disable auto exposure for consistency
                    "Sharpness": 1.0,       # Slight sharpening for better edge detection
                    "Contrast": 1.0         # Standard contrast
                }
            )
            self.picam2.configure(config)
            self.picam2.start()
            
            # Allow camera to settle and auto white balance to work
            print("🔄 Allowing camera to settle and adjust white balance...")
            time.sleep(3)
            
            # Now fix the white balance after it has settled
            try:
                awb_gains = self.picam2.capture_metadata()["ColourGains"]
                print(f"📊 Auto white balance gains: {awb_gains}")
                
                # Apply fixed white balance based on auto-calculated values
                controls = {
                    "AwbEnable": False,  # Now disable auto white balance
                    "ColourGains": awb_gains  # Use the calculated gains
                }
                self.picam2.set_controls(controls)
            except:
                print("⚠️  Could not set fixed white balance, continuing with auto")
            
            print(f"✅ Pi Camera v3 initialized at {self.calibration_resolution[0]}x{self.calibration_resolution[1]}")
            print(f"   Format: RGB888")
            return True
            
        except Exception as e:
            print(f"❌ Camera setup failed: {e}")
            print("   Check camera connection and permissions")
            import traceback
            traceback.print_exc()
            return False
        
    def detect_charuco_board(self, image):
        """Detect ChArUco board in image with support for all OpenCV versions"""
        try:
            # Convert to grayscale
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
            
            # Use the newest API if available
            if self.charuco_detector is not None:
                # OpenCV 4.7+ CharucoDetector
                try:
                    charuco_corners, charuco_ids, marker_corners, marker_ids = self.charuco_detector.detectBoard(gray)
                    
                    # Check if we have sufficient corners
                    if charuco_corners is not None and len(charuco_corners) >= 6:
                        return True, charuco_corners, charuco_ids, marker_corners, marker_ids
                    else:
                        return False, None, None, marker_corners, marker_ids
                except Exception as e:
                    print(f"CharucoDetector failed: {e}, falling back to standard method")
                    # Fall back to standard method
            
            # Standard detection method (works for all versions)
            marker_corners = None
            marker_ids = None
            
            # Detect ArUco markers first
            if self.aruco_detector is not None:
                # Use ArucoDetector for OpenCV 4.5+
                try:
                    marker_corners, marker_ids, _ = self.aruco_detector.detectMarkers(gray)
                except Exception as e:
                    print(f"ArucoDetector failed: {e}, using legacy method")
                    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                        gray, self.aruco_dict, parameters=self.detector_params
                    )
            else:
                # Legacy detection for older OpenCV versions
                marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.detector_params
                )
            
            # If markers found, interpolate ChArUco corners
            if marker_corners is not None and len(marker_corners) > 0:
                # All versions support interpolateCornersCharuco
                try:
                    result = cv2.aruco.interpolateCornersCharuco(
                        marker_corners, marker_ids, gray, self.board
                    )
                    
                    # Handle different return formats
                    if NEW_ARUCO_API in ["NEWEST", "NEW"] and len(result) == 3:
                        # Some new API versions return (charuco_corners, charuco_ids, marker_corners)
                        charuco_corners, charuco_ids, _ = result
                        ret = len(charuco_corners) if charuco_corners is not None else 0
                    else:
                        # Standard format: (ret, charuco_corners, charuco_ids)
                        ret, charuco_corners, charuco_ids = result
                    
                    # Validate results
                    if isinstance(ret, int):
                        corner_count = ret
                        success = ret >= 6
                    elif isinstance(ret, bool):
                        success = ret and charuco_corners is not None
                        corner_count = len(charuco_corners) if charuco_corners is not None else 0
                    else:
                        success = False
                        corner_count = 0
                    
                    # Need at least 6 corners for reliable calibration
                    if success and corner_count >= 6 and charuco_corners is not None:
                        return True, charuco_corners, charuco_ids, marker_corners, marker_ids
                        
                except Exception as e:
                    print(f"Corner interpolation failed: {e}")
                    pass
            
            return False, None, None, marker_corners, marker_ids
            
        except Exception as e:
            print(f"Warning: Detection error: {e}")
            return False, None, None, None, None
        
    def assess_image_quality(self, charuco_corners, charuco_ids):
        """Assess quality of detected ChArUco pattern"""
        if charuco_corners is None or len(charuco_corners) == 0:
            return "NONE", 0
        
        corner_count = len(charuco_corners)
        
        # Quality based on number of corners and distribution
        if corner_count >= 20:
            quality = "EXCELLENT"
            score = 10
        elif corner_count >= 15:
            quality = "VERY_GOOD"
            score = 8
        elif corner_count >= 10:
            quality = "GOOD"
            score = 6
        elif corner_count >= 6:
            quality = "ACCEPTABLE"
            score = 4
        else:
            quality = "POOR"
            score = 2
        
        return quality, score
        
    def collect_calibration_images(self):
        """Collect calibration images interactively with improved UI"""
        print("\n" + "=" * 70)
        print("RGB Camera v3 ChArUco Calibration")
        print("=" * 70)
        print("Setup Checklist:")
        print("✅ ChArUco board printed and mounted on flat surface")
        print("✅ Good, even lighting conditions")
        print("✅ Pi Camera v3 connected and working")
        print("")
        print("Calibration Instructions:")
        print("- Move board to different positions and angles")
        print("- Include corner and edge positions of camera field of view")
        print("- Vary distance from camera (near and far)")
        print("- Ensure board is always fully visible and in focus")
        print("- Wait for GREEN status before saving")
        print("- Press 's' when ChArUco pattern is detected to save image")
        print("- Collect 15-25 images for best accuracy")
        print("- Press 'q' to finish collection")
        print("- Press 'r' to reset collection")
        print("=" * 70)
        
        quality_scores = []
        
        while True:
            try:
                # Capture frame - Pi Camera v3 outputs RGB888
                frame_rgb = self.picam2.capture_array()
                
                # Convert RGB to BGR for OpenCV processing
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                
                # Detect ChArUco board
                ret, charuco_corners, charuco_ids, marker_corners, marker_ids = self.detect_charuco_board(frame_bgr)
                
                # Assess image quality
                quality, score = self.assess_image_quality(charuco_corners, charuco_ids)
                
                # Create display frame (downscale for display)
                display_scale = 0.5
                display_size = (int(self.calibration_resolution[0] * display_scale), 
                              int(self.calibration_resolution[1] * display_scale))
                display_frame = cv2.resize(frame_bgr, display_size)
                
                if ret and charuco_corners is not None:
                    # Scale corners for display
                    charuco_corners_display = charuco_corners * display_scale
                    
                    # Draw ArUco markers
                    if marker_corners is not None and marker_ids is not None:
                        marker_corners_display = [corner * display_scale for corner in marker_corners]
                        cv2.aruco.drawDetectedMarkers(display_frame, marker_corners_display, marker_ids)
                    
                    # Draw ChArUco corners
                    cv2.aruco.drawDetectedCornersCharuco(display_frame, charuco_corners_display, charuco_ids, (0, 255, 0))
                    
                    # Status text
                    cv2.putText(display_frame, f"ChArUco detected! {len(charuco_corners)} corners", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(display_frame, f"Quality: {quality} (Score: {score}/10)", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(display_frame, f"Press 's' to save ({self.images_collected}/20)", 
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Quality color coding
                    if quality == "EXCELLENT":
                        quality_color = (0, 255, 0)  # Green
                    elif quality in ["VERY_GOOD", "GOOD"]:
                        quality_color = (0, 200, 0)  # Light green
                    elif quality == "ACCEPTABLE":
                        quality_color = (0, 150, 150)  # Yellow-green
                    else:
                        quality_color = (0, 100, 255)  # Orange
                    
                    cv2.putText(display_frame, f"Quality: {quality}", 
                               (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, quality_color, 1)
                    
                else:
                    cv2.putText(display_frame, f"No ChArUco pattern detected ({self.images_collected}/20)", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(display_frame, "Ensure good lighting & clear pattern", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)
                    
                    # Show detected ArUco markers even if ChArUco detection failed
                    if marker_corners is not None and marker_ids is not None and len(marker_corners) > 0:
                        marker_corners_display = [corner * display_scale for corner in marker_corners]
                        cv2.aruco.drawDetectedMarkers(display_frame, marker_corners_display, marker_ids)
                        cv2.putText(display_frame, f"Found {len(marker_ids)} ArUco markers", 
                                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
                
                # Display controls
                cv2.putText(display_frame, "s=save | q=quit | r=reset", 
                           (10, display_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Progress bar
                progress = min(self.images_collected / 20.0, 1.0)
                bar_width = 300
                bar_height = 10
                bar_y = display_frame.shape[0] - 50
                cv2.rectangle(display_frame, (10, bar_y), 
                             (10 + int(bar_width * progress), bar_y + bar_height), (0, 255, 0), -1)
                cv2.rectangle(display_frame, (10, bar_y), 
                             (10 + bar_width, bar_y + bar_height), (255, 255, 255), 1)
                
                # Quality score history
                if quality_scores:
                    avg_quality = sum(quality_scores) / len(quality_scores)
                    cv2.putText(display_frame, f"Avg Quality: {avg_quality:.1f}/10", 
                               (display_frame.shape[1] - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow('RGB Camera v3 ChArUco Calibration', display_frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s') and ret:
                    self.all_charuco_corners.append(charuco_corners)
                    self.all_charuco_ids.append(charuco_ids)
                    self.images_collected += 1
                    quality_scores.append(score)
                    
                    print(f"✓ Saved image {self.images_collected} - {len(charuco_corners)} corners, Quality: {quality}")
                    
                    # Save the high-resolution image for reference
                    os.makedirs('calibration_images', exist_ok=True)
                    cv2.imwrite(f'calibration_images/charuco_rgb_{self.images_collected:02d}.jpg', frame_bgr)
                    
                    # Save metadata
                    metadata = {
                        'image_number': self.images_collected,
                        'corners_detected': len(charuco_corners),
                        'quality': quality,
                        'quality_score': score,
                        'corner_ids': charuco_ids.flatten().tolist() if charuco_ids is not None else []
                    }
                    with open(f'calibration_images/charuco_rgb_{self.images_collected:02d}_metadata.json', 'w') as f:
                        json.dump(metadata, f, indent=2)
                    
                elif key == ord('r'):
                    self.all_charuco_corners = []
                    self.all_charuco_ids = []
                    self.images_collected = 0
                    quality_scores = []
                    print("🔄 Reset calibration data")
                    
                elif key == ord('q'):
                    if self.images_collected >= 10:
                        break
                    else:
                        print(f"❌ Need at least 10 images, you have {self.images_collected}")
                        
            except Exception as e:
                print(f"Error in capture loop: {e}")
                time.sleep(0.1)
        
        self.picam2.stop()
        cv2.destroyAllWindows()
        
        # Print collection summary
        if quality_scores:
            avg_quality = sum(quality_scores) / len(quality_scores)
            print(f"\n📊 Collection Summary:")
            print(f"   Images collected: {self.images_collected}")
            print(f"   Average quality score: {avg_quality:.1f}/10")
            
    def calibrate_camera(self):
        """Perform camera calibration using ChArUco corners with enhanced analysis"""
        if len(self.all_charuco_corners) < 10:
            print("❌ Not enough calibration images!")
            return None
            
        print("🔄 Performing ChArUco camera calibration...")
        
        # Image dimensions
        img_shape = self.calibration_resolution
        
        # Calibrate camera using ChArUco corners with enhanced flags
        try:
            ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                self.all_charuco_corners, self.all_charuco_ids, self.board, 
                img_shape, None, None,
                flags=cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL
            )
        except Exception as e:
            print(f"❌ Calibration failed: {e}")
            try:
                # Fallback to standard calibration
                ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                    self.all_charuco_corners, self.all_charuco_ids, self.board, 
                    img_shape, None, None
                )
                print("✅ Used standard calibration model")
            except Exception as e2:
                print(f"❌ Standard calibration also failed: {e2}")
                return None
        
        if ret:
            # Calculate comprehensive reprojection error analysis
            mean_error = 0
            max_error = 0
            total_points = 0
            per_image_errors = []
            
            for i in range(len(self.all_charuco_corners)):
                try:
                    # Get object points for this image
                    if hasattr(self.board, 'getChessboardCorners'):
                        objPoints = self.board.getChessboardCorners()[self.all_charuco_ids[i].flatten()]
                    else:
                        # For older OpenCV versions
                        objPoints = self.board.chessboardCorners[self.all_charuco_ids[i].flatten()]
                        
                    imgpoints2, _ = cv2.projectPoints(objPoints, rvecs[i], tvecs[i], mtx, dist)
                    
                    # Calculate error
                    imgpoints2 = imgpoints2.reshape(-1, 2)
                    corners = self.all_charuco_corners[i].reshape(-1, 2)
                    error = cv2.norm(corners, imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                    
                    per_image_errors.append(error)
                    mean_error += error * len(imgpoints2)
                    max_error = max(max_error, error)
                    total_points += len(imgpoints2)
                except Exception as e:
                    print(f"Warning: Error calculating reprojection for image {i}: {e}")
                    continue
            
            if total_points > 0:
                mean_error /= total_points
            else:
                mean_error = float('inf')
            
            # Quality assessment with more detailed criteria
            if mean_error < 0.3:
                quality = "EXCELLENT"
            elif mean_error < 0.6:
                quality = "GOOD"
            elif mean_error < 1.0:
                quality = "ACCEPTABLE"
            else:
                quality = "POOR - Consider recalibrating"
            
            print(f"✓ ChArUco calibration complete!")
            print(f"  Mean reprojection error: {mean_error:.3f} pixels")
            print(f"  Max reprojection error: {max_error:.3f} pixels")
            if per_image_errors:
                print(f"  Std dev reprojection error: {np.std(per_image_errors):.3f} pixels")
            print(f"  Quality: {quality}")
            print(f"  Images used: {len(self.all_charuco_corners)}")
            print(f"  Total corners: {total_points}")
            
            # Calculate and display FOV
            fx, fy = mtx[0, 0], mtx[1, 1]
            fov_x = 2 * np.arctan(img_shape[0] / (2 * fx)) * 180 / np.pi
            fov_y = 2 * np.arctan(img_shape[1] / (2 * fy)) * 180 / np.pi
            
            print(f"  Field of View: {fov_x:.1f}° × {fov_y:.1f}°")
            print(f"  Focal length (pixels): fx={fx:.1f}, fy={fy:.1f}")
            print(f"  Principal point: cx={mtx[0,2]:.1f}, cy={mtx[1,2]:.1f}")
            
            # Save comprehensive calibration results
            calibration_data = {
                'camera_matrix': mtx,
                'distortion_coefficients': dist,
                'rotation_vectors': rvecs,
                'translation_vectors': tvecs,
                'reprojection_error': mean_error,
                'max_reprojection_error': max_error,
                'std_reprojection_error': np.std(per_image_errors) if per_image_errors else 0.0,
                'per_image_errors': per_image_errors,
                'image_shape': img_shape,
                'board_config': {
                    'squares_x': self.squares_x,
                    'squares_y': self.squares_y,
                    'square_length': self.square_length,
                    'marker_length': self.marker_length,
                    'dict_name': self.dict_name
                },
                'images_used': len(self.all_charuco_corners),
                'total_corners': total_points,
                'camera_model': 'Raspberry Pi Camera v3',
                'calibration_method': 'ChArUco',
                'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
                'opencv_version': cv2.__version__,
                'opencv_api': NEW_ARUCO_API,
                'field_of_view': {'horizontal': fov_x, 'vertical': fov_y},
                'focal_length_mm': None,  # Would need sensor size to calculate
                'calibration_quality': quality
            }
            
            # Save calibration files with specified naming in config directory
            config_pkl_path = os.path.join(self.config_dir, 'rgb_calibration.pkl')
            config_xml_path = os.path.join(self.config_dir, 'rgb_calibration.xml')
            
            # Save to config directory
            with open(config_pkl_path, 'wb') as f:
                pickle.dump(calibration_data, f)
            
            # Save XML format to config directory
            self.save_calibration_xml(calibration_data, config_xml_path)
            
            # Save human-readable summary to config directory
            config_summary_path = os.path.join(self.config_dir, 'rgb_calibration_summary.txt')
            self.save_calibration_summary(calibration_data, config_summary_path)
            
            # Also save to current directory for backward compatibility
            with open('rgb_camera_calibration_charuco.pkl', 'wb') as f:
                pickle.dump(calibration_data, f)
            
            self.save_calibration_xml(calibration_data, 'rgb_camera_calibration_charuco.xml')
            self.save_calibration_summary(calibration_data, 'rgb_camera_calibration_summary.txt')
            
            print("✅ Saved to config directory:")
            print(f"   {config_pkl_path}")
            print(f"   {config_xml_path}")
            print(f"   {config_summary_path}")
            print("✅ Saved to current directory:")
            print("   rgb_camera_calibration_charuco.pkl")
            print("   rgb_camera_calibration_charuco.xml")
            print("   rgb_camera_calibration_summary.txt")
            print("\nCamera Matrix:")
            print(mtx)
            print("\nDistortion Coefficients:")
            print(dist.flatten())
            return True
            
    
    def save_calibration_xml(self, calib_data, filename):
        """Save calibration data in comprehensive XML format"""
        root = ET.Element("CameraCalibration")
        
        # Metadata
        metadata = ET.SubElement(root, "Metadata")
        ET.SubElement(metadata, "CameraModel").text = calib_data['camera_model']
        ET.SubElement(metadata, "CalibrationMethod").text = calib_data['calibration_method']
        ET.SubElement(metadata, "CalibrationDate").text = calib_data['calibration_date']
        ET.SubElement(metadata, "OpenCVVersion").text = calib_data['opencv_version']
        ET.SubElement(metadata, "OpenCVAPI").text = calib_data['opencv_api']
        ET.SubElement(metadata, "ImageWidth").text = str(calib_data['image_shape'][0])
        ET.SubElement(metadata, "ImageHeight").text = str(calib_data['image_shape'][1])
        ET.SubElement(metadata, "ImagesUsed").text = str(calib_data['images_used'])
        ET.SubElement(metadata, "TotalCorners").text = str(calib_data['total_corners'])
        ET.SubElement(metadata, "ReprojectionError").text = f"{calib_data['reprojection_error']:.6f}"
        ET.SubElement(metadata, "MaxReprojectionError").text = f"{calib_data['max_reprojection_error']:.6f}"
        ET.SubElement(metadata, "StdReprojectionError").text = f"{calib_data['std_reprojection_error']:.6f}"
        ET.SubElement(metadata, "CalibrationQuality").text = calib_data['calibration_quality']
        
        # ChArUco Board Configuration
        board_config = ET.SubElement(root, "CharucoBoardConfig")
        ET.SubElement(board_config, "SquaresX").text = str(calib_data['board_config']['squares_x'])
        ET.SubElement(board_config, "SquaresY").text = str(calib_data['board_config']['squares_y'])
        ET.SubElement(board_config, "SquareLength").text = f"{calib_data['board_config']['square_length']:.6f}"
        ET.SubElement(board_config, "MarkerLength").text = f"{calib_data['board_config']['marker_length']:.6f}"
        ET.SubElement(board_config, "ArucoDictionary").text = calib_data['board_config']['dict_name']
        
        # Intrinsic Parameters
        intrinsics = ET.SubElement(root, "IntrinsicParameters")
        
        # Camera Matrix
        camera_matrix = ET.SubElement(intrinsics, "CameraMatrix")
        camera_matrix.set("type_id", "opencv-matrix")
        camera_matrix.set("rows", "3")
        camera_matrix.set("cols", "3")
        camera_matrix.set("dt", "d")
        
        mtx_flat = calib_data['camera_matrix'].flatten()
        camera_matrix.text = '\n' + ' '.join([f"{val:.6f}" for val in mtx_flat]) + '\n'
        
        # Focal lengths and principal point
        focal_length = ET.SubElement(intrinsics, "FocalLength")
        ET.SubElement(focal_length, "fx").text = f"{calib_data['camera_matrix'][0,0]:.6f}"
        ET.SubElement(focal_length, "fy").text = f"{calib_data['camera_matrix'][1,1]:.6f}"
        
        principal_point = ET.SubElement(intrinsics, "PrincipalPoint")
        ET.SubElement(principal_point, "cx").text = f"{calib_data['camera_matrix'][0,2]:.6f}"
        ET.SubElement(principal_point, "cy").text = f"{calib_data['camera_matrix'][1,2]:.6f}"
        
        # Distortion Coefficients
        distortion = ET.SubElement(intrinsics, "DistortionCoefficients")
        distortion.set("type_id", "opencv-matrix")
        distortion.set("rows", "1")
        distortion.set("cols", str(len(calib_data['distortion_coefficients'].flatten())))
        distortion.set("dt", "d")
        
        dist_flat = calib_data['distortion_coefficients'].flatten()
        distortion.text = '\n' + ' '.join([f"{val:.8f}" for val in dist_flat]) + '\n'
        
        # Field of View
        fov = ET.SubElement(intrinsics, "FieldOfView")
        ET.SubElement(fov, "horizontal").text = f"{calib_data['field_of_view']['horizontal']:.2f}"
        ET.SubElement(fov, "vertical").text = f"{calib_data['field_of_view']['vertical']:.2f}"
        
        # Format and save XML
        rough_string = ET.tostring(root, 'unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        
        with open(filename, 'w') as f:
            f.write(pretty_xml)
    
    def save_calibration_summary(self, calib_data, filename):
        """Save human-readable calibration summary"""
        with open(filename, 'w') as f:
            f.write("RGB Camera Calibration Summary\n")
            f.write("=" * 40 + "\n\n")
            
            f.write(f"Camera Model: {calib_data['camera_model']}\n")
            f.write(f"Calibration Date: {calib_data['calibration_date']}\n")
            f.write(f"OpenCV Version: {calib_data['opencv_version']}\n")
            f.write(f"Method: {calib_data['calibration_method']}\n\n")
            
            f.write("ChArUco Board Configuration:\n")
            f.write(f"  Size: {calib_data['board_config']['squares_x']}x{calib_data['board_config']['squares_y']} squares\n")
            f.write(f"  Square size: {calib_data['board_config']['square_length']*1000:.1f}mm\n")
            f.write(f"  Marker size: {calib_data['board_config']['marker_length']*1000:.1f}mm\n")
            f.write(f"  Dictionary: {calib_data['board_config']['dict_name']}\n\n")
            
            f.write("Calibration Results:\n")
            f.write(f"  Images used: {calib_data['images_used']}\n")
            f.write(f"  Total corners: {calib_data['total_corners']}\n")
            f.write(f"  Mean reprojection error: {calib_data['reprojection_error']:.3f} pixels\n")
            f.write(f"  Max reprojection error: {calib_data['max_reprojection_error']:.3f} pixels\n")
            f.write(f"  Std reprojection error: {calib_data['std_reprojection_error']:.3f} pixels\n")
            f.write(f"  Quality assessment: {calib_data['calibration_quality']}\n\n")
            
            f.write("Camera Parameters:\n")
            f.write(f"  Image resolution: {calib_data['image_shape'][0]}x{calib_data['image_shape'][1]}\n")
            f.write(f"  Focal length: fx={calib_data['camera_matrix'][0,0]:.1f}, fy={calib_data['camera_matrix'][1,1]:.1f} pixels\n")
            f.write(f"  Principal point: cx={calib_data['camera_matrix'][0,2]:.1f}, cy={calib_data['camera_matrix'][1,2]:.1f}\n")
            f.write(f"  Field of view: {calib_data['field_of_view']['horizontal']:.1f}° × {calib_data['field_of_view']['vertical']:.1f}°\n\n")
            
            f.write("Camera Matrix:\n")
            mtx = calib_data['camera_matrix']
            for i in range(3):
                f.write(f"  [{mtx[i,0]:10.3f} {mtx[i,1]:10.3f} {mtx[i,2]:10.3f}]\n")
            
            f.write("\nDistortion Coefficients:\n")
            dist = calib_data['distortion_coefficients'].flatten()
            f.write(f"  [")
            for i, d in enumerate(dist):
                f.write(f"{d:8.5f}")
                if i < len(dist) - 1:
                    f.write(", ")
            f.write("]\n")
    

def get_board_parameters():
    """Get ChArUco board parameters from user input"""
    print("\n📏 ChArUco Board Parameter Input")
    print("=" * 40)
    print("Please measure your ChArUco board and enter the parameters:")
    print("(Make sure to measure accurately with a ruler)")
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
    print("Simplified RGB Camera ChArUco Calibration")
    print("=" * 50)
    print(f"OpenCV version: {cv2.__version__}")
    print(f"ArUco API: {NEW_ARUCO_API}")
    
    # Get board parameters from user
    params = get_board_parameters()
    if params is None:
        return False
    
    squares_x, squares_y, square_length_mm, marker_length_mm, dict_name = params
    
    # Create directory for calibration images
    os.makedirs('calibration_images', exist_ok=True)
    
    calibrator = RGBCameraCalibrator(squares_x, squares_y, square_length_mm, marker_length_mm, dict_name)
    
    # Create board objects
    print("\n🔧 Creating calibration objects...")           
    if not calibrator.create_board_objects():
        print("\n❌ Failed to create calibration objects!")
        return False
    
    print("\n📋 IMPORTANT CHECKLIST:")
    print("☐ ChArUco board printed at actual size (no scaling)")
    print("☐ Board mounted on flat, rigid surface")
    print("☐ Good, even lighting conditions")
    print("☐ Pi Camera v3 connected and working")
    
    ready = input("\nIs your setup ready for calibration? (Y/n): ").lower()
    if ready == 'n':
        print("Please prepare your setup and run this script again.")
        return False
    
    try:
        # Setup camera
        print("\n📷 Initializing camera...")
        if not calibrator.setup_camera():
            return False
        
        # Collect calibration images
        print("\n📸 Collecting calibration images...")
        calibrator.collect_calibration_images()
        
        # Perform calibration
        print("\n🔢 Performing calibration...")
        success = calibrator.calibrate_camera()
        
        if success:
            print("\n" + "="*70)
            print("🎉 RGB Camera ChArUco Calibration Complete!")
            print("="*70)
            print("Files created in ../../config/:")
            print("✅ rgb_calibration.pkl - Binary calibration data")
            print("✅ rgb_calibration.xml - XML format")
            print("✅ rgb_calibration_summary.txt - Human-readable summary")
            print("\nFiles also created in current directory for compatibility:")
            print("✅ rgb_camera_calibration_charuco.pkl")
            print("✅ rgb_camera_calibration_charuco.xml")
            print("✅ calibration_images/ - Reference images with metadata")
            print("\nNext step: Run stereo calibration (if you have depth camera)")
            return True
        else:
            print("\n❌ Calibration failed. Please try again with:")
            print("- Better lighting conditions")
            print("- More diverse board positions")
            print("- Ensure board is flat and undamaged")
            print("- Check that board is properly printed at actual size")
            print("- Verify camera focus and stability")
            return False
            
    except KeyboardInterrupt:
        print("\n🛑 Calibration interrupted by user")
        return False
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        print("Check camera connection and permissions")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)