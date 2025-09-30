import cv2
import numpy as np
import pickle
import matplotlib.pyplot as plt

def validate_stereo_calibration():
    """Validate stereo calibration quality"""
    
    # Load calibration data
    with open('stereo_calibration.pkl', 'rb') as f:
        stereo_calib = pickle.load(f)
    
    # Load test images (use some calibration images)
    rgb_image = cv2.imread('calibration_images/stereo_rgb_1.jpg')
    ir_image = cv2.imread('calibration_images/stereo_ir_1.jpg')
    
    if rgb_image is None or ir_image is None:
        print("Test images not found. Please ensure calibration images exist.")
        return
    
    # Apply rectification
    rgb_rect = cv2.remap(rgb_image, 
                        stereo_calib['rectification_map_1_rgb'], 
                        stereo_calib['rectification_map_2_rgb'], 
                        cv2.INTER_LINEAR)
    
    ir_rect = cv2.remap(ir_image, 
                       stereo_calib['rectification_map_1_ir'], 
                       stereo_calib['rectification_map_2_ir'], 
                       cv2.INTER_LINEAR)
    
    # Draw horizontal lines to check rectification
    def draw_horizontal_lines(img, num_lines=10):
        height = img.shape[0]
        for i in range(num_lines):
            y = int(height * i / num_lines)
            cv2.line(img, (0, y), (img.shape[1], y), (0, 255, 0), 1)
        return img
    
    rgb_with_lines = draw_horizontal_lines(rgb_rect.copy())
    ir_with_lines = draw_horizontal_lines(ir_rect.copy())
    
    # Combine images
    combined_original = np.hstack([rgb_image, ir_image])
    combined_rectified = np.hstack([rgb_with_lines, ir_with_lines])
    
    # Display results
    cv2.imshow('Original Images (RGB + IR)', combined_original)
    cv2.imshow('Rectified Images with Epipolar Lines', combined_rectified)
    
    print("Validation Results:")
    print(f"Baseline: {stereo_calib['baseline']:.2f} mm")
    print(f"Rotation matrix determinant: {np.linalg.det(stereo_calib['rotation_matrix']):.6f}")
    print("Epipolar lines should be horizontal and aligned between cameras")
    print("Press any key to close...")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def plot_calibration_errors():
    """Plot calibration errors from individual calibrations"""
    try:
        # Load calibration data
        with open('rgb_camera_calibration.pkl', 'rb') as f:
            rgb_calib = pickle.load(f)
        
        with open('infrared_camera_calibration.pkl', 'rb') as f:
            ir_calib = pickle.load(f)
        
        # Create plot
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # RGB camera error
        ax1.bar(['RGB Camera'], [rgb_calib['reprojection_error']], color='blue', alpha=0.7)
        ax1.set_ylabel('Reprojection Error (pixels)')
        ax1.set_title('RGB Camera Calibration Error')
        ax1.set_ylim(0, max(1.0, rgb_calib['reprojection_error'] * 1.2))
        
        # IR camera error
        ax2.bar(['Infrared Camera'], [ir_calib['reprojection_error']], color='red', alpha=0.7)
        ax2.set_ylabel('Reprojection Error (pixels)')
        ax2.set_title('Infrared Camera Calibration Error')
        ax2.set_ylim(0, max(1.0, ir_calib['reprojection_error'] * 1.2))
        
        plt.tight_layout()
        plt.savefig('calibration_errors.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"RGB camera reprojection error: {rgb_calib['reprojection_error']:.3f} pixels")
        print(f"Infrared camera reprojection error: {ir_calib['reprojection_error']:.3f} pixels")
        print("Good calibration typically has error < 0.5 pixels")
        
        # Compare with RealSense factory calibration
        print("\nComparison with RealSense factory calibration:")
        if 'realsense_ir_intrinsics' in ir_calib:
            factory_fx = ir_calib['realsense_ir_intrinsics']['fx']
            factory_fy = ir_calib['realsense_ir_intrinsics']['fy']
            calibrated_fx = ir_calib['camera_matrix'][0, 0]
            calibrated_fy = ir_calib['camera_matrix'][1, 1]
            
            print(f"fx difference: {abs(calibrated_fx - factory_fx):.2f} pixels")
            print(f"fy difference: {abs(calibrated_fy - factory_fy):.2f} pixels")
        
    except FileNotFoundError:
        print("Calibration files not found!")

def test_depth_accuracy():
    """Test depth accuracy using known distances"""
    print("Depth Accuracy Test")
    print("Place objects at known distances and measure accuracy...")
    
    # Initialize RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    
    print("Instructions:")
    print("- Place object at known distance (measure with ruler)")
    print("- Click on object in depth image")
    print("- Compare measured vs actual distance")
    print("- Press 'q' to quit")
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            depth_value = param[y, x]
            if depth_value > 0:
                distance_mm = depth_value
                distance_cm = distance_mm / 10.0
                print(f"Clicked at ({x}, {y}): {distance_mm}mm ({distance_cm:.1f}cm)")
    
    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
                )
                
                cv2.putText(depth_colormap, "Click on objects to measure distance", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow('Depth Accuracy Test', depth_colormap)
                cv2.setMouseCallback('Depth Accuracy Test', mouse_callback, depth_image)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    print("Calibration Validation")
    print("1. Validating stereo calibration...")
    validate_stereo_calibration()
    
    print("2. Plotting calibration errors...")
    plot_calibration_errors()