#!/usr/bin/env python3
"""
Standalone converter to transform Python pickle calibration files to OpenCV XML format
for use with the C++ calibrated camera system.
"""

import pickle
import numpy as np
import cv2
import os
import sys

def convert_calibration_to_xml():
    """Convert Python pickle calibration to OpenCV XML format"""
    
    print("🔄 Converting Python calibration to C++ XML format...")
    
    # Check for required calibration files
    required_files = [
        'rgb_camera_calibration.pkl',
        'infrared_camera_calibration.pkl', 
        'stereo_calibration.pkl'
    ]
    
    missing_files = []
    for file in required_files:
        if not os.path.exists(file):
            missing_files.append(file)
    
    if missing_files:
        print("❌ Missing calibration files:")
        for file in missing_files:
            print(f"   - {file}")
        print("\nPlease run the calibration scripts first:")
        print("1. python3 rgb_camera_calibration.py")
        print("2. python3 infrared_camera_calibration.py")
        print("3. python3 stereo_calibration.py")
        return False
    
    try:
        # Load stereo calibration from pickle
        print("📂 Loading stereo_calibration.pkl...")
        with open('stereo_calibration.pkl', 'rb') as f:
            stereo_data = pickle.load(f)
        
        print("✅ Loaded stereo calibration data")
        print(f"   Baseline: {stereo_data['baseline']:.2f} mm")
        print(f"   Image shape: {stereo_data['image_shape']}")
        
        # Create OpenCV FileStorage for writing
        print("💾 Creating stereo_calibration.xml...")
        fs = cv2.FileStorage('stereo_calibration.xml', cv2.FILE_STORAGE_WRITE)
        
        # Write transformation matrices
        fs.write('rotation_matrix', stereo_data['rotation_matrix'])
        fs.write('translation_vector', stereo_data['translation_vector'])
        fs.write('essential_matrix', stereo_data['essential_matrix'])
        fs.write('fundamental_matrix', stereo_data['fundamental_matrix'])
        
        # Write rectification matrices
        fs.write('rectification_matrix_1', stereo_data['rectification_matrix_1'])
        fs.write('rectification_matrix_2', stereo_data['rectification_matrix_2'])
        fs.write('projection_matrix_1', stereo_data['projection_matrix_1'])
        fs.write('projection_matrix_2', stereo_data['projection_matrix_2'])
        fs.write('disparity_to_depth_mapping_matrix', stereo_data['disparity_to_depth_mapping_matrix'])
        
        # Write rectification maps
        print("📐 Writing rectification maps...")
        fs.write('rectification_map_1_rgb', stereo_data['rectification_map_1_rgb'])
        fs.write('rectification_map_2_rgb', stereo_data['rectification_map_2_rgb'])
        fs.write('rectification_map_1_ir', stereo_data['rectification_map_1_ir'])
        fs.write('rectification_map_2_ir', stereo_data['rectification_map_2_ir'])
        
        # Write individual camera matrices
        fs.write('rgb_camera_matrix', stereo_data['rgb_camera_matrix'])
        fs.write('rgb_distortion', stereo_data['rgb_distortion'])
        fs.write('ir_camera_matrix', stereo_data['ir_camera_matrix'])
        fs.write('ir_distortion', stereo_data['ir_distortion'])
        
        # Write scalar parameters
        fs.write('baseline', float(stereo_data['baseline']))
        fs.write('image_width', int(stereo_data['image_shape'][0]))
        fs.write('image_height', int(stereo_data['image_shape'][1]))
        
        # Release file
        fs.release()
        
        print("✅ Calibration successfully converted to stereo_calibration.xml")
        
        # Verify the conversion by loading it back
        print("🔍 Verifying XML file...")
        test_fs = cv2.FileStorage('stereo_calibration.xml', cv2.FILE_STORAGE_READ)
        
        if test_fs.isOpened():
            # Test reading a few key matrices
            R = test_fs.getNode('rotation_matrix').mat()
            T = test_fs.getNode('translation_vector').mat()
            baseline = test_fs.getNode('baseline').real()
            
            if R is not None and T is not None and baseline > 0:
                print("✅ XML file verification successful")
                print(f"   Baseline in XML: {baseline:.2f} mm")
            else:
                print("⚠️  Warning: Some data may not have been written correctly")
            
            test_fs.release()
        else:
            print("❌ Could not verify XML file")
            return False
        
        # Also create a summary file for reference
        print("📄 Creating calibration_summary.txt...")
        with open('calibration_summary.txt', 'w') as summary:
            summary.write("Stereo Camera Calibration Summary\n")
            summary.write("=" * 40 + "\n\n")
            summary.write(f"Baseline: {stereo_data['baseline']:.2f} mm\n")
            summary.write(f"Image Size: {stereo_data['image_shape'][0]}x{stereo_data['image_shape'][1]}\n\n")
            
            summary.write("RGB Camera Matrix:\n")
            rgb_mtx = stereo_data['rgb_camera_matrix']
            summary.write(f"fx: {rgb_mtx[0,0]:.2f}, fy: {rgb_mtx[1,1]:.2f}\n")
            summary.write(f"cx: {rgb_mtx[0,2]:.2f}, cy: {rgb_mtx[1,2]:.2f}\n\n")
            
            summary.write("IR Camera Matrix:\n")
            ir_mtx = stereo_data['ir_camera_matrix']
            summary.write(f"fx: {ir_mtx[0,0]:.2f}, fy: {ir_mtx[1,1]:.2f}\n")
            summary.write(f"cx: {ir_mtx[0,2]:.2f}, cy: {ir_mtx[1,2]:.2f}\n\n")
            
            summary.write("Translation Vector (mm):\n")
            T_vec = stereo_data['translation_vector'].flatten()
            summary.write(f"X: {T_vec[0]:.2f}, Y: {T_vec[1]:.2f}, Z: {T_vec[2]:.2f}\n\n")
            
            summary.write("Files created:\n")
            summary.write("- stereo_calibration.xml (for C++ system)\n")
            summary.write("- calibration_summary.txt (this file)\n")
        
        return True
        
    except FileNotFoundError as e:
        print(f"❌ File not found: {e}")
        return False
        
    except Exception as e:
        print(f"❌ Error during conversion: {e}")
        import traceback
        traceback.print_exc()
        return False

def check_existing_calibration():
    """Check if calibration files already exist and their status"""
    
    print("🔍 Checking existing calibration files...")
    
    files_status = {
        'stereo_calibration.pkl': os.path.exists('stereo_calibration.pkl'),
        'stereo_calibration.xml': os.path.exists('stereo_calibration.xml'),
        'rgb_camera_calibration.pkl': os.path.exists('rgb_camera_calibration.pkl'),
        'infrared_camera_calibration.pkl': os.path.exists('infrared_camera_calibration.pkl')
    }
    
    print("\nFile Status:")
    for filename, exists in files_status.items():
        status = "✅ EXISTS" if exists else "❌ MISSING"
        print(f"  {filename:<30} {status}")
    
    return files_status

def main():
    print("🔧 Stereo Calibration Converter for C++ System")
    print("=" * 50)
    
    # Check current file status
    file_status = check_existing_calibration()
    
    # Check if we're in the right directory
    if not file_status['stereo_calibration.pkl']:
        print("\n📁 Looking for calibration files in ../calibration/...")
        if os.path.exists('../calibration/stereo_calibration.pkl'):
            print("✅ Found calibration files in parent calibration directory")
            print("🔄 Copying files to current directory...")
            
            import shutil
            files_to_copy = [
                'stereo_calibration.pkl',
                'rgb_camera_calibration.pkl', 
                'infrared_camera_calibration.pkl'
            ]
            
            for file in files_to_copy:
                src = f'../calibration/{file}'
                if os.path.exists(src):
                    shutil.copy2(src, '.')
                    print(f"   Copied {file}")
            
            # Update status
            file_status = check_existing_calibration()
        else:
            print("❌ No calibration files found")
            print("\nPlease run the Python calibration scripts first:")
            print("cd calibration")
            print("python3 rgb_camera_calibration.py")
            print("python3 infrared_camera_calibration.py")
            print("python3 stereo_calibration.py")
            return False
    
    # Check if XML already exists and is newer
    if file_status['stereo_calibration.xml'] and file_status['stereo_calibration.pkl']:
        xml_time = os.path.getmtime('stereo_calibration.xml')
        pkl_time = os.path.getmtime('stereo_calibration.pkl')
        
        if xml_time > pkl_time:
            print("\n✅ XML calibration file is already up to date")
            print("🚀 Ready to run C++ system!")
            return True
        else:
            print("\n🔄 Pickle file is newer, converting...")
    
    # Perform conversion
    if file_status['stereo_calibration.pkl']:
        success = convert_calibration_to_xml()
        
        if success:
            print("\n🎉 Conversion completed successfully!")
            print("\n📋 Next steps:")
            print("1. Build the C++ system: ./build.sh")
            print("2. Run the calibrated capture: cd build && ./calibrated_capture")
            print("\n📄 Files created:")
            print("  - stereo_calibration.xml (OpenCV format)")
            print("  - calibration_summary.txt (human readable)")
            
            return True
        else:
            print("\n❌ Conversion failed")
            return False
    else:
        print("\n❌ No stereo_calibration.pkl found")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)