#!/usr/bin/env python3
"""
Download and prepare the Roboflow Mineral Detection dataset/model
for use with YOLOv11 and RealSense camera
"""

from roboflow import Roboflow
import os
import yaml

# Initialize Roboflow
# You'll need to sign up at https://roboflow.com/ and get an API key
# Set your API key as an environment variable: export ROBOFLOW_API_KEY="your_key_here"

def download_mineral_model():
    """Download the mineral detection dataset from Roboflow"""
    
    print("=" * 60)
    print("Mineral Detection Model Setup")
    print("=" * 60)
    print()
    
    # Check for API key
    api_key = os.environ.get('ROBOFLOW_API_KEY')
    
    if not api_key:
        print("⚠️  ROBOFLOW_API_KEY not found!")
        print()
        print("To download the mineral detection model:")
        print("1. Sign up at https://roboflow.com/")
        print("2. Go to https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg")
        print("3. Click 'Use this Model' or 'Use this Dataset'")
        print("4. Get your API key from your account settings")
        print("5. Set it as an environment variable:")
        print("   export ROBOFLOW_API_KEY='your_key_here'")
        print()
        print("Then run this script again.")
        print()
        
        # Provide alternative manual download instructions
        print("=" * 60)
        print("ALTERNATIVE: Manual Download")
        print("=" * 60)
        print()
        print("You can also download directly from Roboflow Universe:")
        print("1. Visit: https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg")
        print("2. Click 'Use this Dataset'")
        print("3. Select 'YOLOv11' format")
        print("4. Download and extract to: ./mineral_dataset/")
        print()
        
        return None
    
    try:
        print("🔑 API key found, initializing Roboflow...")
        rf = Roboflow(api_key=api_key)
        
        print("📦 Accessing mineral detection project...")
        project = rf.workspace("mineraldetectionyolo").project("mineral-c42yg")
        
        print("📥 Downloading dataset (Version 8, YOLOv11 format)...")
        dataset = project.version(8).download("yolov11")
        
        print()
        print("✅ Dataset downloaded successfully!")
        print(f"📁 Location: {dataset.location}")
        print()
        
        # Display dataset information
        print("=" * 60)
        print("Dataset Information")
        print("=" * 60)
        print()
        print("Classes (15 minerals):")
        classes = [
            "benitoite", "calcite", "copper", "cuprite", "erythrite",
            "gold", "gypsum", "halite", "limonite", "magnetite",
            "opal", "prehnite", "pyrite", "silver", "tantalite"
        ]
        for i, cls in enumerate(classes, 1):
            print(f"  {i:2d}. {cls}")
        print()
        print("📊 Dataset stats:")
        print(f"  - Total images: ~1,300")
        print(f"  - License: CC BY 4.0")
        print(f"  - Format: YOLOv11")
        print()
        
        # Show next steps
        print("=" * 60)
        print("Next Steps")
        print("=" * 60)
        print()
        print("1. Train a custom model:")
        print(f"   yolo train data={dataset.location}/data.yaml model=yolo11n.pt epochs=100")
        print()
        print("2. Or use with ROS2:")
        print("   Update the model path in your launch file to use the trained model")
        print()
        
        return dataset.location
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print()
        print("Please check:")
        print("- Your API key is valid")
        print("- You have internet connection")
        print("- The project is accessible")
        return None


if __name__ == "__main__":
    download_mineral_model()
