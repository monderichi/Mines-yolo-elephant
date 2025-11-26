#!/usr/bin/env python3
"""
Train YOLOv11 OBB Model for Mineral Detection
This script trains an OBB model on your mineral dataset for pick and place operations.
"""

from ultralytics import YOLO
import os

# Configuration
DATASET_PATH = 'mineral-obb/data.yaml'  # Update this to your OBB dataset path
MODEL_SIZE = 'n'  # Options: 'n' (nano), 's' (small), 'm' (medium), 'l' (large), 'x' (xlarge)
EPOCHS = 100
IMAGE_SIZE = 640
BATCH_SIZE = 16
DEVICE = '0'  # Use '0' for GPU 0, 'cpu' for CPU training
PROJECT_NAME = 'mineral-obb-detector'

def main():
    print("=" * 60)
    print("YOLOv11 OBB Training for Mineral Detection")
    print("=" * 60)
    print()
    
    # Check if dataset exists
    if not os.path.exists(DATASET_PATH):
        print(f"❌ Error: Dataset not found at {DATASET_PATH}")
        print()
        print("You need to:")
        print("1. Annotate your images with ORIENTED bounding boxes")
        print("2. Use tools like Roboflow, CVAT, or LabelImg-OBB")
        print("3. Export in YOLOv11 OBB format")
        print("4. Update DATASET_PATH in this script")
        print()
        print("See OBB_GUIDE.md for detailed instructions.")
        return
    
    # Load base OBB model
    model_name = f'yolo11{MODEL_SIZE}-obb.pt'
    print(f"Loading base model: {model_name}")
    model = YOLO(model_name)
    print("✓ Model loaded")
    print()
    
    # Display training configuration
    print("Training Configuration:")
    print(f"  Dataset: {DATASET_PATH}")
    print(f"  Model: {model_name}")
    print(f"  Epochs: {EPOCHS}")
    print(f"  Image Size: {IMAGE_SIZE}")
    print(f"  Batch Size: {BATCH_SIZE}")
    print(f"  Device: {DEVICE}")
    print(f"  Project: {PROJECT_NAME}")
    print()
    
    input("Press Enter to start training or Ctrl+C to cancel...")
    print()
    
    # Train the model
    print("Starting training...")
    print("=" * 60)
    
    results = model.train(
        data=DATASET_PATH,
        epochs=EPOCHS,
        imgsz=IMAGE_SIZE,
        batch=BATCH_SIZE,
        name=PROJECT_NAME,
        device=DEVICE,
        patience=50,  # Early stopping patience
        save=True,
        plots=True,
        val=True,
        # Augmentation settings (adjust as needed)
        hsv_h=0.015,  # HSV-Hue augmentation
        hsv_s=0.7,    # HSV-Saturation augmentation
        hsv_v=0.4,    # HSV-Value augmentation
        degrees=45.0,  # Rotation augmentation (important for OBB!)
        translate=0.1,
        scale=0.5,
        flipud=0.5,
        fliplr=0.5,
        mosaic=1.0,
    )
    
    print()
    print("=" * 60)
    print("Training Complete!")
    print("=" * 60)
    print()
    
    # Validate the model
    print("Running validation...")
    metrics = model.val()
    print()
    
    # Display results location
    best_model = f"runs/obb/{PROJECT_NAME}/weights/best.pt"
    print("Results saved to:")
    print(f"  Best Model: {best_model}")
    print(f"  Training Results: runs/obb/{PROJECT_NAME}/")
    print()
    
    # Test the model
    print("Testing model on validation images...")
    test_results = model.predict(
        source=f"mineral-obb/valid/images",
        conf=0.5,
        save=True,
        project='runs/obb',
        name=f'{PROJECT_NAME}_validation'
    )
    print(f"✓ Validation predictions saved to: runs/obb/{PROJECT_NAME}_validation/")
    print()
    
    # Export instructions
    print("=" * 60)
    print("Next Steps:")
    print("=" * 60)
    print()
    print("1. Review training results:")
    print(f"   - Training plots: runs/obb/{PROJECT_NAME}/results.png")
    print(f"   - Confusion matrix: runs/obb/{PROJECT_NAME}/confusion_matrix.png")
    print()
    print("2. Test with ROS2:")
    print(f"   ros2 launch yolo_realsense yolo_realsense.launch.py \\")
    print(f"       model:={best_model} \\")
    print(f"       use_obb:=True \\")
    print(f"       confidence:=0.6 \\")
    print(f"       show_preview:=True")
    print()
    print("3. View detections:")
    print("   ros2 topic echo /yolo/obb_detections")
    print()
    print("4. Integrate with your robotic arm!")
    print("   See OBB_GUIDE.md for integration examples.")
    print()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTraining cancelled by user.")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nCheck your dataset path and configuration.")
