#!/bin/bash
# Download YOLOv11 OBB models for pick and place operations

echo "=========================================="
echo "YOLOv11 OBB Model Downloader"
echo "=========================================="
echo ""
echo "This script downloads pre-trained YOLOv11 OBB models."
echo "OBB (Oriented Bounding Box) models are required for"
echo "robotic pick and place operations."
echo ""

# Create models directory
MODELS_DIR="$HOME/.cache/yolo_models"
mkdir -p "$MODELS_DIR"

echo "Models will be downloaded to: $MODELS_DIR"
echo ""

# Function to download model
download_model() {
    local model_name=$1
    local url=$2
    
    if [ -f "$MODELS_DIR/$model_name" ]; then
        echo "✓ $model_name already exists, skipping..."
    else
        echo "Downloading $model_name..."
        wget -q --show-progress -O "$MODELS_DIR/$model_name" "$url"
        if [ $? -eq 0 ]; then
            echo "✓ $model_name downloaded successfully"
        else
            echo "✗ Failed to download $model_name"
        fi
    fi
    echo ""
}

# YOLOv11 OBB models
echo "Downloading YOLOv11 OBB models..."
echo ""

download_model "yolo11n-obb.pt" "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-obb.pt"
download_model "yolo11s-obb.pt" "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s-obb.pt"
download_model "yolo11m-obb.pt" "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m-obb.pt"

echo "=========================================="
echo "Download Complete!"
echo "=========================================="
echo ""
echo "Models are located in: $MODELS_DIR"
echo ""
echo "To use these models with ROS2:"
echo ""
echo "  ros2 launch yolo_realsense yolo_realsense.launch.py \\"
echo "      model:=$MODELS_DIR/yolo11n-obb.pt \\"
echo "      use_obb:=True \\"
echo "      confidence:=0.6"
echo ""
echo "Next steps:"
echo "  1. Train your own OBB model on mineral dataset (see OBB_GUIDE.md)"
echo "  2. Test detection with: use_obb:=True"
echo "  3. Integrate with your robotic arm"
echo ""
