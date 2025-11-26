# Mineral Detection with YOLOv11 and RealSense D455

## Overview

This guide shows you how to use the **Roboflow Mineral Detection dataset** with your YOLOv11 + RealSense D455 setup to detect 15 different types of minerals in real-time.

## Detectable Minerals (15 Classes)

1. **Benitoite** - Rare blue gemstone
2. **Calcite** - Common carbonate mineral
3. **Copper** - Native copper
4. **Cuprite** - Copper oxide mineral
5. **Erythrite** - Pink/purple cobalt mineral
6. **Gold** - Native gold
7. **Gypsum** - Soft sulfate mineral
8. **Halite** - Rock salt
9. **Limonite** - Iron ore
10. **Magnetite** - Magnetic iron ore
11. **Opal** - Hydrated silica gemstone
12. **Prehnite** - Green phyllosilicate
13. **Pyrite** - "Fool's gold"
14. **Silver** - Native silver
15. **Tantalite** - Tantalum ore

## Setup Options

You have two options to get the mineral detection model:

### Option 1: Download Pre-trained Model (Recommended)

The easiest way is to download a pre-trained model from Roboflow Universe.

#### Step 1: Get Roboflow API Key

1. Sign up at https://roboflow.com/ (free account)
2. Visit https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg
3. Click your profile → Settings → Get API Key
4. Copy your API key

#### Step 2: Set API Key

```bash
# Add to ~/.bashrc for permanent setup
echo 'export ROBOFLOW_API_KEY="your_api_key_here"' >> ~/.bashrc
source ~/.bashrc

# Or set temporarily for this session
export ROBOFLOW_API_KEY="your_api_key_here"
```

#### Step 3: Download the Dataset/Model

```bash
cd /media/monders/Files/robotics/mines/ros2_ws
python3 download_mineral_model.py
```

This will:
- Download the mineral detection dataset
- Set up the YOLOv11 format data
- Show you the dataset location

### Option 2: Manual Download

1. Visit https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg
2. Click "Use this Dataset"
3. Select "YOLOv11" format
4. Download and extract to `~/mineral_dataset/`

## Training Your Own Model

Once you have the dataset, train a custom mineral detection model:

```bash
# Navigate to workspace
cd /media/monders/Files/robotics/mines/ros2_ws

# Train YOLOv11 nano model (fastest)
yolo train data=mineral_dataset/data.yaml model=yolo11n.pt epochs=100 imgsz=640

# Or train medium model (better accuracy)
yolo train data=mineral_dataset/data.yaml model=yolo11m.pt epochs=100 imgsz=640

# Training parameters explained:
# - data: Path to dataset YAML file
# - model: Base YOLO model to start from
# - epochs: Number of training iterations (100-300 typical)
# - imgsz: Input image size (640 is standard)
```

**Training time:** 
- Nano model: ~1-2 hours on GPU, ~4-8 hours on CPU
- Medium model: ~3-6 hours on GPU, ~12-24 hours on CPU

The trained model will be saved to: `runs/detect/train/weights/best.pt`

## Using the Trained Model with RealSense

### Option A: Update Launch File

Edit the launch file to use your custom model:

```bash
# Edit launch file
nano src/yolo_realsense/launch/yolo_realsense.launch.py
```

Change the default model path:
```python
model_arg = DeclareLaunchArgument(
    'model',
    default_value='/path/to/runs/detect/train/weights/best.pt',  # Your custom model
    description='YOLO model to use'
)
```

Then launch:
```bash
source install/setup.bash
ros2 launch yolo_realsense yolo_realsense.launch.py
```

### Option B: Specify Model at Launch

```bash
source install/setup.bash
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=/path/to/runs/detect/train/weights/best.pt \
  confidence:=0.5 \
  show_preview:=True
```

## Testing Without RealSense

Test your model on sample images first:

```bash
# Test on a single image
yolo predict model=runs/detect/train/weights/best.pt source=path/to/image.jpg

# Test on a folder of images
yolo predict model=runs/detect/train/weights/best.pt source=path/to/images/

# Test on webcam
yolo predict model=runs/detect/train/weights/best.pt source=0
```

## Using Pre-trained Weights (Skip Training)

If you don't want to train, you can export the model directly from Roboflow:

1. Visit https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg/model/1
2. Click "Deploy" → "Export Model"
3. Select "YOLOv11" format
4. Download the weights file
5. Use it directly with your ROS2 setup

## Complete Workflow Example

```bash
# 1. Set up API key
export ROBOFLOW_API_KEY="your_key_here"

# 2. Download dataset
cd /media/monders/Files/robotics/mines/ros2_ws
python3 download_mineral_model.py

# 3. Train model (this takes time!)
yolo train data=mineral_dataset/data.yaml model=yolo11n.pt epochs=100

# 4. Test the model
yolo predict model=runs/detect/train/weights/best.pt source=test_image.jpg

# 5. Use with RealSense
source install/setup.bash
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=runs/detect/train/weights/best.pt \
  confidence:=0.6 \
  show_preview:=True
```

## Tips for Better Performance

### Improving Detection Accuracy

1. **Use more training epochs**: 150-300 for better results
2. **Use a larger model**: yolo11m.pt or yolo11l.pt
3. **Adjust confidence threshold**: Lower for more detections, higher for precision
4. **Data augmentation**: Roboflow already applies augmentations

### Optimizing for Real-time

1. **Use nano model**: yolo11n.pt for fastest inference
2. **Reduce image size**: imgsz=480 instead of 640
3. **Adjust camera FPS**: Lower FPS if needed

### Best Lighting for Minerals

- Use consistent lighting conditions
- Avoid harsh shadows
- Consider using a lightbox for better detection
- The model was trained on various lighting conditions

## Mineral-Specific ROS2 Package (Optional)

If you want a dedicated mineral detection node, create a new package:

```bash
cd src
ros2 pkg create mineral_detector --build-type ament_python \
  --dependencies rclpy sensor_msgs cv_bridge std_msgs

# Copy and modify the yolo_detector.py to use mineral-specific logic
```

## Dataset Information

- **Source**: Roboflow Universe
- **Images**: ~1,300 annotated images
- **License**: CC BY 4.0 (free to use with attribution)
- **Format**: YOLO (ready for YOLOv11)
- **Project**: https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg

## Citation

If you use this dataset in research or publication:

```bibtex
@misc{
mineral-c42yg_dataset,
title = { mineral +++ Dataset },
type = { Open Source Dataset },
author = { mineraldetectionyolo },
howpublished = { \url{ https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg } },
url = { https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg },
journal = { Roboflow Universe },
publisher = { Roboflow },
year = { 2024 },
month = { may },
}
```

## Troubleshooting

### API Key Issues
```bash
# Check if API key is set
echo $ROBOFLOW_API_KEY

# Set it if missing
export ROBOFLOW_API_KEY="your_key"
```

### Training Errors
- Ensure you have enough disk space (5-10GB)
- Check GPU memory if using CUDA
- Reduce batch size if out of memory: `batch=8` or `batch=4`

### Download Fails
- Check internet connection
- Verify API key is valid
- Try manual download from Roboflow Universe

## Next Steps

1. **Download the dataset** using the script or manually
2. **Train your model** on the mineral dataset
3. **Test** on sample images
4. **Integrate** with RealSense camera
5. **Deploy** for real-time mineral detection

## Support

- **Roboflow**: https://docs.roboflow.com/
- **YOLOv11**: https://docs.ultralytics.com/
- **Dataset**: https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg

Happy mineral detecting! 💎🔍
