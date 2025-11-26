# OBB Training Guide - Custom Mineral Detection

This guide covers how to train a custom YOLOv11 OBB (Oriented Bounding Box) model for mineral detection with rotational information for robotic pick and place.

## Why OBB for Robotics?

**Regular Bounding Boxes:**
- Only provide (x, y, width, height)
- No rotation information
- Suitable for classification, but limited for manipulation

**Oriented Bounding Boxes (OBB):**
- Provide (x, y, width, height, rotation_angle)
- Full 6-DOF pose when combined with depth
- Essential for robotic grasping of non-upright objects
- Better for elongated or rotated minerals

## Dataset Preparation

### Option 1: Roboflow (Recommended)

1. **Create Roboflow Account:**
   - Go to https://roboflow.com
   - Create free account

2. **Create OBB Project:**
   - Click "Create New Project"
   - Select "Object Detection (OBB)" or "Oriented Bounding Box"
   - Name it "Mineral OBB Detection"

3. **Upload Images:**
   - Upload images from `mineral-+++-8/train/images/`
   - Roboflow supports batch upload

4. **Annotate with Rotated Boxes:**
   - Click on image to annotate
   - Select "Oriented Bounding Box" tool
   - Draw rotated boxes around minerals
   - Label each box with mineral class
   - Keyboard shortcuts:
     - Draw box: Click 4 corners
     - Rotate: Drag corner handles
     - Delete: Delete key

5. **Classes to Annotate:**
   Based on your dataset (15 classes):
   - Alunite, Andesite, Azurite, Basalt, Bauxite
   - Chalcopyrite, Diorite, Galena, Gold, Limestone
   - Malachite, Pyrite, Quartz, Sandstone, Sphalerite

6. **Export Dataset:**
   - Click "Generate" → "Export"
   - Select "YOLOv8 OBB" format
   - Download ZIP file
   - Extract to `~/ros2_ws/datasets/mineral-obb/`

### Option 2: CVAT (Computer Vision Annotation Tool)

1. **Install CVAT:**
```bash
# Docker installation (easiest)
git clone https://github.com/opencv/cvat
cd cvat
docker-compose up -d
```

2. **Access CVAT:**
   - Open browser: http://localhost:8080
   - Create account

3. **Create Task:**
   - Upload images
   - Select "Oriented Bounding Boxes" mode
   - Annotate with rotated rectangles

4. **Export:**
   - Export as "YOLO OBB 1.0" format

### Dataset Structure

After export, your dataset should look like:
```
datasets/mineral-obb/
├── data.yaml
├── train/
│   ├── images/
│   │   ├── img1.jpg
│   │   └── img2.jpg
│   └── labels/
│       ├── img1.txt
│       └── img2.txt
└── valid/
    ├── images/
    └── labels/
```

**Label Format (OBB):**
Each line in .txt file:
```
class_id x_center y_center width height rotation
```
- All values normalized to [0, 1]
- rotation in radians

**data.yaml:**
```yaml
path: /path/to/datasets/mineral-obb
train: train/images
val: valid/images

nc: 15  # number of classes
names:
  0: Alunite
  1: Andesite
  2: Azurite
  3: Basalt
  4: Bauxite
  5: Chalcopyrite
  6: Diorite
  7: Galena
  8: Gold
  9: Limestone
  10: Malachite
  11: Pyrite
  12: Quartz
  13: Sandstone
  14: Sphalerite
```

## Training

### Using the Training Script

1. **Update Dataset Path:**
Edit `train_mineral_obb.py`:
```python
DATASET_PATH = '/home/monders/ros2_ws/datasets/mineral-obb'
```

2. **Run Training:**
```bash
cd ~/ros2_ws
python3 train_mineral_obb.py
```

3. **Training Parameters:**
- Model: YOLOv11n-obb (nano, fastest)
- Epochs: 100
- Image size: 640x640
- Batch size: 16
- Device: CUDA if available, else CPU

4. **Monitor Training:**
Training creates `runs/obb/mineral-obb-detector/`:
- `weights/best.pt` - Best model
- `weights/last.pt` - Last epoch
- `results.png` - Training curves
- `confusion_matrix.png` - Validation metrics

### Manual Training (Advanced)

```bash
# YOLOv11n (nano - fastest, 3M params)
yolo obb train data=datasets/mineral-obb/data.yaml model=yolo11n-obb.pt epochs=100 imgsz=640

# YOLOv11s (small - balanced, 9M params)
yolo obb train data=datasets/mineral-obb/data.yaml model=yolo11s-obb.pt epochs=100 imgsz=640

# YOLOv11m (medium - accurate, 20M params)
yolo obb train data=datasets/mineral-obb/data.yaml model=yolo11m-obb.pt epochs=100 imgsz=640
```

## Testing the Trained Model

1. **Test with RealSense:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=runs/obb/mineral-obb-detector/weights/best.pt \
  use_obb:=True \
  confidence:=0.5
```

2. **View Results:**
```bash
# Terminal 1: Launch detection
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=runs/obb/mineral-obb-detector/weights/best.pt \
  use_obb:=True

# Terminal 2: View web interface
python3 web_viewer.py

# Terminal 3: Monitor detections
ros2 topic echo /yolo/obb_detections
```

3. **Check Detection Quality:**
- Open browser: http://localhost:8080
- Look for your mineral classes
- Verify rotation angles make sense
- Check 3D positions (X, Y, Z coordinates)

## Converting Existing Dataset

If you have regular bounding box annotations (like mineral-+++-8/), you need to re-annotate:

### Quick Conversion Tips:

1. **Import to Roboflow:**
   - Upload images + existing labels
   - Roboflow can import YOLO format

2. **Add Rotation:**
   - Click each box
   - Adjust to oriented box
   - Rotate to match mineral orientation

3. **Batch Operations:**
   - Use "Smart Polygon" for similar rotations
   - Copy annotations between similar images

## Video Tutorials

Search YouTube for these topics:

1. **"Roboflow OBB annotation tutorial"**
   - Learn Roboflow's OBB annotation interface

2. **"YOLOv8 OBB training"** (also applies to YOLOv11)
   - End-to-end training workflow

3. **"CVAT oriented bounding box annotation"**
   - Alternative annotation tool

4. **"YOLO OBB for robotics pick and place"**
   - Real-world applications

5. **"Custom YOLO dataset preparation"**
   - Data quality and augmentation tips

## Alternative: Use Regular Detection

If OBB annotation is too time-consuming, you can use your existing regular detection model:

```bash
# This still provides 3D positions (x,y,z) but no rotation
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=runs/detect/train/weights/best.pt \
  use_obb:=False
```

**Limitations:**
- No rotation angle
- Assumes minerals are upright
- Less accurate grasping for irregular shapes

## Tips for Better Results

### Data Quality:
- At least 100 images per class
- Vary lighting conditions
- Different angles and distances
- Include partial occlusions
- Mix backgrounds

### Annotation Quality:
- Tight boxes around minerals
- Consistent rotation reference (e.g., longest axis = 0°)
- Double-check all labels
- Remove duplicate/bad images

### Training Optimization:
- Start with yolo11n-obb (fast iteration)
- Use data augmentation (rotation, flip, scale)
- Monitor validation loss (early stopping)
- Test on unseen images

### Hardware:
- CPU training: Very slow (~2-10 hours)
- GPU training: Fast (~30-60 min)
- Consider Google Colab free GPU if needed

## Integration with Robotic Arm

Once trained, your OBB model provides:

```python
# From /yolo/obb_detections topic
detection.position.x      # 3D position (meters)
detection.position.y
detection.position.z
detection.orientation.x   # Quaternion orientation
detection.orientation.y
detection.orientation.z
detection.orientation.w
detection.rotation        # 2D rotation angle (radians)
detection.class_name      # Mineral type
detection.confidence      # Detection confidence
```

Use this for:
1. **Move-to-position:** Navigate gripper to (x, y, z)
2. **Align gripper:** Rotate to match orientation
3. **Grasp:** Close gripper at optimal angle
4. **Verification:** Check class_name for sorting

## Troubleshooting

### No detections after training:
- Lower confidence threshold: `confidence:=0.3`
- Check if test images similar to training data
- Verify dataset labels are correct

### Poor accuracy:
- Need more training data
- Increase epochs: `epochs=150`
- Try larger model: yolo11s-obb or yolo11m-obb
- Improve annotation quality

### Rotation angles wrong:
- Check annotation consistency
- Verify rotation reference frame
- May need to adjust in post-processing

### Training crashes:
- Reduce batch size: `batch=8`
- Reduce image size: `imgsz=320`
- Check GPU memory: `nvidia-smi`

## Next Steps

1. **Annotate Dataset:** Start with 50-100 images in Roboflow
2. **Quick Training:** Train yolo11n-obb for 50 epochs (test run)
3. **Evaluate:** Test with RealSense, check accuracy
4. **Iterate:** Add more data, adjust annotations, retrain
5. **Deploy:** Use best.pt in production with robotic arm

Good luck with your OBB training! This enables true 6-DOF robotic pick and place.
