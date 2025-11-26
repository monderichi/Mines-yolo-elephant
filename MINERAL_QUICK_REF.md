# 🔍 Mineral Detection - Quick Reference

## ✅ What's Installed

- ✅ Roboflow library (v1.2.11)
- ✅ Download script created
- ✅ Complete documentation

## 📦 Mineral Classes (15)

| Mineral | Type | Notes |
|---------|------|-------|
| Benitoite | Gemstone | Rare blue |
| Calcite | Carbonate | Common |
| Copper | Native metal | Reddish |
| Cuprite | Oxide | Red/brown |
| Erythrite | Cobalt | Pink/purple |
| Gold | Native metal | Yellow |
| Gypsum | Sulfate | Soft, white |
| Halite | Halide | Rock salt |
| Limonite | Iron oxide | Yellow/brown |
| Magnetite | Iron oxide | Magnetic, black |
| Opal | Silica | Iridescent |
| Prehnite | Silicate | Green |
| Pyrite | Sulfide | "Fool's gold" |
| Silver | Native metal | Metallic |
| Tantalite | Oxide | Dark, heavy |

## 🚀 Quick Start

### 1. Get Roboflow API Key

```bash
# Sign up at https://roboflow.com/
# Get API key from: https://app.roboflow.com/settings/api
# Set it:
export ROBOFLOW_API_KEY="your_key_here"
```

### 2. Download Dataset

```bash
cd /media/monders/Files/robotics/mines/ros2_ws
python3 download_mineral_model.py
```

### 3. Train Model

```bash
# Fast training (nano model)
yolo train data=mineral_dataset/data.yaml model=yolo11n.pt epochs=100

# Better accuracy (medium model)
yolo train data=mineral_dataset/data.yaml model=yolo11m.pt epochs=150
```

### 4. Use with RealSense

```bash
source install/setup.bash
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=runs/detect/train/weights/best.pt \
  confidence:=0.6
```

## 📋 Training Options

| Parameter | Options | Description |
|-----------|---------|-------------|
| `model` | yolo11n/s/m/l/x.pt | Starting model |
| `epochs` | 50-300 | Training iterations |
| `imgsz` | 480/640/1024 | Image size |
| `batch` | 4/8/16/32 | Batch size |
| `patience` | 20-100 | Early stopping |

## ⏱️ Expected Training Time

| Model | GPU | CPU |
|-------|-----|-----|
| yolo11n | 1-2h | 4-8h |
| yolo11s | 2-3h | 8-12h |
| yolo11m | 3-6h | 12-24h |
| yolo11l | 6-12h | 24-48h |

## 🎯 Usage Examples

### Test on Image
```bash
yolo predict model=runs/detect/train/weights/best.pt source=mineral.jpg
```

### Test on Webcam
```bash
yolo predict model=runs/detect/train/weights/best.pt source=0
```

### Test on Video
```bash
yolo predict model=runs/detect/train/weights/best.pt source=video.mp4
```

### Use with ROS2
```bash
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=runs/detect/train/weights/best.pt
```

## 📁 Important Files

| File | Location | Purpose |
|------|----------|---------|
| Download script | `ros2_ws/download_mineral_model.py` | Get dataset |
| Full guide | `ros2_ws/MINERAL_DETECTION_GUIDE.md` | Complete docs |
| Dataset | `mineral_dataset/` | After download |
| Trained model | `runs/detect/train/weights/best.pt` | After training |

## 🔧 Common Commands

```bash
# Check training progress
tensorboard --logdir runs/detect/train

# Validate model
yolo val model=runs/detect/train/weights/best.pt data=mineral_dataset/data.yaml

# Export for deployment
yolo export model=runs/detect/train/weights/best.pt format=onnx

# Benchmark speed
yolo benchmark model=runs/detect/train/weights/best.pt
```

## 💡 Tips

1. **Start with nano model** - Train fastest, test workflow
2. **Use GPU if available** - 5-10x faster training
3. **Monitor training** - Use TensorBoard for live metrics
4. **Save checkpoints** - Training saves automatically every epoch
5. **Test before deploy** - Validate on test images first

## 🐛 Troubleshooting

### No API Key
```bash
echo $ROBOFLOW_API_KEY  # Should show your key
export ROBOFLOW_API_KEY="your_key"  # If empty
```

### Out of Memory
```bash
# Reduce batch size
yolo train data=... model=... batch=4

# Or use smaller model
yolo train data=... model=yolo11n.pt
```

### Slow Training
```bash
# Check if using GPU
python3 -c "import torch; print(torch.cuda.is_available())"

# Use smaller image size
yolo train data=... imgsz=480
```

## 📚 Resources

- **Dataset**: https://universe.roboflow.com/mineraldetectionyolo/mineral-c42yg
- **Roboflow Docs**: https://docs.roboflow.com/
- **YOLOv11 Docs**: https://docs.ultralytics.com/models/yolo11/
- **Full Guide**: `MINERAL_DETECTION_GUIDE.md`

---

**Ready to detect minerals!** 💎🔍

Start with: `python3 download_mineral_model.py`
