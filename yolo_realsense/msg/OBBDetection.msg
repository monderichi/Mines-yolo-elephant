# Single Oriented Bounding Box Detection
# For robotic pick and place operations

std_msgs/Header header

# Detection info
string class_name
float32 confidence

# 2D image coordinates (pixels)
float32 center_x
float32 center_y
float32 width
float32 height
float32 rotation  # rotation angle in radians

# 3D world coordinates (meters) from depth camera
geometry_msgs/Point position  # 3D position in camera frame
geometry_msgs/Quaternion orientation  # Orientation as quaternion

# Depth information
float32 depth  # distance to object in meters
bool has_depth  # whether depth data is available
