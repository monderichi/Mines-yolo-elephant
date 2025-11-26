#!/usr/bin/env python3
"""
Web-based YOLO detection viewer with 3D positions
Access at http://localhost:8080
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import base64
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading


class DetectionViewerNode(Node):
    def __init__(self):
        super().__init__('web_detection_viewer')
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections = []
        self.lock = threading.Lock()
        
        # Subscribe to detection image
        self.image_sub = self.create_subscription(
            Image,
            '/yolo/detection_image',
            self.image_callback,
            10
        )
        
        # Subscribe to detection info
        self.detection_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('Web Detection Viewer started')
        self.get_logger().info('Open browser to: http://localhost:8080')
        
    def detection_callback(self, msg):
        """Store the latest detection data"""
        try:
            with self.lock:
                self.latest_detections = json.loads(msg.data)
        except:
            pass
    
    def image_callback(self, msg):
        """Store the latest image with 3D position overlay"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Overlay 3D positions on the image
            for det in self.latest_detections:
                if 'position_3d' in det:
                    # Get bbox center
                    if 'bbox' in det:
                        bbox = det['bbox']
                        center_x = int((bbox['x1'] + bbox['x2']) / 2)
                        center_y = int((bbox['y1'] + bbox['y2']) / 2)
                    elif 'obb' in det:
                        center_x = int(det['obb']['center_x'])
                        center_y = int(det['obb']['center_y'])
                    else:
                        continue
                    
                    # Get 3D position
                    pos = det['position_3d']
                    x, y, z = pos['x'], pos['y'], pos['z']
                    
                    # Create position text
                    pos_text = f"3D: ({x:.2f}, {y:.2f}, {z:.2f})m"
                    class_text = f"{det['class']} {det['confidence']:.2f}"
                    
                    # Draw text with background
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.7
                    thickness = 2
                    
                    # Position text
                    (w, h), _ = cv2.getTextSize(pos_text, font, font_scale, thickness)
                    cv2.rectangle(cv_image, (center_x - 5, center_y + 5), 
                                (center_x + w + 5, center_y + h + 15), (0, 0, 0), -1)
                    cv2.putText(cv_image, pos_text, (center_x, center_y + h + 10), 
                              font, font_scale, (0, 255, 255), thickness)
                    
                    # Class text above
                    (w2, h2), _ = cv2.getTextSize(class_text, font, font_scale, thickness)
                    cv2.rectangle(cv_image, (center_x - 5, center_y - h2 - 10), 
                                (center_x + w2 + 5, center_y - 5), (0, 0, 0), -1)
                    cv2.putText(cv_image, class_text, (center_x, center_y - 5), 
                              font, font_scale, (0, 255, 0), thickness)
            
            with self.lock:
                self.latest_image = cv_image
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
    
    def get_jpeg_image(self):
        """Get the latest image as JPEG bytes"""
        with self.lock:
            if self.latest_image is not None:
                _, jpeg = cv2.imencode('.jpg', self.latest_image)
                return jpeg.tobytes()
        return None
    
    def get_detections_html(self):
        """Get detection data as HTML table"""
        with self.lock:
            if not self.latest_detections:
                return "<p>No detections</p>"
            
            html = "<table style='width:100%; border-collapse: collapse;'>"
            html += "<tr style='background:#333;'><th>Class</th><th>Confidence</th><th>X (m)</th><th>Y (m)</th><th>Z (m)</th></tr>"
            
            for det in self.latest_detections:
                if 'position_3d' in det:
                    pos = det['position_3d']
                    html += f"<tr style='border-bottom:1px solid #444;'>"
                    html += f"<td>{det['class']}</td>"
                    html += f"<td>{det['confidence']:.2f}</td>"
                    html += f"<td>{pos['x']:.3f}</td>"
                    html += f"<td>{pos['y']:.3f}</td>"
                    html += f"<td>{pos['z']:.3f}</td>"
                    html += "</tr>"
            
            html += "</table>"
            return html


# Global viewer instance
viewer = None


class WebHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress server logs
    
    def do_GET(self):
        global viewer
        
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>YOLO Detection Viewer</title>
                <style>
                    body { 
                        background: #1a1a1a; 
                        color: #fff; 
                        font-family: Arial; 
                        margin: 0;
                        padding: 20px;
                    }
                    h1 { color: #0f0; }
                    #image { 
                        max-width: 100%; 
                        border: 2px solid #0f0;
                        margin: 20px 0;
                    }
                    table { 
                        background: #2a2a2a;
                        color: #fff;
                        margin: 20px 0;
                    }
                    th, td { 
                        padding: 10px; 
                        text-align: left;
                    }
                    th { color: #0f0; }
                    .status { 
                        padding: 10px; 
                        background: #2a2a2a; 
                        border-radius: 5px;
                        margin: 10px 0;
                    }
                </style>
            </head>
            <body>
                <h1>🎯 YOLO Detection Viewer with 3D Positions</h1>
                <div class="status">
                    <strong>Status:</strong> <span id="status">Loading...</span><br>
                    <strong>FPS:</strong> <span id="fps">0</span>
                </div>
                <img id="image" src="/image" alt="Detection Stream">
                <h2>Detection Data</h2>
                <div id="detections">Loading...</div>
                
                <script>
                    let frameCount = 0;
                    let lastTime = Date.now();
                    
                    function updateImage() {
                        const img = document.getElementById('image');
                        img.src = '/image?t=' + new Date().getTime();
                        
                        frameCount++;
                        const now = Date.now();
                        if (now - lastTime >= 1000) {
                            document.getElementById('fps').textContent = frameCount;
                            frameCount = 0;
                            lastTime = now;
                        }
                    }
                    
                    function updateDetections() {
                        fetch('/detections')
                            .then(response => response.text())
                            .then(data => {
                                document.getElementById('detections').innerHTML = data;
                                document.getElementById('status').textContent = 'Active';
                                document.getElementById('status').style.color = '#0f0';
                            })
                            .catch(err => {
                                document.getElementById('status').textContent = 'Error';
                                document.getElementById('status').style.color = '#f00';
                            });
                    }
                    
                    // Update image every 100ms (10 FPS)
                    setInterval(updateImage, 100);
                    
                    // Update detection data every 500ms
                    setInterval(updateDetections, 500);
                    
                    // Initial load
                    updateImage();
                    updateDetections();
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode())
            
        elif self.path.startswith('/image'):
            if viewer:
                jpeg_data = viewer.get_jpeg_image()
                if jpeg_data:
                    self.send_response(200)
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    self.wfile.write(jpeg_data)
                    return
            
            self.send_error(404)
            
        elif self.path == '/detections':
            if viewer:
                html = viewer.get_detections_html()
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(html.encode())
                return
            
            self.send_error(404)
        
        else:
            self.send_error(404)


def run_web_server():
    server = HTTPServer(('0.0.0.0', 8080), WebHandler)
    print('Web server started on http://localhost:8080')
    server.serve_forever()


def main(args=None):
    global viewer
    
    rclpy.init(args=args)
    viewer = DetectionViewerNode()
    
    # Start web server in separate thread
    web_thread = threading.Thread(target=run_web_server, daemon=True)
    web_thread.start()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
