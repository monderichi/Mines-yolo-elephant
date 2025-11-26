import time
import sys
import glob
import serial.tools.list_ports
from pymycobot.mycobot import MyCobot

def list_serial_ports():
    """ Lists serial port names """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def main():
    print("--- myCobot 320 M5 Connection Test ---")
    
    # 1. Detect Ports
    print("Scanning for serial ports...")
    ports = list(serial.tools.list_ports.comports())
    
    usb_ports = [p.device for p in ports if 'USB' in p.description or 'USB' in p.device or 'ACM' in p.device]
    
    if not usb_ports:
        print("No USB serial ports found. Please check the USB connection.")
        print("Found ports:", [p.device for p in ports])
        # Fallback to common Linux ports
        usb_ports = ['/dev/ttyUSB0', '/dev/ttyACM0']
    else:
        print(f"Found potential ports: {usb_ports}")

    # 2. Try to connect
    connected = False
    mc = None
    
    for port in usb_ports:
        print(f"Attempting to connect to {port} at 115200 baud...")
        try:
            mc = MyCobot(port, 115200)
            time.sleep(0.5)
            # Try to read angles to verify connection
            angles = mc.get_angles()
            if angles and len(angles) == 6:
                print(f"SUCCESS! Connected to {port}")
                print(f"Current Angles: {angles}")
                connected = True
                
                # Flash LED to confirm (if supported)
                print("Flashing LED (Green)...")
                mc.set_color(0, 255, 0)
                time.sleep(1)
                break
            else:
                print(f"Connected to {port} but received invalid data: {angles}")
                mc = None
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")

    if not connected:
        print("\n[ERROR] Could not connect to myCobot 320 M5.")
        print("Troubleshooting:")
        print("1. Ensure the robot is powered on.")
        print("2. Ensure the USB cable is connected.")
        print("3. Check permissions: sudo usermod -a -G dialout $USER")
        print("4. Try a different USB cable or port.")
    else:
        print("\n[OK] Robot is ready for ROS2 integration.")

if __name__ == "__main__":
    main()
