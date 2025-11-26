#!/usr/bin/env python3
"""
Diagnostic script for myCobot 320 M5
Tests basic communication and robot status
"""
import time
import sys

try:
    from pymycobot import MyCobot320
except ImportError:
    from pymycobot.mycobot import MyCobot
    MyCobot320 = MyCobot

PORT = "/dev/ttyACM0"
BAUD = 115200

def main():
    print("=" * 50)
    print("myCobot 320 M5 Diagnostic Tool")
    print("=" * 50)
    
    print(f"\n[1] Connecting to {PORT} at {BAUD} baud...")
    
    try:
        mc = MyCobot320(PORT, BAUD)
        time.sleep(1)  # Give time for connection to establish
        print("    Connection established.")
    except Exception as e:
        print(f"    [ERROR] Failed to connect: {e}")
        print("\n    Troubleshooting:")
        print("    - Is the robot powered on?")
        print("    - Is the USB cable connected?")
        print("    - Run: sudo chmod 666 /dev/ttyACM0")
        return
    
    print("\n[2] Checking robot power status...")
    try:
        power = mc.is_power_on()
        if power == 1:
            print("    Robot power: ON")
        elif power == 0:
            print("    Robot power: OFF")
            print("    Attempting to power on...")
            mc.power_on()
            time.sleep(2)
        else:
            print(f"    Robot power status unknown: {power}")
    except Exception as e:
        print(f"    [WARNING] Could not check power: {e}")
    
    print("\n[3] Reading current joint angles...")
    try:
        angles = mc.get_angles()
        if angles and len(angles) == 6:
            print(f"    Joint angles: {angles}")
        else:
            print(f"    [WARNING] Invalid response: {angles}")
            print("    The robot may need firmware update or be in wrong mode.")
    except Exception as e:
        print(f"    [ERROR] Failed to read angles: {e}")
    
    print("\n[4] Checking servo status...")
    try:
        # Check if servos are enabled
        for i in range(1, 7):
            status = mc.is_servo_enable(i)
            print(f"    Servo {i}: {'Enabled' if status == 1 else 'Disabled' if status == 0 else f'Unknown({status})'}")
    except Exception as e:
        print(f"    [WARNING] Could not check servos: {e}")
    
    print("\n[5] Testing LED (visual confirmation)...")
    try:
        print("    Setting LED to GREEN...")
        mc.set_color(0, 255, 0)
        time.sleep(1)
        print("    Setting LED to BLUE...")
        mc.set_color(0, 0, 255)
        time.sleep(1)
        print("    Setting LED to RED...")
        mc.set_color(255, 0, 0)
        time.sleep(1)
        mc.set_color(0, 0, 0)  # Off
        print("    If you saw the LED change colors, communication is working!")
    except Exception as e:
        print(f"    [WARNING] LED test failed: {e}")
    
    print("\n[6] Attempting small movement test...")
    print("    WARNING: Robot will attempt to move! Ensure area is clear.")
    response = input("    Proceed with movement test? (y/n): ")
    
    if response.lower() == 'y':
        try:
            # Get current position
            current = mc.get_angles()
            print(f"    Current position: {current}")
            
            # Try to move joint 1 by a small amount
            if current and len(current) == 6:
                test_angles = current.copy()
                test_angles[0] = current[0] + 5  # Move joint 1 by 5 degrees
                print(f"    Moving to: {test_angles}")
                mc.send_angles(test_angles, 20)
                time.sleep(3)
                
                new_angles = mc.get_angles()
                print(f"    New position: {new_angles}")
                
                if new_angles and abs(new_angles[0] - test_angles[0]) < 2:
                    print("    [SUCCESS] Robot moved successfully!")
                else:
                    print("    [FAIL] Robot did not move to target position.")
            else:
                print("    Cannot test movement - could not read current position.")
        except Exception as e:
            print(f"    [ERROR] Movement test failed: {e}")
    else:
        print("    Movement test skipped.")
    
    print("\n" + "=" * 50)
    print("Diagnostic complete.")
    print("=" * 50)
    
    print("\nIf the robot is not responding:")
    print("1. Check that 'Transponder' firmware is loaded on the M5Stack")
    print("2. On the M5Stack screen, select 'Transponder' mode")
    print("3. Make sure the robot is not in 'Free Move' mode")
    print("4. Try power cycling the robot")

if __name__ == "__main__":
    main()
