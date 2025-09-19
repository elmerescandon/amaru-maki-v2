import numpy as np 
import asyncio
from bleak import BleakScanner, BleakClient
import struct
from utils import ArmMotionTracker
import time

# Multi-IMU Motion Tracking Configuration
CHAR_UUID = "abcd1234-5678-90ab-cdef-1234567890ab"  # characteristic UUID
CALIBRATION_TIME = 5  # seconds

# Initialize the multi-IMU motion tracker
motion_tracker = ArmMotionTracker(calibration_time=CALIBRATION_TIME, verbose=False)

# Configure which body parts to track (upper_arm, elbow, wrist)
motion_tracker.set_active_body_parts(upper_arm=True, elbow=True, wrist=True) 

async def main():
    print("Scanning...")
    devices = await BleakScanner.discover(timeout=5.0)
    esp32 = None
    for d in devices:
        if d.name == "ESP32_Hello":  # Match by advertised name
            esp32 = d
            break

    if not esp32:
        print("ESP32 not found!")
        return

    print(f"Found ESP32: {esp32.address}")

    async with BleakClient(esp32) as client:
        print("Connected!")
        
        def handle_data(sender, data):
            """Clean data handler for multi-IMU joint tracking"""
            # Extract raw quaternion data (12 doubles)
            raw_quat_data = struct.unpack('<12d', data)
            
            # Process through the motion tracker (handles all output internally)
            joint_measurements = motion_tracker.process_frame(raw_quat_data)
            
            # During calibration, joint_measurements is None - motion tracker handles output
            # During normal operation, motion tracker displays results based on verbose setting
            pass

        await client.start_notify(CHAR_UUID, handle_data)
        await asyncio.sleep(100)

asyncio.run(main())
