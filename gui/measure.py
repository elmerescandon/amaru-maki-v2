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
motion_tracker = ArmMotionTracker(calibration_time=CALIBRATION_TIME) 

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
            """Enhanced data handler for multi-IMU joint tracking"""
            # Extract raw quaternion data (12 doubles)
            raw_quat_data = struct.unpack('<12d', data)
            
            # Process through the motion tracker
            joint_measurements = motion_tracker.process_frame(raw_quat_data)
            
            # During calibration, motion_tracker.process_frame returns None
            if joint_measurements is None:
                return
                
            # Display comprehensive joint angle measurements
            print("\n" + "="*60)
            print("JOINT ANGLE MEASUREMENTS")
            print("="*60)
            
            # Wrist angles
            wrist = joint_measurements['wrist']
            print(f"WRIST:")
            print(f"  Flexion/Extension:     {wrist['flexion_extension']:6.1f}° {'(Flex)' if wrist['flexion_extension'] > 0 else '(Ext)'}")
            print(f"  Radial/Ulnar Deviation: {wrist['radial_ulnar_deviation']:6.1f}° {'(Radial)' if wrist['radial_ulnar_deviation'] > 0 else '(Ulnar)'}")
            print(f"  Pronation/Supination:   {wrist['pronation_supination']:6.1f}° {'(Pronate)' if wrist['pronation_supination'] > 0 else '(Supinate)'}")
            
            # Elbow angles  
            elbow = joint_measurements['elbow']
            print(f"\nELBOW:")
            print(f"  Flexion/Extension:     {elbow['flexion_extension']:6.1f}° {'(Flex)' if elbow['flexion_extension'] > 0 else '(Ext)'}")
            
            # Optional: Display raw quaternions for debugging
            # raw_quats = joint_measurements['raw_quaternions']
            # print(f"\nRAW QUATERNIONS:")
            # print(f"  Upper Arm: w:{raw_quats['upper_arm'][0]:5.2f} x:{raw_quats['upper_arm'][1]:5.2f} y:{raw_quats['upper_arm'][2]:5.2f} z:{raw_quats['upper_arm'][3]:5.2f}")
            # print(f"  Forearm:   w:{raw_quats['forearm'][0]:5.2f} x:{raw_quats['forearm'][1]:5.2f} y:{raw_quats['forearm'][2]:5.2f} z:{raw_quats['forearm'][3]:5.2f}")
            # print(f"  Palm:      w:{raw_quats['palm'][0]:5.2f} x:{raw_quats['palm'][1]:5.2f} y:{raw_quats['palm'][2]:5.2f} z:{raw_quats['palm'][3]:5.2f}")
            
            print("="*60)

        await client.start_notify(CHAR_UUID, handle_data)
        await asyncio.sleep(100)

asyncio.run(main())
