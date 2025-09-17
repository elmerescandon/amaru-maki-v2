import numpy as np 
import asyncio
from bleak import BleakScanner, BleakClient
import struct
from utils import * 
import time

# Quaternion Format (w,x,y,z)

CHAR_UUID = "abcd1234-5678-90ab-cdef-1234567890ab"  # characteristic UUID
CALIBRATION_TIME = 5  # seconds
n = 0
start_time = 0
end_time = 0
# Upper Arm - SU (Sensor Upperarm)
q_su_arr = []
q_su = np.array([1, 0, 0, 0])  
q_gu = np.array([1, 0, 0, 0])
S_su = np.zeros((4,4)) 

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
        
        def handle_data(s, data):
            # Get Data
            quat = struct.unpack('<4d', data)
            new_qsu = np.array([quat[0], quat[1], quat[2], quat[3]])

            # Initialize Counter
            global n
            if n == 0:
                global start_time
                start_time = time.time()
                print ("Starting calibration, don't move")
            else: 
                end_time = time.time()


            global q_su_arr, q_su,q_gu
            q_su_arr.append(new_qsu)
            if end_time - start_time < CALIBRATION_TIME:
                q_su = quaternion_mean(new_qsu, q_su_arr[0] , S_su, w=1)
                print("Calibrating... %.2f / %d seconds, don't move" % (end_time - start_time, CALIBRATION_TIME))
            elif end_time - start_time == CALIBRATION_TIME:
                print("Calibration complete! You can move now")
            else: 
                # Begin Calculation
                q_gu = quat_multiply(new_qsu, q_su)
                roll, pitch, yaw = quaternion_to_euler_zyx(q_gu)
                print("Euler Angles (radians): Roll: %.2g, Pitch: %.2g, Yaw: %.2g" % (roll, pitch, yaw))
            # Finishing Quaternion
            n += 1
            # print('Q1: %.2g, %.2g, %.2g, %.2g' % (quat_shoulder[0], quat_shoulder[1], quat_shoulder[2], quat_shoulder[3]))

        await client.start_notify(CHAR_UUID, handle_data)
        # await asyncio.sleep(100)

asyncio.run(main())
