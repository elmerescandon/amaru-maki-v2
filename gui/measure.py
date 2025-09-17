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
finished_calibration = False
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
            quat = struct.unpack('<12d', data)
            global new_qsu
            new_qsu = np.array([quat[0], quat[1], quat[2], quat[3]])

            # Initialize Counter
            global n, start_time, end_time
            if n == 0:
                start_time = time.time()
                print ("Starting calibration, don't move")
            else: 
                end_time = time.time()


            global q_su_arr, q_su,q_gu, S_su
            q_su_arr.append(new_qsu)
            if end_time - start_time < CALIBRATION_TIME:
                q_su, S_su = quaternion_mean(new_qsu, q_su_arr[0] , S_su, w=1)
                print("Calibrating... %.2f / %d seconds, don't move. Quaternion (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (end_time - start_time, CALIBRATION_TIME, q_su[0], q_su[1], q_su[2], q_su[3]))
            else: 
                global finished_calibration
                if not finished_calibration:
                    finished_calibration = True
                    print("Calibration complete! You can move now")
                    # print("Initial Upper Arm Quaternion (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (q_su[0], q_su[1], q_su[2], q_su[3]))
                # Begin Calculation
                q_gu = quat_multiply(new_qsu, q_su)
                # print("Upper Arm Quaternion (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (q_gu[0], q_gu[1], q_gu[2], q_gu[3]))
                roll, pitch, yaw = quaternion_to_euler_zyx(q_gu)
                print("Euler Angles (radians): Roll: %.2g, Pitch: %.2g, Yaw: %.2g" % (roll*180/np.pi, pitch*180/np.pi, yaw*180/np.pi))
            n += 1

        await client.start_notify(CHAR_UUID, handle_data)
        await asyncio.sleep(100)

asyncio.run(main())
