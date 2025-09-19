import asyncio
import threading
import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from scipy.spatial.transform import Rotation as R
from bleak import BleakScanner, BleakClient
import struct
import time
from utils import *

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
S_su = np.eye((4))*10e-6 
new_qsu = np.array([1, 0, 0, 0])  # Initialize with a default quaternion

async def handle_data(s, data):
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


async def ble_task():
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
        await client.start_notify(CHAR_UUID, handle_data)
        print("BLE connected. Listening for quaternion data...")
        while True:
            await asyncio.sleep(0.01)

def start_ble_in_background():
    thread = threading.Thread(target=lambda: asyncio.run(ble_task()), daemon=True)
    thread.start()

# ---------- OPENGL RENDERING ----------

# Cube vertices and edges
cube_vertices = [
    [-1, -1, -1],
    [ 1, -1, -1],
    [ 1,  1, -1],
    [-1,  1, -1],
    [-1, -1,  1],
    [ 1, -1,  1],
    [ 1,  1,  1],
    [-1,  1,  1]
]

cube_edges = (
    (0,1), (1,2), (2,3), (3,0),
    (4,5), (5,6), (6,7), (7,4),
    (0,4), (1,5), (2,6), (3,7)
)

def draw_cube():
    glBegin(GL_LINES)
    for edge in cube_edges:
        for vertex in edge:
            glVertex3fv(cube_vertices[vertex])
    glEnd()

def quaternion_to_matrix(q):
    """Convert quaternion [w, x, y, z] to 4x4 rotation matrix."""
    r = R.from_quat([q[1], q[2], q[3], q[0]])  # scipy expects (x, y, z, w)
    rot_mat = np.eye(4)
    rot_mat[:3, :3] = r.as_matrix()
    # First rotate +90 degrees around Y, then +180 degrees around Z
    y_rot = np.eye(4)
    y_rot[:3, :3] = R.from_euler('y', 90, degrees=True).as_matrix()
    z_rot = np.eye(4)
    z_rot[:3, :3] = R.from_euler('z', 180, degrees=True).as_matrix()
    rot_mat = z_rot @ y_rot @ rot_mat
    return rot_mat.T  # transpose for OpenGL's column-major order

def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Apply quaternion rotation
        # q = np.array(q_gu)
        global q_gu
        rot_matrix = quaternion_to_matrix(q_gu)  # Use the latest quaternion directly for testing
        glPushMatrix()
        glMultMatrixf(rot_matrix.flatten())

        draw_cube()
        glPopMatrix()

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    start_ble_in_background()
    main()
