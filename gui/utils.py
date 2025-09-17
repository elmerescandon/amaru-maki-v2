import numpy as np

def quaternion_mean(q, q_ref, S, w = 1):
    # Normalize Input Quaternion
    q_normalized = normalize_quaternion(q, q_ref)

    if np.dot(q, q_ref) < 0:
        q_normalized = -q_normalized

    print("Normalized Quaternion (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (q_normalized[0], q_normalized[1], q_normalized[2], q_normalized[3]))
    Q = np.outer(q_normalized, q_normalized)
    print ("Outer Product Q:\n", Q)
    S_new = S + w*Q
    print("Updated Covariance Matrix S:\n", S_new)

    eigvals, eigvecs = np.linalg.eigh(S_new)  # eigh() is for symmetric matrices
    max_index = np.argmax(eigvals)
    q_selected = eigvecs[:, max_index]
    print ("Selected Eigenvector (before normalization) (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (q_selected[0], q_selected[1], q_selected[2], q_selected[3]))

    q_selected =q_selected/np.linalg.norm(q_selected)
    print ("Updated Mean Quaternion (after normalization) (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (q_selected[0], q_selected[1], q_selected[2], q_selected[3]))
    return q_selected, S_new

def normalize_quaternion(q, q_ref):
    return q * (1 if (q_ref.T).dot(q) >= 0 else -1)


def quat_multiply(q1, q2):
    """
    Multiply two quaternions q1 * q2 (Hamilton product).
    q1, q2: array-like, shape (4,) [w, x, y, z]
    Returns: numpy array (4,) quaternion product.
    """
    w1 = q1[0]; x1 = q1[1]; y1 = q1[2]; z1 = q1[3]
    w2 = q2[0]; x2 = q2[1]; y2 = q2[2]; z2 = q2[3]

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)  # normalize result to unit length


def quaternion_to_euler_zyx(q):
    """
    Convert a quaternion (w, x, y, z) to Euler angles (ZYX order: roll, pitch, yaw).
    Returns (x, y, z) angles in radians.
    """
    w, x, y, z = q
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z

