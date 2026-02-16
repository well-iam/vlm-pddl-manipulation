import numpy as np
from scipy.spatial.transform import Rotation as R

calibration_translation = [0.73624, 0.633062, 0.304491]
calibration_quaternion = [0.299594, 0.775731, -0.518128, -0.200071]

# Internal RealSense Data (camera_link -> camera_color_optical_frame)
# DO NOT MODIFY ---------------------------------------------------
internal_translation = [0.0, 0.014, 0.0]
internal_quaternion = [-0.499, 0.501, -0.495, 0.505]
# ---------------------------------------------------------------

def get_matrix(trans, quat):
    mat = np.eye(4)
    mat[:3, 3] = trans
    mat[:3, :3] = R.from_quat(quat).as_matrix()
    return mat

def main():
    # Creatae 4x4 matrix
    T_base_optical = get_matrix(calibration_translation, calibration_quaternion)
    T_camera_optical = get_matrix(internal_translation, internal_quaternion)

    # Compute the inverse matrix of the internal offset (Optical -> Camera Link)
    T_optical_camera = np.linalg.inv(T_camera_optical)

    # Compute the final transform (Base -> Link)
    # Base->Camera_link
    T_base_camera = np.dot(T_base_optical, T_optical_camera)

    # Extract translation and rotation
    final_trans = T_base_camera[:3, 3]
    final_rot_matrix = T_base_camera[:3, :3]
    final_quat = R.from_matrix(final_rot_matrix).as_quat()

    print(f"Translation: {final_trans[0]:.6f} {final_trans[1]:.6f} {final_trans[2]:.6f}")
    print(f"Rotation: {final_quat[0]:.6f} {final_quat[1]:.6f} {final_quat[2]:.6f} {final_quat[3]:.6f}")

if __name__ == '__main__':
    main()