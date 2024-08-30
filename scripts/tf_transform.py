import numpy as np
import tf.transformations as tr

def invert_transform(transform):
    """
    Inverts a 4x4 transformation matrix.
    
    :param transform: 4x4 numpy array representing the transformation matrix
    :return: 4x4 numpy array representing the inverted transformation matrix
    """
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    
    # Invert the rotation
    rotation_inv = np.transpose(rotation)
    
    # Invert the translation
    translation_inv = -np.dot(rotation_inv, translation)
    
    # Form the inverted transformation matrix
    transform_inv = np.identity(4)
    transform_inv[:3, :3] = rotation_inv
    transform_inv[:3, 3] = translation_inv
    
    return transform_inv

def transform_to_translation_quaternion(transform):
    """
    Converts a 4x4 transformation matrix into a translation vector and quaternion.

    :param transform: 4x4 numpy array representing the transformation matrix
    :return: tuple containing the translation vector (x, y, z) and quaternion (x, y, z, w)
    """
    # Extract the translation component (last column)
    translation = transform[:3, 3]

    # Convert the rotation matrix to a quaternion
    quaternion = tr.quaternion_from_matrix(transform)

    return translation, quaternion

def main():
    # Camera-to-IMU transformation matrix
    cam_to_imu = np.array([[-0.008348880356758986, 0.9999231732836108, -0.009162080943901807, 0.014124810978116763],
                           [0.04865538534203184, 0.009557762902442785, 0.9987698947432884, -0.040279739514793425],
                           [0.9987807315292015, 0.007892825776284085, -0.04873144392749215, -0.1381421947521823],
                           [0.0, 0.0, 0.0, 1.0]])


    # Invert the transformation matrix to get IMU-to-Camera
    imu_to_cam = invert_transform(cam_to_imu)

    # Get translation vector and quaternion
    translation, quaternion = transform_to_translation_quaternion(imu_to_cam)
    
    print("IMU to Camera Translation Vector:", translation)
    print("IMU to Camera Quaternion:", quaternion)

if __name__ == '__main__':
    main()