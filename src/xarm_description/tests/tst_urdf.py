import subprocess
import ikpy.chain
import os
import numpy as np
import math


def degs_to_input(x):
    #x = 0 to 500, x = -135 to 0 and x = 135 to 
    x = max(-134.99, min(x, 134.99)) 
    return int(500.*x/135. + 500.)

def degs_to_radians(x):
    return x * math.pi / 180.0

def scale_angles(angles, min_val=-2.35619, max_val=2.35619, out_max=1000):
    arr = np.array(angles, dtype=float)
    scaled = (arr - min_val) / (max_val - min_val) * out_max
    scaled = np.clip(scaled, 0, out_max)
    return scaled.astype(int).tolist()

xacro_file = "../urdf/xarm_v2.urdf"
urdf_file = "../urdf/xarm_v2_ik_final.urdf"
# Paths
convert_file = False
if convert_file:
    

    command = f"bash -c 'source ../../../install/setup.bash'"
    # command = f"bash -c source install/setup.bash && ros2 run xacro xacro {xacro_file} -o {urdf_file}'"
    subprocess.run(command, shell=True, check=True)


    # 1️⃣ Generate plain URDF from Xacro
    subprocess.run([
        "ros2", "run", "xacro", "xacro", xacro_file,
        "-o", urdf_file
    ], check=True)

# 2️⃣ Load chain from generated URDF
my_chain = ikpy.chain.Chain.from_urdf_file(
    urdf_file,
    active_links_mask=[False, True, True, True, True, True, False],
    base_elements=["xarm_base_link"]
)


print("Chain loaded with {} links".format(len(my_chain.links)))


# initialize parameters for inverse kinematics
oritent_axis = "Z"

z_orientation = np.array([0.0, 0.0, -1.0])  # world down, as a numpy vector

target_position = [0.0, 0.2, -0.1] # [0.1, 0.1, 0.1]
target_orientation = np.array(target_position) * np.array([1.0, 1.0, -1.0])  # world down, as a numpy vector
# target_orientation = np.array([0.5, 0.0, -0.5])  # world down, as a numpy vector

ik = my_chain.inverse_kinematics(
    target_position=target_position,
    target_orientation=target_orientation,
    orientation_mode=oritent_axis
)

for i in range(3):
    target_position = [0.0, 0.2, -0.1]
    target_orientation = target_orientation + z_orientation #np.array([0.5, 0.25, -5.0])  # world down, as a numpy vector

    ik = my_chain.inverse_kinematics(
        target_position=target_position,
        target_orientation=target_orientation,
        orientation_mode=oritent_axis, 
        initial_position=ik
    )

print(ik)
joint_pos = ik.tolist()

print("The angles of each joints are : ", joint_pos)

computed_position = my_chain.forward_kinematics(ik)
print("Computed position: %s, original position : %s" % (computed_position[:3, 3], target_position))
print("Computed position (readable) : %s" % [ '%.2f' % elem for elem in computed_position[:3, 3] ])

angles = joint_pos[:-1]
angles[3] = -angles[3]  # Invert the 4th joint angle

final_angles = scale_angles(angles)
print("Scaled angles: ", final_angles)


