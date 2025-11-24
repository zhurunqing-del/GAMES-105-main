import numpy as np
from scipy.spatial.transform import Rotation as R

def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):
    """
    完成函数，计算逆运动学
    输入: 
        meta_data: 为了方便，将一些固定信息进行了打包，见上面的meta_data类
        joint_positions: 当前的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 当前的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
        target_pose: 目标位置，是一个numpy数组，shape为(3,)
    输出:
        经过IK后的姿态
        joint_positions: 计算得到的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 计算得到的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
    """
    path, path_name, path1, path2 =  meta_data.get_path_from_root_to_end()
    max_iterations = 20
    tolerance = 0.01
    alpha = 0.5
    joint_orientations = np.zeros((len(meta_data.joint_name), 4))
    joint_positions = np.zeros((len(meta_data.joint_namee), 3))
    for iter in range(max_iterations):
        # for i in range(len(meta_data.joint_name)):
        #     joint =meta_data.joint_name [i]
        #     parent_idx = meta_data.joint_parent[i]

        #     if i == 0:
        #         joint_positions[0] = meta_data.joint_initial_position[0]

        #     elif joint.endswith("_end"):
        #         joint_positions[i] = joint_positions[parent_idx]+meta_data.joint_initial_position[i]

        #     else :
        #         joint_positions[i] = joint_positions[parent_idx]+meta_data.joint_initial_position[i]
        # 计算误差
        current_pos = joint_positions[path[-1]]
        error = target_pose - current_pos
        if np.linalg.norm(error) < tolerance: break
        J = np.zeros(3, (len(path1)-1)*3)
        for i in path1:
            u_x = R.from_quat(joint_orientations[path1[i]])* [1,0,0]
            u_y = R.from_quat(joint_orientations[path1[i]])* [0,1,0]
            u_z = R.from_quat(joint_orientations[path1[i]])* [0,0,1]
            r = joint_positions[path1[-1]]-joint_positions[path1[i]]
            










    
    return joint_positions, joint_orientations

def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    输入lWrist相对于RootJoint前进方向的xz偏移，以及目标高度，IK以外的部分与bvh一致
    """
    
    return joint_positions, joint_orientations

def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    输入左手和右手的目标位置，固定左脚，完成函数，计算逆运动学
    """
    
    return joint_positions, joint_orientations