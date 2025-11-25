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
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
        
    # 获取从根节点到末端执行器的路径
    path_to_end_effector = path1
    
    # 迭代参数
    max_iterations = 20
    tolerance = 0.01
    alpha = 0.1
    
    # 复制一份以防修改原始数据
    ik_joint_positions = joint_positions.copy()
    ik_joint_orientations = joint_orientations.copy()

    for iter in range(max_iterations):
        
        # 1. 正向动力学 (FK)，更新所有关节的位置
        for i in range(len(meta_data.joint_name)):
            if meta_data.joint_parent[i] == -1: # Root joint
                continue
            parent_idx = meta_data.joint_parent[i]
            parent_orientation = R.from_quat(ik_joint_orientations[parent_idx])
            
            # 从父关节旋转和初始偏移计算当前关节的位置
            ik_joint_positions[i] = ik_joint_positions[parent_idx] + parent_orientation.apply(meta_data.joint_initial_position[i])

        # 2. 计算误差
        end_effector_pos = ik_joint_positions[path_to_end_effector[-1]]
        error = target_pose - end_effector_pos
        
        # 检查是否收敛
        if np.linalg.norm(error) < tolerance:
            break
            
        # 3. 构建雅可比矩阵 J
        # 路径中除了末端执行器外的所有关节都可以旋转
        movable_joints = path_to_end_effector[:-1]
        J = np.zeros((3, len(movable_joints) * 3))
        
        for i, joint_idx in enumerate(movable_joints):
            joint_pos = ik_joint_positions[joint_idx]
            joint_orientation = R.from_quat(ik_joint_orientations[joint_idx])
            
            # 计算从当前关节到末端执行器的向量
            r = end_effector_pos - joint_pos
            
            # 计算世界坐标系下的旋转轴
            axis_x = joint_orientation.apply([1, 0, 0])
            axis_y = joint_orientation.apply([0, 1, 0])
            axis_z = joint_orientation.apply([0, 0, 1])
            
            # 计算叉乘，并填入雅可比矩阵
            J[:, i*3 + 0] = np.cross(axis_x, r)
            J[:, i*3 + 1] = np.cross(axis_y, r)
            J[:, i*3 + 2] = np.cross(axis_z, r)

        # 4. 使用雅可比转置法求解关节角度增量
        delta_theta = alpha * J.T @ error

        # 5. 更新关节旋转
        for i, joint_idx in enumerate(movable_joints):
            # 取出当前关节的角度增量 (旋转向量)
            d_theta = delta_theta[i*3 : i*3 + 3]
            
            # 转换为旋转对象并更新
            delta_rot = R.from_rotvec(d_theta)
            current_rot = R.from_quat(ik_joint_orientations[joint_idx])
            new_rot = delta_rot * current_rot
            
            # 标准化四元数并更新
            ik_joint_orientations[joint_idx] = new_rot.as_quat()
            

    # 在循环结束后，最后进行一次FK以确保位置和旋转是匹配的
    # for i in range(len(meta_data.joint_name)):
    #     if meta_data.joint_parent[i] == -1:
    #         continue
    #     parent_idx = meta_data.joint_parent[i]
    #     parent_orientation = R.from_quat(ik_joint_orientations[parent_idx])
    #     ik_joint_positions[i] = ik_joint_positions[parent_idx] + parent_orientation.apply(meta_data.joint_initial_position[i])
        
    return ik_joint_positions, ik_joint_orientations


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