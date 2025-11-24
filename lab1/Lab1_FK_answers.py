import numpy as np
from scipy.spatial.transform import Rotation as R

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data



def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    joint_name = []
    joint_parent = []
    joint_offset = []
    stack = []
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            line = lines[i].strip()
            if line.startswith('ROOT'):
                joint_name.append(line.split()[1])
                joint_parent.append(-1)
                stack.append(len(joint_name)-1)
            elif line.startswith('JOINT'):
                joint_name.append(line.split()[1])
                joint_parent.append(stack[-1])
                stack.append(len(joint_name)-1)
            elif line.startswith('End Site'):
                joint_name.append(joint_name[stack[-1]] + '_end')
                joint_parent.append(stack[-1])
                stack.append(len(joint_name)-1)
            elif line.startswith('OFFSET'):
                joint_offset.append([float(x) for x in line.split()[1:]])
            elif line.startswith('}'):
                stack.pop()

    joint_offset = np.array(joint_offset).reshape(len(joint_offset), 3)
    return joint_name, joint_parent, joint_offset




def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    joint_positions = np.zeros((len(joint_name), 3))
    joint_orientations = np.zeros((len(joint_name), 4))
    cnt = 0 # 通道计数器

    motion_data_frame = motion_data[frame_id]
    for i in range(len(joint_name)):
        joint =joint_name [i]
        parent_idx = joint_parent[i]

        if i == 0:
            joint_positions[0] = motion_data_frame[cnt:cnt+3]
            cnt += 3
            joint_orientations[0] =R.from_euler('XYZ', motion_data_frame[cnt:cnt+3], degrees=True).as_quat()
            cnt += 3

        elif joint.endswith("_end"):
            # End Site 没有通道数据，cnt 不增加
            parent_pos = joint_positions[parent_idx]
            parent_rot = R.from_quat(joint_orientations[parent_idx]) 

            joint_positions[i] = parent_pos + parent_rot.apply(joint_offset[i])
            joint_orientations[i] = joint_orientations[parent_idx] 
        else :
            rotation_local = R.from_euler('XYZ', motion_data_frame[cnt:cnt+3], degrees=True)
            cnt += 3

            parent_pos = joint_positions[parent_idx]
            parent_rot = R.from_quat(joint_orientations[parent_idx]) 

            joint_positions[i] =  parent_pos + parent_rot.apply(joint_offset[i])
            joint_orientations[i] =(parent_rot * rotation_local).as_quat()
    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致
        as_euler时也需要大写的XYZ
    """
    # 读取两份骨架拓扑（仅关节顺序和 EndSite 信息）
    joint_name_T, _, _ = part1_calculate_T_pose(T_pose_bvh_path)
    joint_name_A, _, _ = part1_calculate_T_pose(A_pose_bvh_path)

    # 读取 A-pose 的动作数据
    motion_data_A = load_motion_data(A_pose_bvh_path)
    # 计算每个关节在通道向量中的旋转起始索引（根6通道，其他3通道，EndSite 0通道）
    def build_rot_start_indices(joint_names):
        rot_start={}
        cnt = 0
        for idx, name in enumerate(joint_names):
            if idx ==0:
                rot_start[name] =3
                cnt = 6
            elif name.endswith('_end'):
                rot_start[name] = None
            else:
                rot_start[name] = cnt
                cnt+=3
        total_channels = cnt
        return rot_start, total_channels
    
    rot_start_A, total_A = build_rot_start_indices(joint_name_A)
    rot_start_T, total_T = build_rot_start_indices(joint_name_T)

    N = motion_data_A.shape[0]
    X_T =total_T
    motion_data = np.zeros((N, X_T), dtype=np.float64)

    for f in range(N):
        src = motion_data_A[f]
        dst = motion_data[f]

        for idx, name in enumerate(joint_name_T):
            cnt_src = rot_start_A [name]
            cnt_dst = rot_start_T [name]
            if idx ==0:
                dst[0:6] = src[0:6]
            elif name.endswith('_end'):
                continue
            elif name == 'lShoulder':
                angles =src[cnt_src:cnt_src+3]  
                rotation_local = R.from_euler('XYZ', angles, degrees=True)
                A2T_rot = R.from_euler('XYZ', [0, 0, -45], degrees=True)
                angles_T = (A2T_rot * rotation_local).as_euler('XYZ', degrees=True)
                dst[cnt_dst:cnt_dst+3] = angles_T
            elif name == 'rShoulder':
                angles =src[cnt_src:cnt_src+3]
                rotation_local = R.from_euler('XYZ', angles, degrees=True)
                A2T_rot = R.from_euler('XYZ', [0, 0, 45], degrees=True)
                angles_T = (A2T_rot * rotation_local).as_euler('XYZ', degrees=True)
                dst[cnt_dst:cnt_dst+3] = angles_T
            else:
                dst[cnt_dst:cnt_dst+3] = src[cnt_src:cnt_src+3]
    return motion_data

