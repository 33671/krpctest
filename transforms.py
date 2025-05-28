import numpy as np
from scipy.spatial.transform import Rotation
import math

def get_trans_to_surface(latitude, longitude):
        """计算从全局坐标系到地表坐标系的旋转变换（四元数）"""
        # 绕 Z 轴旋转 -latitude（纬度）
        rot_z = Rotation.from_euler('z', -latitude, degrees=True)
        # 绕 Y 轴旋转 -longitude（经度）

        rot_y = Rotation.from_euler('y', -longitude, degrees=True)
        # 组合旋转：先经度（Y），后纬度（Z）
        total_rot = rot_z * rot_y
        return total_rot
def get_ground_velocity(vx_global, vy_global, vz_global, latitude, longitude):
    """将全局坐标系下的速度转换到地表局部坐标系"""
    trans_to_surface = get_trans_to_surface(latitude, longitude)

    # 全局速度（注意 Z 轴取反，可能由于坐标系定义不同）
    v_global = np.array([vx_global, vy_global, -vz_global])
    # 应用旋转
    v_local = trans_to_surface.apply(v_global)
    return v_local


def cross_product(u, v):
    return (u[1]*v[2] - u[2]*v[1],
            u[2]*v[0] - u[0]*v[2],
            u[0]*v[1] - u[1]*v[0])


def dot_product(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]


def magnitude(v):
    return math.sqrt(dot_product(v, v))


def angle_between_vectors(u, v):
    """ Compute the angle between vector u and v """
    dp = dot_product(u, v)
    if dp == 0:
        return 0
    um = magnitude(u)
    vm = magnitude(v)
    return math.acos(dp / (um*vm)) * (180. / math.pi)


def map_throttle_to_engine(base_throttle, x_throttle, y_throttle, roll):
    import math
import numpy as np

def map_throttle_to_engine(base_throttle, x_throttle, y_throttle, roll):
    # 定义原始引擎方向向量（120度均匀分布）
    orig_engine1 = np.array([1.0, 0.0])
    orig_engine2 = np.array([-0.5, math.sqrt(3)/2])
    orig_engine3 = np.array([-0.5, -math.sqrt(3)/2])
    
    # 将roll角度转换为弧度
    roll_rad = np.radians(roll)
    
    # 构建旋转矩阵
    rotation_matrix = np.array([
        [np.cos(roll_rad), -np.sin(roll_rad)],
        [np.sin(roll_rad),  np.cos(roll_rad)]
    ])
    
    # 应用旋转矩阵到每个引擎方向向量
    norm_engine1 = rotation_matrix @ orig_engine1
    norm_engine2 = rotation_matrix @ orig_engine2
    norm_engine3 = rotation_matrix @ orig_engine3
    
    # 计算节流阀值
    x_y = np.array([x_throttle, y_throttle])
    return (
        base_throttle + np.dot(norm_engine1, x_y),
        base_throttle + np.dot(norm_engine2, x_y),
        base_throttle + np.dot(norm_engine3, x_y)
    )


if __name__ == "__main__":
    result = map_throttle_to_engine(0.5, 0.0, 0.0, 0)
    print("Mapped Throttle:", result)