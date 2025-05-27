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




def map_throttle_to_engine(base_throttle, x_throttle, y_throttle, roll):
    engine1 = np.array([1.0, 0.0])
    engine2 = np.array([-0.5, math.sqrt(3)/2])  
    engine3 = np.array([-0.5, -math.sqrt(3)/2]) 
    
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    rotation_matrix = np.array([[cos_roll, -sin_roll],
                                [sin_roll, cos_roll]])
    
    rotated_engine1 = np.dot(rotation_matrix, engine1)
    rotated_engine2 = np.dot(rotation_matrix, engine2)
    rotated_engine3 = np.dot(rotation_matrix, engine3)
    
    x_y = np.array([x_throttle, y_throttle])
    
    return (base_throttle + np.dot(rotated_engine1, x_y),
            base_throttle + np.dot(rotated_engine2, x_y),
            base_throttle + np.dot(rotated_engine3, x_y))


if __name__ == "__main__":
    result = map_throttle_to_engine(0.5, 0.0, 0.0, 0)
    print("Mapped Throttle:", result)