import math
import random

class HeatingModel:
    """
    一个模拟对象加热和冷却过程的类。
    该模型考虑了加热功率、热容、与环境的热传递以及可选的随机温度扰动。
    温度更新采用常微分方程的解析解，以提高精度。
    增加了加热器余热（惯性）的模拟。
    """
    def __init__(self,
                 initial_temp: float = 20.0,
                 heat_capacity: float = 100.0,
                 heat_transfer_coeff: float = 0.1,
                 ambient_temp: float = 20.0,
                 heater_thermal_mass: float = 10.0,  # 新增: 加热器自身的热质量 (J/℃)
                 heater_to_object_transfer_coeff: float = 0.5): # 新增: 加热器到物体的热传递系数 (W/℃)
        """
        初始化加热模型。

        参数:
            initial_temp (float): 初始温度 (℃)。默认为 20.0。
            heat_capacity (float): 被加热物体的热容 (J/℃)。必须为正数。默认为 100.0。
            heat_transfer_coeff (float): 被加热物体与环境的热传递系数 (W/℃)。必须为非负数。默认为 0.1。
            ambient_temp (float): 环境温度 (℃)。默认为 20.0。
            heater_thermal_mass (float): 加热器自身的热质量 (J/℃)。默认为 10.0。用于模拟加热器存储热量的能力。
            heater_to_object_transfer_coeff (float): 加热器到被加热物体的热传递系数 (W/℃)。默认为 0.5。
        异常:
            ValueError: 如果 heat_capacity 不是正数或 heat_transfer_coeff 是负数。
        """
        if heat_capacity <= 0:
            raise ValueError("热容 (heat_capacity) 必须是正数。")
        if heat_transfer_coeff < 0:
            raise ValueError("热传递系数 (heat_transfer_coeff) 必须是非负数。")
        if heater_thermal_mass <= 0:
            raise ValueError("加热器热质量 (heater_thermal_mass) 必须是正数。")
        if heater_to_object_transfer_coeff < 0:
            raise ValueError("加热器到物体热传递系数 (heater_to_object_transfer_coeff) 必须是非负数。")


        self.current_temp: float = initial_temp
        self.heat_capacity: float = heat_capacity
        self.heat_transfer_coeff: float = heat_transfer_coeff
        self.ambient_temp: float = ambient_temp

        self.is_heating: bool = False
        self.heating_power: float = 0.0  # 加热元件产生的功率 (W)

        # 新增用于模拟惯性的属性
        self.heater_temp: float = initial_temp # 加热器自身的温度
        self.heater_thermal_mass: float = heater_thermal_mass
        self.heater_to_object_transfer_coeff: float = heater_to_object_transfer_coeff

        self.virtual_time: float = 0.0  # 虚拟时钟 (秒)

        self.disturbance_enabled: bool = False
        self.disturbance_sigma: float = 0.152  # 温度扰动的标准差
        self.disturbance_max: float = 1  # 温度扰动的最大值

    def set_heating(self, power: float):
        """
        设置加热功率并开始加热。

        参数:
            power (float): 加热元件产生的功率 (W)。建议为非负数。
        """
        if power < 0:
            pass
        self.is_heating = True
        self.heating_power = power

    def stop_heating(self):
        """停止加热，并将加热元件产生的功率设为0。"""
        self.is_heating = False
        self.heating_power = 0.0

    def advance_time(self, seconds: float):
        """
        推进虚拟时间，并根据流逝的时间更新模型状态。

        参数:
            seconds (float): 推进的时间 (秒)。必须为非负数。

        异常:
            ValueError: 如果 seconds 是负数。
        """
        if seconds < 0:
            raise ValueError("推进的时间 (seconds) 不能是负数。")
        if seconds == 0:
            return

        self._update_temperatures(seconds) # 调用新的更新方法
        self.virtual_time += seconds

    def get_current_temp(self) -> float:
        """获取当前被加热物体的温度 (℃)。"""
        return self.current_temp

    def get_heater_temp(self) -> float:
        """获取当前加热器自身的温度 (℃)。"""
        return self.heater_temp

    def get_current_time(self) -> float:
        """获取当前虚拟时间 (秒)。"""
        return self.virtual_time

    # 其他 setter 方法保持不变
    def set_heat_capacity(self, capacity: float):
        if capacity <= 0:
            raise ValueError("热容 (heat_capacity) 必须是正数。")
        self.heat_capacity = capacity

    def set_heat_transfer_coeff(self, coeff: float):
        if coeff < 0:
            raise ValueError("热传递系数 (heat_transfer_coeff) 必须是非负数。")
        self.heat_transfer_coeff = coeff

    def set_ambient_temp(self, temp: float):
        self.ambient_temp = temp

    def enable_disturbance(self, sigma: float = 0.5, max_val: float = 1.0):
        if sigma < 0:
            raise ValueError("sigma 不能是负数。")
        self.disturbance_enabled = True
        self.disturbance_sigma = sigma
        self.disturbance_max = max_val

    def disable_disturbance(self):
        self.disturbance_enabled = False

    def _update_temperatures(self, elapsed: float):
        """
        根据物理定律更新加热器和被加热物体的温度（内部方法）。
        这里使用简单的欧拉法进行迭代，因为涉及到两个耦合的微分方程。
        dT_object/dt = (Q_from_heater - Q_to_ambient) / C_object
        dT_heater/dt = (Q_from_power - Q_to_object) / C_heater
        """
        if elapsed <= 0:
            return

        # 参数别名
        To = self.current_temp          # 物体温度
        Th = self.heater_temp           # 加热器温度
        Ta = self.ambient_temp          # 环境温度
        Co = self.heat_capacity         # 物体热容
        Ch = self.heater_thermal_mass   # 加热器热质量
        ho_a = self.heat_transfer_coeff # 物体到环境热传递系数
        hh_o = self.heater_to_object_transfer_coeff # 加热器到物体热传递系数
        P_power = self.heating_power if self.is_heating else 0.0 # 加热元件产生的功率

        # 为了更准确地处理耦合系统，我们将时间步细分
        # 这是一种简单的迭代方法，比解析解更通用，但可能需要更小的步长来保证精度
        num_sub_steps = max(1, int(elapsed * 10)) # 每秒10个子步，至少1个
        dt = elapsed / num_sub_steps

        for _ in range(num_sub_steps):
            # 计算热流量
            # 加热器对物体传递的热量
            Q_heater_to_object = hh_o * (Th - To)

            # 物体对环境散失的热量
            Q_object_to_ambient = ho_a * (To - Ta)

            # 加热元件产生的热量 (只有当is_heating为True时才产生)
            Q_from_power = P_power

            # 更新加热器温度
            # dTh/dt = (Q_from_power - Q_heater_to_object) / Ch
            delta_Th = (Q_from_power - Q_heater_to_object) / Ch * dt
            self.heater_temp += delta_Th

            # 更新物体温度
            # dTo/dt = (Q_heater_to_object - Q_object_to_ambient) / Co
            delta_To = (Q_heater_to_object - Q_object_to_ambient) / Co * dt
            self.current_temp += delta_To

        # 添加随机扰动 (例如：传感器噪声或微小的未建模物理效应)
        if self.disturbance_enabled and self.disturbance_sigma > 0:
            disturbance = random.gauss(0, self.disturbance_sigma)
            disturbance = max(-self.disturbance_max, min(self.disturbance_max, disturbance))
            self.current_temp += disturbance

        # 确保温度不低于环境温度 (针对物体温度)
        self.current_temp = max(self.current_temp, self.ambient_temp)
        # 加热器温度也应该有其物理限制，例如不低于环境温度（如果它与环境也有热交换，但这里简化为只与物体交换）
        # 暂时不强制加热器温度下限，因为它可能比物体温度低，如果物体温度已经很高而加热器停止加热


if __name__ == "__main__":
    # 创建模型实例，并设置一些参数来增强惯性效果
    # 增加 heater_thermal_mass 让加热器能存储更多热量
    # 调整 heater_to_object_transfer_coeff 让热量传递更明显
    model = HeatingModel(
        initial_temp=20.0,
        heat_capacity=1000.0, # 物体热容大一点，升温慢，惯性更明显
        heat_transfer_coeff=1, # 物体散热慢一点
        ambient_temp=20.0,
        heater_thermal_mass=60.0, # 加热器热质量大，能存储更多余热
        heater_to_object_transfer_coeff=5 # 加热器到物体热传递适中
    )

    print(f"初始温度: 物体={model.get_current_temp():.2f}°C, 加热器={model.get_heater_temp():.2f}°C")

    # 阶段1: 加热一段时间
    print("\n--- 阶段1: 开始加热 (功率1000W) ---")
    model.set_heating(1000.0)
    for _ in range(60): # 加热60秒
        model.advance_time(1.0)
        print(f"时间: {model.get_current_time():.0f}s, 物体温度: {model.get_current_temp():.2f}°C, 加热器温度: {model.get_heater_temp():.2f}°C")

    # 阶段2: 停止加热，观察惯性
    print("\n--- 阶段2: 停止加热，观察惯性升温 ---")
    model.stop_heating()
    for _ in range(30): # 停止加热后继续模拟30秒
        model.advance_time(1.0)
        print(f"时间: {model.get_current_time():.0f}s, 物体温度: {model.get_current_temp():.2f}°C, 加热器温度: {model.get_heater_temp():.2f}°C")
        # 可以在这里观察到物体温度在加热器停止工作后仍有短暂升高

    # 阶段3: 冷却
    print("\n--- 阶段3: 自然冷却 ---")
    for _ in range(60): # 冷却60秒
        model.advance_time(1.0)
        print(f"时间: {model.get_current_time():.0f}s, 物体温度: {model.get_current_temp():.2f}°C, 加热器温度: {model.get_heater_temp():.2f}°C")

    # 启用扰动测试
    print("\n--- 启用扰动测试 ---")
    model.enable_disturbance(magnitude=0.1)
    for _ in range(10):
        model.advance_time(1.0)
        print(f"时间: {model.get_current_time():.0f}s, 物体温度 (带扰动): {model.get_current_temp():.2f}°C")