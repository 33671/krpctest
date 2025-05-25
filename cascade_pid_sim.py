from heater_model import HeatingModel
from pid import PidController
import numpy as np
import matplotlib.pyplot as plt
import math

if __name__ == "__main__":
    model = HeatingModel(
        initial_temp=20.0,
        heat_capacity=200.0,  # 物体热容
        heat_transfer_coeff=2,
        ambient_temp=20.0,
        heater_thermal_mass=60.0,
        heater_to_object_transfer_coeff=5
    )
    model.enable_disturbance(sigma=0.213, max_val=1.0)  # 启用扰动，模拟环境变化

    # 外环PID - 控制物体温度
    outer_pid = PidController(kp=7.5, ki=0.001, kd=1.5, setpoint=70.0)
    # 内环PID - 控制加热器温度
    inner_pid = PidController(kp=2.8, ki=0.001, kd=0.5, setpoint=0.0)  # setpoint将由外环动态设置

    # 准备数据存储
    time_points = []
    temp_points = []
    heater_temp_points = []
    outer_pid_outputs = []
    inner_pid_outputs = []
    heating_states = []
    
    print(f"初始温度: {model.get_current_temp():.2f}°C (时间: {model.get_current_time():.1f}s)")

    dt = 6.0  # 控制周期(秒)
    waiting_time = 0 
    
    while model.get_current_time() < 600:
        current_heating_state = 1 if waiting_time > 0 else 0
        
        if waiting_time <= 0:
            waiting_time = 0
            model.stop_heating()

        if math.floor(model.get_current_time()) % int(dt) == 0:
            # 外环控制 - 计算需要的加热器温度设定值
            current_temp = model.get_current_temp()
            outer_output_temp_delta = outer_pid.compute(current_temp, dt)
            
            # 将外环输出作为内环的设定值
            # 限制加热器温度设定值在合理范围内(如70-150°C)
            heater_setpoint = max(70, min(200, current_temp + outer_output_temp_delta))
            inner_pid.setpoint = heater_setpoint
            
            # 内环控制 - 计算需要的加热功率
            current_heater_temp = model.get_heater_temp()
            inner_output = inner_pid.compute(current_heater_temp, dt)
            
            if inner_output > 0:
                model.set_heating(500)
                waiting_time = math.ceil(dt)  # 保持加热直到下一个控制周期
                current_heating_state = 1
            else:
                model.stop_heating()
                waiting_time = 0
                current_heating_state = 0

        # 记录数据
        time_points.append(model.get_current_time())
        temp_points.append(model.get_current_temp())
        heater_temp_points.append(model.get_heater_temp())
        outer_pid_outputs.append(outer_output_temp_delta)
        inner_pid_outputs.append(inner_output)
        heating_states.append(current_heating_state)
        
        model.advance_time(1)
        waiting_time -= 1

    # 绘制结果
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # 温度曲线
    color = 'tab:red'
    ax1.set_ylabel('temp', color=color)
    ax1.plot(time_points, temp_points, color=color, label='liquid temp')
    ax1.plot(time_points, heater_temp_points, color='tab:purple', label='heater temp')
    ax1.axhline(y=70, color='blue', linestyle='--', label='target temp')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.set_ylim(0, max(max(temp_points), max(heater_temp_points))*1.1)
    ax1.grid(True)
    ax1.legend(loc='upper right')
    
    # PID输出
    ax2.set_ylabel('PID out')
    ax2.plot(time_points, outer_pid_outputs, color='tab:green', linestyle='-', label='outer')
    ax2.plot(time_points, inner_pid_outputs, color='tab:blue', linestyle='-', label='inner')
    ax2.tick_params(axis='y')
    max_pid = max(abs(min(min(outer_pid_outputs), min(inner_pid_outputs))), 
                 abs(max(max(outer_pid_outputs), max(inner_pid_outputs))))
    ax2.set_ylim(-max_pid*1.5, max_pid*1.5)
    ax2.axhline(y=0, color='black', linewidth=0.5)
    ax2.legend(loc='upper left')
    ax2.grid(True)
    
    # 加热状态
    ax3.step(time_points, heating_states, where='post', color='tab:orange', label='heating state (1=on, 0=off)')
    ax3.set_xlabel('time (s)')
    ax3.set_ylabel('heating state')
    ax3.set_yticks([0, 1])
    ax3.set_ylim(-0.1, 1.1)
    ax3.grid(True)
    ax3.legend(loc='upper right')
    
    plt.suptitle('Cascade PID Control Simulation', fontsize=16)
    plt.tight_layout()
    plt.show()