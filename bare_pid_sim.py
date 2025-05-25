from heater_model import HeatingModel
from pid import PidController
import numpy as np
import matplotlib.pyplot as plt
import math

if __name__ == "__main__":
    model = HeatingModel(
        initial_temp=20.0,
        heat_capacity=200.0, # 物体热容大一点，升温慢，惯性更明显
        heat_transfer_coeff=2, # 物体散热慢一点
        ambient_temp=20.0,
        heater_thermal_mass=60.0, # 加热器热质量大，能存储更多余热
        heater_to_object_transfer_coeff=5 # 加热器到物体热传递适中
    )
    # model.enable_disturbance(sigma=0.213, max_val=1.0)  # 启用扰动，模拟环境变化
    # model.disable_disturbance()

    pid = PidController(kp=2.2, ki=.0001, kd=2, setpoint=70.0)

    # Prepare data storage
    time_points = []
    temp_points = []
    pid_outputs = []
    heating_states = []  # Will store 1 (heating) or 0 (not heating)
    
    print(f"Initial temperature: {model.get_current_temp():.2f}°C (Time: {model.get_current_time():.1f}s)")

    dt = 6.0
    waiting_time = 0 
    while model.get_current_time() < 1000:
        current_heating_state = 1 if waiting_time > 0 else 0
        
        if waiting_time <= 0:
            waiting_time = 0
            model.stop_heating()

        if math.floor(model.get_current_time()) % int(dt) == 0:
            current_temp = model.get_current_temp()
            pid_output = pid.compute(current_temp, dt)
            if pid_output > 0:
                model.set_heating(500)
                waiting_time = math.ceil(pid_output)
                current_heating_state = 1
            else:
                model.stop_heating()
                waiting_time = 0
                current_heating_state = 0

        # Record data
        time_points.append(model.get_current_time())
        temp_points.append(model.get_current_temp())
        pid_outputs.append(pid_output)
        heating_states.append(current_heating_state)
        
        model.advance_time(1)
        waiting_time -= 1

    # Plotting code remains the same as in the previous corrected version
    fig, (ax1, ax3) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Temperature plot
    color = 'tab:red'
    ax1.set_ylabel('Temperature (°C)', color=color)
    ax1.plot(time_points, temp_points, color=color, label='Temperature')
    ax1.axhline(y=70, color='blue', linestyle='--', label='Target (70°C)')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.set_ylim(0, max(temp_points)*1.1)
    ax1.grid(True)
    ax1.legend(loc='upper right')
    
    # PID output
    ax2 = ax1.twinx()
    color = 'tab:green'
    ax2.set_ylabel('PID Output', color=color)
    ax2.plot(time_points, pid_outputs, color=color, linestyle=':', label='PID Output')
    ax2.tick_params(axis='y', labelcolor=color)
    max_pid = max(abs(min(pid_outputs)), abs(max(pid_outputs)))
    ax2.set_ylim(-max_pid*1.5, max_pid*1.5)
    ax2.axhline(y=0, color='black', linewidth=0.5)
    ax2.legend(loc='upper left')
    
    # Heating state
    ax3.step(time_points, heating_states, where='post', color='tab:orange', label='Heating (1=On, 0=Off)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Heating State')
    ax3.set_yticks([0, 1])
    ax3.set_ylim(-0.1, 1.1)
    ax3.grid(True)
    ax3.legend(loc='upper right')
    
    plt.suptitle('Temperature Control with PID Feedback')
    plt.tight_layout()
    plt.show()