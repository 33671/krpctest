from time import sleep
import krpc
import time
import numpy as np
import matplotlib.pyplot as plt
from transforms import get_ground_velocity
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import math
from pid import PidController
def main():
    conn = krpc.connect()
    vessel = conn.space_center.active_vessel
    # ground is y-z plane, x is up

    # velocity y-z --> direction y-z plane --> 3 throttle delta 
    direction_y_pid = PidController(kp=7.5, ki=0.001, kd=1.5, setpoint=1.0)
    direction_z_pid = PidController(kp=7.5, ki=0.001, kd=1.5, setpoint=1.0)
    velocity_y_pid = PidController(kp=2.8, ki=0.001, kd=0.5, setpoint=0.0)  
    velocity_z_pid = PidController(kp=0.25, ki=0.001, kd=0.5, setpoint=0.0)
    # height --> velocity
    height_pid = PidController(kp=0.015, ki=0.002, kd=0.08, setpoint=200.0)
    # velocity --> throttle
    velocity_h_pid = PidController(kp=0.01, ki=0.001, kd=0.1, setpoint=0.0) 
    for engine in vessel.parts.engines:
        engine.independent_throttle = False
        engine.throttle = 0.0
        if not engine.active:
            engine.active = True
    last_time = 0
  
    # panel.add_button("Clear", conn.drawing.clear)
    sleep_duration = 0.1
    ground_vel_ref_frame= conn.space_center.ReferenceFrame.create_hybrid(position=vessel.orbit.body.reference_frame,rotation=vessel.surface_reference_frame)
    ground_vel_ref_frame_flight = vessel.flight(ground_vel_ref_frame)
    vessel.control.sas = True 
    
    while True:
        # conn.ui.clear()
        # canvas = conn.ui.add_canvas()
        # panel = canvas.add_panel()
        # panel.rect_transform.position = (10,10)
        # text = panel.add_text("Ground Velocity:")
        # 地速
        vessel.control.sas_mode = conn.space_center.SASMode.radial
        # print("Ground Velocity:",  ["{0:0.2f}".format(i) for i in ground_vel_ref_frame_flight.velocity])
        current_h_velocity = ground_vel_ref_frame_flight.velocity[0]
        print("currrent height velocity:", current_h_velocity)
        h = ground_vel_ref_frame_flight.surface_altitude
        print("height:{0:0.2f}".format(h))
        # get direction vector
        # d = ground_vel_ref_frame_flight.direction
        # print("Direction Vector:",["{0:0.2f}".format(i) for i in d])

        current_time = time.time()
        if last_time == 0:
            last_time = current_time - sleep_duration  # Initialize last_time to avoid division by zero
        duration = current_time - last_time
        print("dt: {0:0.2f}".format(duration))
        target_h_velocity = height_pid.clamp_compute(h,dt=duration,min_output=-20,max_output=20)
        velocity_h_pid.setpoint = target_h_velocity
        
        base_throttle = velocity_h_pid.clamp_compute(current_h_velocity, dt=duration, min_output=-0.5, max_output=0.5) + 0.48
        last_time = current_time
        print("Base Throttle:", base_throttle)
        # for engine in vessel.parts.engines:
            # engine.throttle = base_throttle
        vessel.control.throttle = base_throttle
       

        conn.drawing.clear()
         # up
        conn.drawing.add_direction_from_com((0, 1, 0),  vessel.reference_frame)
        # left
        conn.drawing.add_direction_from_com((1, 0, 0), vessel.reference_frame,visible=True)
        # right
        conn.drawing.add_direction_from_com((0, 0, 1), vessel.reference_frame,visible=True)

        # text.content = "Throttle: {0:0.2f}".format(base_throttle)

        # canvas.add_text("Ground Velocity: {0:0.2f}, {1:0.2f}, {2:0.2f}".format(*ground_vel_ref_frame_flight.velocity))
        # conn.ui.stock_canvas.add_text("throttle: {0:0.2f}".format(base_throttle))
        # conn.
        # print()


        sleep(sleep_duration)


if __name__ == "__main__":
    main()
