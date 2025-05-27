from time import sleep
import krpc
import time
import numpy as np
import matplotlib.pyplot as plt
from transforms import get_ground_velocity,map_throttle_to_engine
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
    ground_vel_ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame, rotation=vessel.surface_reference_frame)
    ground_vel_ref_frame_flight = vessel.flight(ground_vel_ref_frame)
    vessel.control.sas = True

    # vessel_self_ref_flight = vessel.flight(vessel.reference_frame)

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
        y_velocity = ground_vel_ref_frame_flight.velocity[1]
        z_velocity = ground_vel_ref_frame_flight.velocity[2]
        y_direction = ground_vel_ref_frame_flight.direction[1]
        z_direction = ground_vel_ref_frame_flight.direction[2]
        current_time = time.time()
        if last_time == 0:
            # Initialize last_time to avoid division by zero
            last_time = current_time - sleep_duration
        duration = current_time - last_time
        print("dt: {0:0.2f}".format(duration))
        target_h_velocity = height_pid.clamp_compute(
            h, dt=duration, min_output=-20, max_output=20)
        target_velocity_y = direction_y_pid.clamp_compute(
            y_direction, dt=duration, min_output=-1.0, max_output=1.0)
        target_velocity_z = direction_z_pid.clamp_compute(
            z_direction, dt=duration, min_output=-1.0, max_output=1.0)
        print(
            "Target Direction Y-Z:{0:0.2f}, {1:0.2f}".format(target_velocity_y, target_velocity_z))
        velocity_y_pid.setpoint = target_velocity_y
        velocity_z_pid.setpoint = target_velocity_z
        velocity_h_pid.setpoint = target_h_velocity
        throttle_delta_y = velocity_y_pid.clamp_compute(
            y_velocity, dt=duration, min_output=-0.1, max_output=0.1)
        throttle_delta_z = velocity_z_pid.clamp_compute(
            z_velocity, dt=duration, min_output=-0.1, max_output=0.1)
        print(
            "Throttle Delta Y-Z:{0:0.2f}, {1:0.2f}".format(throttle_delta_y, throttle_delta_z))
        base_throttle = velocity_h_pid.clamp_compute(
            current_h_velocity, dt=duration, min_output=-0.5, max_output=0.5) + 0.48
        last_time = current_time
        print("Base Throttle:", base_throttle)

        for engine in vessel.parts.engines:
            engine.independent_throttle = True
        roll = ground_vel_ref_frame_flight.roll
        (engine1_throttle, engine2_throttle, engine3_throttle) = map_throttle_to_engine(base_throttle, throttle_delta_y, throttle_delta_z, roll)
        vessel.parts.engines[0].throttle = engine1_throttle
        vessel.parts.engines[1].throttle = engine2_throttle
        vessel.parts.engines[2].throttle = engine3_throttle

        conn.drawing.clear()
        # up
        conn.drawing.add_direction_from_com((0, 1, 0),  vessel.reference_frame)
        # left
        conn.drawing.add_direction_from_com(
            (1, 0, 0), vessel.reference_frame, visible=True)
        # right
        conn.drawing.add_direction_from_com(
            (0, 0, 1), vessel.reference_frame, visible=True)


        sleep(sleep_duration)


if __name__ == "__main__":
    main()
