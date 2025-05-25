from time import sleep
import krpc
import numpy as np
import matplotlib.pyplot as plt
from transforms import get_ground_velocity
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import math

def main():
    conn = krpc.connect()
    vessel = conn.space_center.active_vessel
   
    while True:
        # 地速
        ground_vel_ref_frame= conn.space_center.ReferenceFrame.create_hybrid(position=vessel.orbit.body.reference_frame,rotation=vessel.surface_reference_frame)
        ground_vel_ref_frame_flight = vessel.flight(ground_vel_ref_frame)
        print("Ground Velocity:",  ["{0:0.2f}".format(i) for i in ground_vel_ref_frame_flight.velocity])
        print("height:{0:0.2f}".format(ground_vel_ref_frame_flight.surface_altitude))
        sleep(1)


if __name__ == "__main__":
    main()