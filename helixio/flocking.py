from time import time
import numpy as np
import random
import math
from mavsdk.offboard import VelocityNedYaw, AccelerationNed, PositionNedYaw
import pymap3d as pm

def check_velocity(
    desired_vel, my_pos_vel, max_speed, yaw, time_step, max_accelleration
):
    # current_vel = np.array(my_pos_vel.velocity_ned)

    # impose velocity limit
    if np.linalg.norm(desired_vel) > max_speed:
        desired_vel = desired_vel / np.linalg.norm(desired_vel) * max_speed

    # impose accelleration limit
    # output_vel = limit_accelleration(
    #     desired_vel, current_vel, time_step, max_accelleration
    # )

    # yaw = 0.0
    return VelocityNedYaw(desired_vel[0], desired_vel[1], desired_vel[2], yaw)

def check_acceleration(
    desired_acc, my_pos_vel, max_acc, yaw, time_step, max_accelleration
):
    # current_vel = np.array(my_pos_vel.velocity_ned)

    # impose velocity limit
    if np.linalg.norm(desired_acc) > max_acc:
        desired_acc = desired_acc / np.linalg.norm(desired_acc) * max_acc
    
    print(desired_acc)
    # impose accelleration limit
    # output_vel = limit_accelleration(
    #     desired_vel, current_vel, time_step, max_accelleration
    # )

    # yaw = 0.0
    return AccelerationNed(desired_acc[0], desired_acc[1], desired_acc[2])

def check_position(
    desired_pos,my_pos_vel, max_speed, yaw, time_step, reference_point, home_position
):
    # current_vel = np.array(my_pos_vel.velocity_ned)

    # impose velocity limi

    # impose accelleration limit
    # output_vel = limit_accelleration(
    #     desired_vel, current_vel, time_step, max_accelleration
    # )

    # yaw = 0.0
    home_ref_vector=pm.geodetic2ned(
                reference_point[0],
                reference_point[1],
                reference_point[2],
                home_position[0],
                home_position[1],
                home_position[2],
            )
    for i in range(3):
        desired_pos[i]=desired_pos[i]+home_ref_vector[i] # to obtain the desired position with respect to the home point
    return PositionNedYaw(desired_pos[0], desired_pos[1], desired_pos[2],0)


# NOT NEEDED
def limit_accelleration(desired_vel, current_vel, time_step, max_accel_mag):
    delta_v = desired_vel - current_vel
    accelleration = delta_v / time_step
    accelleration_mag = np.linalg.norm(accelleration)

    # impose accelleration limit (seems to cause the loop to fail but px4 limits accelleration so not needed)
    if accelleration_mag > max_accel_mag:
        max_accel = (accelleration / np.linalg.norm(accelleration)) * max_accel_mag
        output_vel = (max_accel * time_step) + current_vel
    output_vel = desired_vel
    return output_vel


def simple_flocking(drone_id, swarm_pos_vel, my_pos_vel, time_step, max_accel):
    com = np.array([0, 0, 0])
    k_cohesion = 1
    for key in swarm_pos_vel:
        p = np.array(swarm_pos_vel[key].position_ned)
        com = com + p
    com = com / len(swarm_pos_vel)
    v_cohesion = (
        k_cohesion
        * (com - np.array(my_pos_vel.position_ned))
        / np.linalg.norm(com - np.array(my_pos_vel.position_ned))
    )

    # changing from 10
    r_0 = 20
    v_separation = np.array([0, 0, 0])
    for key in swarm_pos_vel:
        if key == drone_id:
            continue
        p = np.array(swarm_pos_vel[key].position_ned)
        x = np.array(my_pos_vel.position_ned) - p
        d = np.linalg.norm(x)
        if d <= r_0:
            v_separation = v_separation + ((x / d) * (r_0 - d) / r_0)

    output_vel = v_cohesion + v_separation

    output_vel = limit_accelleration(
        output_vel, np.array(my_pos_vel.velocity_ned), time_step, max_accel
    )

    return output_vel


def get_desired_yaw(north, east):
    # rho = np.sqrt(x**2 + y**2)
    yaw = np.arctan2(east, north)
    yaw = yaw * 180 / np.pi
    return yaw


def migration_test(migrated):
    if migrated == True:
        north = 0
    else:
        north = 200

    east = random.randint(0, 50)
    down = -20

    return [north, east, down]


def velocity_to_point(my_pos_vel, desired_pos):
    k = 40
    desired_pos = np.array(desired_pos)
    current_pos = np.array(my_pos_vel.position_ned)
    unit_vector = (desired_pos - current_pos) / np.linalg.norm(
        desired_pos - current_pos
    )

    output_vel = unit_vector * k
    yaw = get_desired_yaw(output_vel[0], output_vel[1])

    return output_vel, yaw
