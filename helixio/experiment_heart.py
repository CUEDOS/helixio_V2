from __future__ import annotations
import json
import flocking
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.telemetry import raw_gps# transfer it to onboard.py
import pymap3d as pm
from communication import DroneCommunication
from data_structures import AgentTelemetry
import math
import numpy as np
from string import digits


def index_checker(input_index, length) -> int:
    if input_index >= length:
        return int(input_index % length)
    return input_index


def only_numeric(input_string):
    output_string = "".join(c for c in input_string if c in digits)
    return output_string


class Experiment:
    def __init__(self, id, swarm_telem, experiment_file_path) -> None:
        self.ready_flag = False
        self.id = id
        self.least_distance = 2  # minimum allowed distance between two agents
        # Set up corridor variables
        self.points = [[]]
        self.current_path = 0
        self.rotation_factor = 1
        self.directions = []
        self.current_index = 0
        self.pass_permission = False
        self.target_point = np.array([0, 0, 0], dtype="float64")
        self.integ_error=np.array([0,0,0], dtype="float64")
        self.load(experiment_file_path, swarm_telem)

    def load(self, experiment_file_path, swarm_telem):

        with open(experiment_file_path, "r") as f:
            experiment_parameters = json.load(f)

        self.k_separation = experiment_parameters["k_seperation"]
        self.r_conflict = experiment_parameters["r_conflict"]
        self.r_collision = experiment_parameters["r_collision"]
        self.Kp = experiment_parameters["Kp"]
        self.Ki = experiment_parameters["Ki"]
        self.k_PI=experiment_parameters["k_PI"]

        self.points = experiment_parameters["corridor_points"]

    def get_swarm_priorities(self, swarm_telem):
        numeric_ids = {}
        # assigned_pre_start_positions = {}
        # numeric_id = int(only_numeric(self.id))
        for agent in swarm_telem.keys():
            numeric_ids[agent] = int(only_numeric(agent))

        swarm_priorities = sorted(numeric_ids, key=numeric_ids.get)
        return swarm_priorities

    def path_following(self, swarm_telem, max_speed, time_step, max_accel):
        
        dis_vector=self.points-np.array(swarm_telem[self.id].position_ned, dtype="float64")
        limit_integral=1
        self.integ_error=self.integ_error+(dis_vector*time_step)
        for i in range(len(self.integ_error)):
            if self.integ_error[i]>limit_integral:
                self.integ_error[i]=self.integ_error[i]*limit_integral/abs(self.integ_error[i])

        #Calculating PI velocity ----------------------------------------
        limit_v_PI = 1
        v_PI=self.Kp*dis_vector+self.Ki*self.integ_error
        if np.linalg.norm(v_PI) > limit_v_PI:
            v_PI = v_PI * limit_v_PI / np.linalg.norm(v_PI)

        # Calculating v_separation (normalized) -----------------------------
        limit_v_separation = 5
        r_conflict = 5
        r_collision = 2.5
        v_separation = np.array([0, 0, 0], dtype="float64")
        for key in swarm_telem:
            if key == self.id:
                continue
            p = np.array(swarm_telem[key].position_ned, dtype="float64")
            x = np.array(swarm_telem[self.id].position_ned, dtype="float64") - p
            d = np.linalg.norm(x)
            if self.least_distance > d:
                self.least_distance = d
            if d <= r_conflict and d > r_collision and d != 0:
                v_separation = v_separation + (
                    (x / d) * (r_conflict - d / r_conflict - r_collision)
                )
            if d <= r_collision and d != 0:
                v_separation = v_separation + 1 * (x / d)
            if np.linalg.norm(v_separation) > limit_v_separation:
                v_separation = (
                    v_separation * limit_v_separation / np.linalg.norm(v_separation)
                )
        desired_vel = (self.k_PI*v_PI + self.k_separation * v_separation)
        v_separation = np.array([0, 0, 0])

        output_vel = flocking.check_velocity(
            desired_vel, swarm_telem[self.id], max_speed, 0, time_step, max_accel
        )
        return output_vel
