import json
import flocking
import math
import numpy as np
from string import digits


def only_numeric(input_string):
    output_string = "".join(c for c in input_string if c in digits)
    return output_string

class Experiment:
    def __init__(self, id, swarm_telem, experiment_file_path, home_lat, home_lon, home_alt, ref_lat, ref_lon, ref_alt) -> None:
        self.id = id # the id of the drone in its swarm
        self.flight_mode="position"
        self.prestart_position=[0, 0, 0]
        self.reference_point=[ref_lat, ref_lon, ref_alt]
        self.home_position=[home_lat, home_lon, home_alt]
        self.load(experiment_file_path) # to load data from json file 

    def load(self, experiment_file_path): # load date from 
        with open(experiment_file_path, "r") as f:
            experiment_parameters = json.load(f)
        self.travel_time = experiment_parameters["travel_time"] # magnitude of speed
        self.pre_start_positions = experiment_parameters["pre_start_positions"] # loading the prestart positions
        self.ready_flag = True # to show everything is loaded

    def get_pre_start_positions(self, swarm_telem, swarm_priorities):

        assigned_pre_start_positions = {}

        for i, agent in enumerate(swarm_priorities):
            if len(self.pre_start_positions) > i:
                assigned_pre_start_positions[agent] = self.pre_start_positions[i]
            else:
                # if there isnt enough pre start positions, start from current position
                assigned_pre_start_positions[agent] = swarm_telem[self.id].position_ned
        self.prestart_position=assigned_pre_start_positions[self.id]
        return assigned_pre_start_positions

    def get_swarm_priorities(self, swarm_telem):
        numeric_ids = {}
        for agent in swarm_telem.keys():
            numeric_ids[agent] = int(only_numeric(agent))

        swarm_priorities = sorted(numeric_ids, key=numeric_ids.get)
        return swarm_priorities

    def path_following(self, swarm_telem, max_speed, time_step, max_accel, mission_start_time): # method to run during the experiment
        mission_time=swarm_telem[self.id].current_time-mission_start_time # swarm_telem[self.id].current_time is the current time which is synchronous (from GPS)
        # calculating the target position
        if (mission_time<=self.travel_time/2.00):
            target_position=[10 + self.prestart_position[0], 0 + self.prestart_position[1], -20]
        elif (mission_time<=self.travel_time):
            target_position=[0 + self.prestart_position[0], 0+ self.prestart_position[1], -20]
        else:
            target_position=[0 + self.prestart_position[0], 0 + self.prestart_position[1], -20]
        output_pos = flocking.check_position(target_position, swarm_telem[self.id], max_speed, 0, time_step, self.reference_point, self.home_position) # getting the target velocity in its right format
        print(self.id, "current_position=", swarm_telem[self.id].position_ned)
        print(self.id, "target_position=", target_position)
        return output_pos # sending the target velocity