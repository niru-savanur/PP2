import yaml
import traci
import numpy as np


config_file = open("./config.yaml")
cfg = yaml.safe_load(config_file)


class Vehicle:
    def __init__(self, veh_id, route, v_state, veh_data=None):
        if veh_data is None:
            veh_data = {}
        self.veh_id = veh_id
        self.route = route
        self.currentRoad = None
        self.enteredSimulation = False
        self.veh_data = veh_data

        traci.vehicle.add(veh_id, route, typeID="type1")
        self.set_vehicle_state(v_state)

    def set_vehicle_state(self, v_state):
        
       
        traci.vehicle.setSpeedMode(self.veh_id, cfg["veh_mode"])
        traci.vehicle.setSpeed(self.veh_id, cfg["veh_speed_default"])

        if v_state == "default":
            traci.vehicle.setColor(self.veh_id, cfg["veh_col_green"])
            

        elif v_state == "intersection":
            traci.vehicle.setColor(self.veh_id, cfg["veh_col_orange"])
            # traci.vehicle.slowDown(self.veh_id, cfg["veh_speed_intersection"], cfg['slowdown_time'])

        elif v_state == "int_crossed":
            traci.vehicle.setColor(self.veh_id, cfg["veh_col_grey"])
            # traci.vehicle.slowDown(self.veh_id, cfg["veh_speed_default"], cfg['speedup_time'])

    # def is_outbound(self):
    #     return traci.vehicle.getRoadID(self.veh_id) == traci.vehicle.getRoute(self.veh_id)[-1]

    def get_dist_to_intersection(self, veh_id=None):
        if veh_id is None:
            veh_id = self.veh_id

        loc = np.array(traci.vehicle.getPosition(veh_id))
        intersection_loc = np.array(cfg["intersection_coords"])

        return np.linalg.norm(loc - intersection_loc)

    def get_speed(self):
        return traci.vehicle.getSpeed(self.veh_id)

    def getRoad(self):
        return traci.vehicle.getRoadID(self.veh_id)

    def updateRoad(self):
        self.currentRoad = traci.vehicle.getRoadID(self.veh_id)
        self.enteredSimulation = True
        # print(self.currentRoad)


    def roadChanged(self):
        if self.enteredSimulation and self.currentRoad != traci.vehicle.getRoadID(self.veh_id):

            return True
        return False

    def v_accelerate(self):
        traci.vehicle.setAcceleration(self.veh_id, cfg['acceleration_rate'], cfg['acceleration_duration'])

    def v_decelerate(self, deceleration_rate=cfg['default_deceleration_rate'], duration=cfg['default_deceleration_duration']):
        traci.vehicle.setAcceleration(self.veh_id, -1 * deceleration_rate, duration)
        

    # def get_veh_data(self, vehicles):
    #     self.veh_data = {}
    #     for veh in vehicles:
    #         self.veh_data[veh.veh_id] = {
    #             "route": veh.route[6:],
    #             "edge": traci.vehicle.getLaneID(veh.veh_id),
    #             "distance": float(self.get_dist_to_intersection(veh.veh_id)),
    #             "adjustment": 0
    #         }

    #     return self.veh_data

    
