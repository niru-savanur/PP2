import os
import sys
import yaml
import traci
from traci.exceptions import TraCIException
from random import randint
import random

from vehicle import Vehicle

from sumolib import checkBinary
sumoBinary = checkBinary('sumo-gui')

config_file = open("./config.yaml")
cfg = yaml.safe_load(config_file)

sumoCmd = [
    sumoBinary,
    '-c', cfg["sumo_cfg"],
    '--tripinfo-output', cfg["trip_info_out"],
    '--collision-output', cfg["collisions_out"],
    '--statistic-output', cfg["statistics_out"],
    '--step-length', cfg['step_length']
]

def add_vehicles(all_vehicles):
    for i in range(3):
        for j in 0, 3, 6, 9:
        # for j in 0, 3:
            # probability = randint(0, 1)
            probability = 1
            if probability == 1:
                veh_id = "veh_{}".format(len(all_vehicles))
                route = "route_{}".format( i+j )

                veh = Vehicle(
                veh_id=veh_id,
                route=route,
                v_state="default"
                )

                all_vehicles.append(veh)

    return all_vehicles



def add_vehs_to_control_zone(all_vehicles, control_vehicles, priority_queue):
    try:
        for veh in all_vehicles:
            if veh.get_dist_to_intersection() < cfg['control_radius'] and veh not in control_vehicles:
                veh.set_vehicle_state("intersection")
                control_vehicles.append(veh)
                priority_queue.append(veh)
                print(veh.veh_id, " Entered intersection control zone")

    except TraCIException:
        all_vehicles.remove(veh)
        if veh in control_vehicles:
            control_vehicles.remove(veh)
            priority_queue.remove(veh)

    return all_vehicles, control_vehicles, priority_queue



def remove_vehs_from_control_zone(all_vehicles, control_vehicles, priority_queue):
    try:
        for veh in control_vehicles:
            if veh.roadChanged():
                veh.set_vehicle_state("int_crossed")
                print(veh.veh_id, " Left intersection control zone")
                control_vehicles.remove(veh)
                priority_queue.remove(veh)
                all_vehicles.remove(veh)
            else:
                veh.updateRoad()

    except TraCIException:
        control_vehicles.remove(veh) 
        priority_queue.remove(veh)       


    return all_vehicles, control_vehicles, priority_queue


def is_conflict(veh1, veh2):
    junction_area_taken = cfg['junction_area_taken']
    if set(junction_area_taken[veh1.veh_id]) & set(junction_area_taken[veh2.veh_id]):
        return True
    return False



def execute_adjustments(priority_queue):

    expected_arrival_times = [priority_queue[0].get_dist_to_intersection() / priority_queue[0].get_speed()]
    priority_queue[0].v_accelerate()
    for i in range(1, len(priority_queue)):
        expected_arrival_times.append(priority_queue[i].get_dist_to_intersection() / priority_queue[i].get_speed())
        same_road_veh = None
        conflict_veh = None
        for vehicle in priority_queue[0: i-1]:
            if vehicle.getRoad() == priority_queue[i].getRoad():
                same_road_veh = priority_queue.index(vehicle)
            
            elif is_conflict(priority_queue[i], vehicle):
                conflict_veh = priority_queue.index(vehicle)

        if not same_road_veh and conflict_veh:
            previous_veh = expected_arrival_times[conflict_veh]
        elif not conflict_veh and same_road_veh:
            previous_veh = expected_arrival_times[same_road_veh]
        elif conflict_veh and same_road_veh:
            previous_veh = min(expected_arrival_times[same_road_veh], expected_arrival_times[conflict_veh])
        else:
            priority_queue[i].v_accelerate()
            return

        if expected_arrival_times[i] > expected_arrival_times[previous_veh] + 1:
            priority_queue[i].v_accelerate()

        elif expected_arrival_times[i] < expected_arrival_times[previous_veh] + 1:
            priority_queue[i].v_decelerate()



def log_veh_data(all_vehicles):
    try:
        for veh in all_vehicles:
            print("id: ", veh.veh_id, " dist: ", veh.get_dist_to_intersection())
            
    except TraCIException:  
        all_vehicles.remove(veh)



def run():
    all_vehicles = []
    control_vehicles = []
    traci.start(sumoCmd)
    step = 0
    total_vehicles_added = 0

    priority_queue = []
    
    # traci.vehicle.setImperfection(typeID="type1", imperfection=cfg["sigma"])
    # traci.vehicle.setMinGap(typeID="type1", minGap=cfg["safe_distance"])

    while total_vehicles_added < cfg["num_vehicles"] or traci.simulation.getMinExpectedNumber() > 0:
        if step % 5 == 0 and total_vehicles_added < cfg["num_vehicles"]:
            all_vehicles = add_vehicles(all_vehicles)
            total_vehicles_added = len(all_vehicles)
            
        
        traci.simulationStep()
        # log_veh_data(all_vehicles)

        all_vehicles, control_vehicles, priority_queue = remove_vehs_from_control_zone(all_vehicles, control_vehicles, priority_queue)

        all_vehicles, control_vehicles, priority_queue = add_vehs_to_control_zone(all_vehicles, control_vehicles, priority_queue)

        if step % 5 == 0 and len(priority_queue):
            execute_adjustments(priority_queue) 

        step += 1
        

    print("Total simulation steps: ", step)
    print("Num vehicles added: ", total_vehicles_added)
    print("Done!!")





if __name__ == "__main__":
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("You must declare environment variable SUMO_HOME")

    run()