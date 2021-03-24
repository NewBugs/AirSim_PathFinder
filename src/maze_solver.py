"""
This script will demonstrate how to navigate a maze using AirSim's multirotor with a distance sensor.
"""
import airsim
import sys
from time import sleep
import argparse
import math
from enum import Enum

class MazePoint(Enum):
    UNVISITED = 0
    VISITED = 1
    BLOCKED = 2


class Tremaux:
    def __init__(self):
        # reference - {"0, 0": MazePoint.UNVISITED}
        self.maze_reference_map = {"0, 0": MazePoint.VISITED}
        self._last = (0, 0)

        self.available_paths = {"forward": True, "right": False, "back": False, "left": False}

    def step(self) -> (int, int, str):
        # Return the direction to try 
        # TODO: Refactor to use dict? {"forward": ()}
        possible_paths = []
        append_path = possible_paths.append
        
        for path, allowed in self.available_paths.items():

            if allowed:
                if path == "forward":
                    append_path((self._last[0], self._last[1] + 1, "forward"))

                elif path == "right":
                    append_path((self._last[0] + 1 , self._last[1], "right"))


                elif path == "back":
                    append_path((self._last[0], self._last[1] - 1, "back"))

                elif path == "left":
                    append_path((self._last[0] - 1, self._last[1], "left"))

        if len(possible_paths) == 0:
            raise KeyError("No available paths")
        elif len(possible_paths) == 1:
            self.maze_reference_map[f"{path[0]}, {path[1]}"] = MazePoint.VISITED
            return possible_paths[0]
        
        return self.check_next_step(possible_paths)

        # TODO: MAKE SURE MOVEMENT IS SAFE
    
    def check_next_step(self, possible_paths: list) -> (int, int):
        
        # Set max value for path options
        cheapest_path_value = MazePoint.BLOCKED
        best_path = None

        for path in possible_paths:
            if f"{path[0]}, {path[1]}" in self.maze_reference_map:
                if self.maze_reference_map[f"{path[0]}, {path[1]}"] < cheapest_path_value:
                    cheapest_path_value = self.maze_reference_map[f"{path[0]}, {path[1]}"]
                    best_path = path
            else:
                cheapest_path_value = MazePoint.UNVISITED
                best_path = path
                break
        
        if best_path == None:
            raise AttributeError("Couldn't find the cheapest path")

        self.maze_reference_map[f"{best_path[0]}, {best_path[1]}"] = MazePoint.VISITED
        return best_path
        

class MazeTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        # Enable API control
        self.client.enableApiControl(True)
        self.distance_sensor_name = "Distance"
        self.vehicle = "Drone1"
        self.maze_solver = Tremaux()
        self.yaw_direction = 0

    def find_available_paths(self):

        for path in self.maze_solver.available_paths:
            # Get initial distance 
            distance_data = self.client.getDistanceSensorData(distance_sensor_name=self.distance_sensor_name, vehicle_name=self.vehicle)
            
            print(f"Distance: {distance_data.distance}")
            if distance_data.distance <= 4.0:
                self.maze_solver.available_paths[path] = False
            else:
                self.maze_solver.available_paths[path] = True

            print("ROTATE TO YAW")
            # TODO: FIX YAW (REACTIVE TORQUE) - MAINTAIN ALTITUDE
            # self.client.rotateToYawAsync(yaw=90, margin=360, vehicle_name="Drone1").join()

            # MIGHT need a sleep
            self.client.rotateByYawRateAsync(yaw_rate=85, duration=1, vehicle_name="Drone1").join()
            
            # self.client.moveByRollPitchYawZAsync(0, 0, -180 * (math.pi/360), -1, 0.1, vehicle_name="Drone1")
        
    def execute(self):

        print("arming the drone...")
        self.client.armDisarm(True)

        state = self.client.getMultirotorState()
        s = print(state)
        #print("state: %s" % s)

        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()

        # self.client.rotateToYawAsync(-90)
        # AirSim uses NED coordinates so negative axis is up, so -10 meters above the original launch point.
        # Thanks @dbaldwin for helping my understanding.

        # Fly given velocity vector for 5 seconds
        duration = 5
        stuck = False
        while not stuck:

            # TODO: Calculate for momentum

            # self.yaw_direction = (yaw_direction + 90) % 360

            # Fly straight for 5 and attempt to maintain altitude 
            if self.yaw_direction == 0:
                self.client.moveByVelocityZAsync(0, 1, -1, duration, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
            elif self.yaw_direction == 90:
                self.client.moveByVelocityZAsync(-1, 0, -1, duration, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
            elif self.yaw_direction == 180:
                self.client.moveByVelocityZAsync(0, -1, -1, duration, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
            elif self.yaw_direction == 270:
                self.client.moveByVelocityZAsync(1, 0, -1, duration, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()

            # stop momentum
            self.client.moveByVelocityZAsync(0, 0, -1, duration, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()

            self.find_available_paths()

            next_step = self.maze_solver.step()

            if next_step[2] == "forward":
                self.yaw_direction += 0
            elif next_step[2] == "right":
                self.yaw_direction += 90
            elif next_step[2] == "back":
                self.yaw_direction += 180
            elif next_step[2] == "left":
                self.yaw_direction += 270


            # self.client.moveByRollPitchYawZAsync(0, 30*(math.pi/180), 0, -1, 0.1, vehicle_name="Drone1")

            # TODO: DEBUG COLLISION
            updated_state = self.client.getMultirotorState()

            if updated_state.collision.has_collided:
                stuck = True


        self.client.landAsync().join()  

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("fly_forward.py makes drone fly and attempts to find way out of a maze.")
  
    args = arg_parser.parse_args(args)    
    maze_test = MazeTest()
    try:
        maze_test.execute()
    except Exception as e:
        print(e)
        maze_test.stop()



# # Reset heading
# time.sleep(2)

# # Box with nose pointed forward the whole time
# client.moveByVelocityZAsync(5, 0, z, duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
# time.sleep(2)

# # Box with nose pointed towards the inside of the box
# client.moveByVelocityZAsync(5, 0, z, duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 90)).join()
# time.sleep(2)

# Land
# time.sleep(2)
# client.landAsync().join()