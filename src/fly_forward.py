"""
This script will demonstrate how to navigate a maze using AirSim's multirotor with a distance sensor.
"""
import airsim
import sys
import time


# Makes the drone fly and get distance data
class MazeTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        # Enable API control
        self.client.enableApiControl(True)

    def execute(self):

        print("arming the drone...")
        self.client.armDisarm(True)

        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        #print("state: %s" % s)

        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()

        # AirSim uses NED coordinates so negative axis is up, so -10 meters above the original launch point.
        z = -10

        # Fly given velocity vector for 5 seconds
        duration = 5

        # Fly straight for 5 and attempt to maintain altitude 
        client.moveByVelocityZAsync(0, 5, -1, duration, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
        time.sleep(2)

        # TODO: Add distance sensor collection

        time.sleep(2)
        client.landAsync().join()  

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
    finally:
        maze_test.stop()

# # Reset heading
# client.rotateToYawAsync(0)
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