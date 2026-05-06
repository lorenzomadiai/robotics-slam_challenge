"""
"""

from __future__ import print_function
from src.env    import VrepEnvironment
from src.agents import Pioneer
from src.disp   import Display
import settings
import time, argparse
import matplotlib.pyplot as plt
import random
import numpy as np
""" Motors:

          1. agent.change_velocity([ speed_left: float, speed_right: float ]) 
               Set the target angular velocities of left
               and right motors with a LIST of values:
               e.g. [1., 1.] in radians/s.
               
               Values in range [-5:5] (above these 
               values the control accuracy decreases)
                    
          2. agent.current_speed_API() 
          ----
               Returns a LIST of current angular velocities
               of the motors
               [speed_left: float, speed_right: float] in radians/s.

    Lidar:
          3. agent.read_lidars()   
          ----
               Returns a list of floating point numbers that 
               indicate the distance towards the closest object at a particular angle from the robot's lidar position.
               
               Basic configuration of the lidar:
               Lidar distance reading list: 270 lidar distance readings starting from 134 deg  to 1 deg 
               followed by a dummy-value and further readings from -1 deg to -135 deg. 
               That is: A total of 270 real values representing distance readings starting with the right-most lidar point/reading and going anti-clockwise
               Note: ignore center-element, number 135 (is index 134 as we start counting from 0), of the list, it has always value 1.0

    Agent:
          You can access these attributes to get information about the agent's positions

          4. agent.pos  

          ----
               Current x,y position of the agent (derived from 
               SLAM data) Note: unreliable as SLAM is not solved here.

          5. agent.position_history

               A deque containing N last positions of the agent 
               (200 by default, can be changed in settings.py) Note: unreliable as SLAM is not solved here.
"""

###########
###########

def loop(agent):

    """
    Robot control loop 
    Your code goes here
    """
    # Calculate average Lidar readings in segments
    sum = 0
    scan_average = []
    array = agent.read_lidars()
    for i, data in enumerate(array):
        sum += data
        # Calculate average Lidar reading for every 6 readings
        if i == 269:
            average = sum / 6
            scan_average.append(average)
            break
        if i % 6 == 0:
            average = sum / 6
            scan_average.append(average)
            print("direction: {} degrees, average: {}".format(i, str(average)))
            sum = 0

    # Trying to find a feature in the Lidar Data to detect the door in front of the robot,
    # basically if the robot has two objects near and a high distance in front probably it is
    # in the presence of a door
    # If the algorithm finds this pattern in the data, the robot moves forward to pass the door
    closest = agent.find_closest()
    if closest[1][0] < 2 and closest[1][1] < 2:
        if 25 < abs(closest[0][0]) < 75 and 25 < abs(closest[0][1]) < 75 and scan_average[21] > 4 and scan_average[22] > 4:
            agent.change_velocity([3.5,3.5])
            time.sleep(1)

    # If the robot finds a very high distance in front of it, it moves towards that direction
    for i in range(10, len(scan_average) - 10):
        if scan_average[i] > 9: 
            direction = i * 6
            print("direction is {}".format(direction))
            move(direction)

    
    # The algorithm tries to find a pattern in the Lidar data to detect a door jamb 
    count = 0
    direction = 0
    found = False
    # The algorithm checks only the central part of the Lidar data to not comeback in a door already passed
    for i in range(10, len(scan_average) - 10): 
        distance = scan_average[i]
        # Detect an obstacle near
        if distance < 1.4:
            count += 1
        # If consecutive readings indicate an obstacle-free path
        # If the direction is found the robot slightly moves back to facilitate the changes of direction
        elif count > 2:
            # A distance higher than 3.5 could be a door open
            if distance > 3.5:
                direction = (i + 1) * 6  # Calculate the possible direction of the free path
                print("direction is {}".format(direction ))
                found = True
                # agent.change_velocity([-1,-1])
                # time.sleep(2)
                break
            if scan_average[i + 1] > 3.5:
                direction = (i + 2) * 6 
                print("direction is {}".format(direction ))
                found = True
                # agent.change_velocity([-1,-1])
                # time.sleep(2)
                break
            if scan_average[i + 2] > 3.5:
                direction = (i + 3) * 6  
                print("direction is {}".format(direction ))
                found = True
                # agent.change_velocity([-1,-1])
                # time.sleep(2)
                break
            # If we dont't find a open path probably it wasn't a door jamb but only an obstacle, e.g. a wardrobe,
            # so starting to search again
            count = 0
    
    # Case where the door jamb is not found, we try to find another pattern in the data to detect a door
    # Especially when the robot is far from a door this detection can be more useful than the previous one
    if found == False:

       # The algorithm checks only the central part of the Lidar data to not comeback in a door already passed 
        i = 4
        for distance in scan_average[4:len(scan_average) - 4]:

            # Trying to find a open path between two obstacles nearer (like a door jamb - open path - a door jamb) 
            # or increasing distances (like a door jamb - open path - open path)
            if distance - scan_average[i - 1] > 2:
                if abs(distance - scan_average[i + 1]) > 2:
                    direction = (i + 1) * 6
                    print("direction2 is {}".format(direction ))
                    found = True
                    break
                if abs(distance - scan_average[i + 2]) > 2:
                    direction = (i + 2) * 6
                    print("direction2 is {}".format(direction ))
                    found = True
                    break
                if abs(distance - scan_average[i + 3]) > 2:
                    direction = (i + 3) * 6
                    print("direction2 is {}".format(direction ))
                    found = True
                    break
            i += 1
    
                

    # If it has been found a direction, the robot moves towards that direction
    if found == True:
        move(direction)

    # Otherwise, it continues moving towards the previous direction found
    else:
        print("the robot continues moving towards the previous direction...")
        time.sleep(1.5)
    
    # Check if the robot is stuck and perform recovery action
    current_velocity = agent.current_speed_API()
    print(agent.current_speed_API())
    if abs(current_velocity[0]) < 0.09 and abs(current_velocity[1]) < 0.09:
        print("robot is stuck moving forward")
        agent.change_velocity([-2.2,-2])
        time.sleep(2.5)

        # The robot moves also forward to avoid being stuck by a rear obstacle
        agent.change_velocity([0.5,0.5])
        time.sleep(1)
        return
    
##########
##########
def move(direction):
    """
    Move the robot based on the detected direction.
    """

    # Define velocity mappings for each 10-degree range
    velocity_mapping = {
    (0, 10): [4, 1],       # Velocity for 0-10 degrees
    (10, 20): [3.8, 1.1],      # Velocity for 10-20 degrees
    (20, 30): [3.6, 1.2],    # Velocity for 20-30 degrees
    (30, 40): [3.4, 1.3],    # Velocity for 30-40 degrees
    (40, 50): [3.2, 1.4],    # Velocity for 40-50 degrees
    (50, 60): [3.0, 1.5],    # Velocity for 50-60 degrees
    (60, 70): [2.8, 1.5],    # Velocity for 60-70 degrees
    (70, 80): [2.6, 1.6],    # Velocity for 70-80 degrees
    (80, 90): [2.4, 1.7],   # Velocity for 80-90 degrees
    (90, 100): [2.2, 1.8],  # Velocity for 90-100 degrees
    (100, 110): [2.1, 1.9],    # Velocity for 100-110 degrees
    (110, 120): [2.05, 1.95],    # Velocity for 110-120 degrees
    (120, 130): [3, 3],  # Velocity for 120-130 degrees
    (130, 140): [1.95, 2.05],  # Velocity for 130-140 degrees
    (140, 150): [1.9, 2.1],  # Velocity for 140-150 degrees
    (150, 160): [1.8, 2.2],  # Velocity for 150-160 degrees
    (160, 170): [1.7, 2.4],    # Velocity for 160-170 degrees
    (170, 180): [1.6, 2.6],    # Velocity for 170-180 degrees
    (180, 190): [1.5, 2.8],  # Velocity for 180-190 degrees
    (190, 200): [1.5, 3.0],  # Velocity for 190-200 degrees
    (200, 210): [1.4, 3.2],  # Velocity for 200-210 degrees
    (210, 220): [1.4, 3.4],  # Velocity for 210-220 degrees
    (220, 230): [1.3, 3.4],  # Velocity for 220-230 degrees
    (230, 240): [1.2, 3.6],  # Velocity for 230-240 degrees
    (240, 250): [1.1, 3.8],    # Velocity for 240-250 degrees
    (250, 260): [1, 4],    # Velocity for 250-260 degrees
}


    # Find the appropriate velocity based on the direction
    for angle_range, velocity in velocity_mapping.items():
        if angle_range[0] <= direction <= angle_range[1]:
            agent.change_velocity(velocity)
            print("Moving forward with velocity: {}".format(velocity))
            time.sleep(2.5)  # Pause for 5 seconds (adjust as needed)
            return  # Exit the loop after setting velocity

    # If direction doesn't fall within any range, print an error message
    print("Direction {} is not within a valid range.".format(direction))


#######
#######
if __name__ == "__main__":
    plt.ion()
    # Initialize and start the environment
    environment = VrepEnvironment(settings.SCENES + '/room_static.ttt')  # Open the file containing our scene (robot and its environment)
    environment.connect()        # Connect python to the simulator's remote API
    agent   = Pioneer(environment)
    display = Display(agent, False)

    print('\nDemonstration of Simultaneous Localization and Mapping using CoppeliaSim robot simulation software. \nPress "CTRL+C" to exit.\n')
    start = time.time()
    step  = 0
    done  = False
    environment.start_simulation()
    time.sleep(1)

    try:
        while step < settings.simulation_steps and not done:
            display.update()                      # Update the SLAM display
            loop(agent)                           # Control loop
            step += 1
    except KeyboardInterrupt:
        print('\n\nInterrupted! Time: {}s'.format(time.time()-start))

    display.close()
    environment.stop_simulation()
    environment.disconnect()
