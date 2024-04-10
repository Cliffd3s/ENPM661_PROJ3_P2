>> HOW TO RUN THE CODE
		- Note that both parts of the project will run for the same script file from the ros package: proj3_phase2_draft10.py
		- ensure the libraries from the dependencies are install on your pc, and ensure the ros package is setup
		- in the terminal, from your workspace run start the script using: ros2 run turtlebot3_project3 proj3_phase2_draft10.py
		
		- Part 1 will run first. The code will ask you to input the clearnance in mm, then the starting point x and y coordinates and theta angle then the goal-x and goal-y, one by one. Please enter the starting theta angle in degrees and the start, goal and clearance in mm. Please read the prompts carefully before typing in your points. For the first part, please choose your points w.r.t to the project3 pdf map origin frame. You will then be prompted to enter the left and right rpm one at a time. do not enter any units in the inputs.   
		  If the chosen point is out of the limits of the display or within the obstacle space, the code will notify you and another prompt will come up asking you to re-enter the x and y points and theta. Similarly if the clearance if greater than 29mm or the rpm are greater than 75, you will be asked to re-enter them. The code will also ask you to re-enter the goal x and y coordinates and theta, if they are the same as the starting point that you entered initially. The 75 rpm comes from the max linear velocity limit 0.26 m/s listed for the robot's spec. 
		 - Example inputs: clearance = 29, start_x = 500, start_y= 1000, goal_x = 5750, goal_y = 1000, rpm_l = 22, rpm_r = 22. 
		  
		- After properly entering your points, the code will run the A* algorithm first, then once the goal node is found, the pygame display and animation will appear. The goal point will first be highlighted in white, then all the lines or curves from the node exploration will appear in green. The optimal path will be drawn first with black dots, then a white line will run through it. 
		  There may be a 30 to 60 second delay betwen the time the desired start & goal points are entered and the time the animation comes up, depending on the chosen points. Also note that because there are so many explored points the green action lines during the node exploration may not appear clearly. 
		  
		- The execution time for the algorithm search, for the animation, and the total combined time for both, will each be printed out in the terminal.
		- For some reason, sometimes two blue lines (which are for the points outside the obstacle space) may appear in the yellow bloated bottom space, this seems to be a pygame issue which I did not have time to figure out. 
		
		- Once your done with part 1, in another terminal, from your project workspace, use the following command to open the gazebo simulation. 
			ros2 launch turtlebot3_project3 competition_world.launch.py
		- Close the pygame window. An input prompt will appear in the other terminal asking you to enter your goal-x and goal-y coordinates in mm. Please enter those with respect to the frame where the robot spawns in the gazebo world. for example, enter goal_x=5250 , goal_y= 0, to reach the point that would be (5750, 1000) in the coordinate at the bottom left corner of the layout in the pdf. 
		- the A* star search algorithm will run again with the starting point fixed at where the robot spawned, theta=0, clearance = 29, and rpm-left and right=22. 
		- Once the optimal path is found, a message will print in the terminal saying the ROS publisher has started.(note that for this case, the goal_x, goal_y printed out with the success message will be with respect the coordinate at the bottom left corner of the layout pdf, but this is only to keep it consistant with the part 1 run. The A* converst the coordinate for both part 1 and part 2 into the pygames coordinate before running starting the search). 
		- Watch the robot move in the gazebo world. Unfortunately, we did not have time fine tune the robot's motion so it usually ends up hitting part of an obstacle. 

>> LIBRARIES/DEPENDENCIES
		- pygame (pygame 2.5.0 (SDL 2.28.0, Python 3.8.10))
		- sys, 
		- math, 
		- time
		- heapq as hq
		- from collections import deque
		- from pygame.locals import *
		import rclpy
		from rclpy.node import Node
		from nav_msgs.msg import Odometry
		from geometry_msgs.msg import Twist
		from rclpy.qos import QoSProfile
		from time import sleep
>>> GITHUB LINK
	- https://github.com/Cliffd3s/ENPM661_PROJ3_P2.git

>>> REFERENCES
	- Website used to get the turtlebot3 waffle pi's specs: 
		https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
	



