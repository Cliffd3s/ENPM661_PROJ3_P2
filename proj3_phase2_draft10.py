#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from time import sleep
import pygame, sys, math, time
import heapq as hq
from collections import deque
from pygame.locals import *
pygame.init()

robot_r = (220)/10 # 220 mm robot radius converted to cm
wheel_r = (66/2)/10 # wheel diameter = 66 mm converted to cm
wheel_dist = (287)/10 # 287 mm converted to cm, distance from a line through the middle of one wheel to another line through the middle of the other wheel

WINDOW_WIDTH, WINDOW_HEIGHT = 6000/10, 2000/10
# Initialize Pygame window
# WINDOW_WIDTH, WINDOW_HEIGHT = 6000, 2000

open_list = []  # to be used for heapq for nodes in the open-list
visited_matrix_idx = set() #set used to store the index of each visited node if they were added to a visted-node-matrix (see proj3 part 1 pdf page 14). replaces the closed-list from proj2
animation_v_matrix_idx = set() # set same as visited_matrix_idx but used for the animation part only. 
prnt_node_map = {} # dictionary (key=node, val=parent) for all the nodes visited mapped to their parents to be used for backtracking
lowest_c2c_map ={} # dictionary (key=node, val=c2c) to keep track of the nodes and their cost, used to ensure only lowest cost in open-list
step_info_map = {} # dictionary (key = node, val= (linear_velocity, angular_velocity, distance_covered) to keep track of the nodes, traveled distance, linear & angular velocities
# node_action_map = {} # dictionary (key= node, value= (rpm_l,rpm_r)) to keep track of action used be each node. Useful for drawing final path curve
path = deque()     # deque used for storing nodes after ordering them using backtracking
step_info = deque() # deque for storing linear_velocity, angular_velocity, distance_covered of each node for the optimal path
# node_action_rpms = deque() # deque for storing the (RPM_L, RPM_R) actions performed to get to each node in an ordered way. Used for drawing final path curve. 
thr_x, thr_y, thr_theta = 0.5, 0.5, 30  # x, y, theta thresholds to avoid visiting too many close points (see page 14 in project 3 part1 description pdf)        

def nrm_angle(angle):
    """ normalizes all angles to be with respect to 360 degrees"""
    return angle % 360

clrnc_trigger = 1
while clrnc_trigger == 1:
    try:
        clrnc = float(input("Enter a value in mm between 0 and 29 (including 0 & 29) for the wall & obstacle clearance: "))/10
    except:
        print('you did not enter a number, please enter only numbers')
        continue
    if clrnc <0 or clrnc >29:
        print('The value you entered is outside the acceptable range, try again')
        continue
    else:
        clrnc_trigger =0

bloat = robot_r + clrnc
# OBSTACLE 1 RECTANGLE 1 for the left side of the layout
rect1_l, rect1_w = 1000/10, 250/10
b_rect1_l = rect1_l + bloat  #bloated rectangle1 length
b_rect1_w = rect1_w + 2*bloat  # bloated rectangle1 width
b_rect1_x = (500+1000)/10-bloat     #x-coordinate of rect corner point
# OBSTACLE 2: RECTANGLE 2
rect2_l, rect2_w = 1000/10, 250/10
b_rect2_l = rect2_l + bloat  # bloated rectangle 2 length
b_rect2_w = rect2_w + 2*bloat # bloated rectangle 2 width
b_rect2_x = (500 + 2000)/10 - bloat #x-coordinate of rect corner point
b_rect2_y = WINDOW_HEIGHT - b_rect2_l #y-coordinate of rect corner point
# OBSTACLE3: CIRLCE
center_x, center_y = (500+3700)/10, 800/10
circle_r, bltd_circle_r = (1200/2)/10, bloat+(1200/2)/10

# general equation of circle: (x-center_x)^2 + (y-center_y)^2 = radius^2

def nrm_angle(angle):
    """ normalizes all angles to be with respect to 360 degrees"""
    return angle % 360

def IsInObstacle(x, y):
    """ Checks if a point is in the obstacle space. Returns True if the point is in the obstacle space, and False if it is not"""
    # First 2 rectangular obstacles
    if ((b_rect1_x <= x <=(b_rect1_x+b_rect1_w)) and y <= b_rect1_l) or ((b_rect2_x <=x <=(b_rect2_x + b_rect2_w)) and y >= b_rect2_y):
        return True
    # Circle Obstacle
    # general equation of circle: (x-center_x)^2 + (y-center_y)^2 = radius^2
    elif (x-center_x)**2+(y-center_y)**2<= bltd_circle_r**2:
        return True
    # Border region outside all obstacles 
    elif (x <=bloat or x >= (WINDOW_WIDTH-bloat)) or (y <=bloat or y >= (WINDOW_HEIGHT-bloat)):
        return True
    else:               # not in obstacle space
        return False
    
def A_star(a_star_weight, strt_and_rpm_input_sw):
    global start_x, start_y
    global start_theta
    global goal_x, goal_y
    global RPM_L, RPM_R
    global clrnc
    global obstacle_pts
    global actions

    start_pt_trigger = 1
    goal_pt_trigger =1
    wheel_rpm_trigger =1
    if strt_and_rpm_input_sw ==1:
        while start_pt_trigger ==1:
            try:
                start_x, startb_y, startb_theta = float(input('Enter starting point x-cordinate in mm: '))/10, float(input('Enter starting point y-cordinate in mm: ')), float(input('Enter starting theta in degrees: '))
                # worst case start wrt to coord at bottom-left corner: start (6,6), goal (1194, 162) or goal (1194, 338)
            except:
                print('you did not enter a number, please enter only numbers')
                continue
            start_y = WINDOW_HEIGHT - startb_y/10 # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate
            start_theta = nrm_angle(startb_theta)
            if IsInObstacle(start_x, start_y):
                print('The chosen start point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
            else:
                start_pt_trigger = 0
    while goal_pt_trigger == 1:
        try:
            goal_x, goalb_y = float(input('Enter goal point x- cordinate in mm: '))/10, float(input('Enter goal point y-cordinate in mm: '))
        except:
            print('you did not enter a number, please enter only numbers')
            continue
        goal_y = WINDOW_HEIGHT - goalb_y/10  # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate
        if IsInObstacle(goal_x, goal_y):
            print('The chosen goal point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
            continue
        if (start_x,start_y) == (goal_x, goal_y):
            print('you chose the same starting and goal points, choose different starting and goal points')
        else:
            goal_pt_trigger = 0
    if strt_and_rpm_input_sw ==1:
        while wheel_rpm_trigger == 1:
            try:
                RPM_L, RPM_R = float(input('Enter an rpm less than 76 for the left-wheel: ')), float(input('Enter an rpm less than 76 for the right-wheel: '))
            except:
                print('you did not enter a number, please enter only numbers')
                continue
            if (RPM_L <0 or RPM_L >75) or (RPM_R <0 or RPM_R >75):
                print('The value you entered is outside the acceptable range (0 to 75), try again')
                continue
            wheel_rpm_trigger = 0

    # start_x, start_y, start_theta = 500/10, WINDOW_HEIGHT/2, 0
    # goal_x, goal_y = WINDOW_WIDTH-(250/10), WINDOW_HEIGHT-1000/10
    # goal_x, goal_y = 2900/10, WINDOW_HEIGHT-50
    a_star_strt_time = time.time()  # used to calculate how long the code runs.

    def euclidean_dist(strt_x, strt_y, end_x, end_y):
        """ calculates the euclidean distance between two points.Used for c2g calculation """
        return round(math.sqrt((end_y - strt_y)**2 + (end_x - strt_x)**2),2)

    def range_float(start, stop, step):
        """Function to generate a range with float steps, since in-built range() function only allows interger steps"""
        while start <= stop:
            yield start
            start += step

    # Use nested for loops to sweep accross all points in the layout, with a 0.5 interval, then add the points that are in the obstacle space in a set.
    obstacle_pts = {(x,y) for x in range_float(0, WINDOW_WIDTH, 0.5) for y in range_float(0, WINDOW_HEIGHT, 0.5) if IsInObstacle(x,y)}
    print(f'number of obstacle points {len(obstacle_pts)} \n')   # only for debugging

    def rnd2closest_pt5(number):
        """ rounds number to the nearest 0.5 decimal. If the number is close to 0.5, it rounds it to that, otherwise it rounds it to the closest whole number"""
        return round(number*2)/2

    def Move(Xi,Yi,Thetai,c2c,UL,UR):
        # robot_r = 220 # 220 mm 
        # wheel_r = 66/2 # wheel diameter = 66
        # wheel_dist = 287 # 287 mm, distance from a line through the middle of one wheel to another line through the middle of the other wheel
        t = 0
        dt = 0.1
        Xn=Xi
        Yn=Yi
        Thetan = math.radians(Thetai)
        UL = UL*2*math.pi/60 # convert rpm to rad/s
        UR = UR*2*math.pi/60 # convert rpm to rad/s

        x_dot = 0.5*wheel_r * (UL + UR) * math.cos(Thetan) 
        y_dot = 0.5*wheel_r * (UL + UR) * math.sin(Thetan)
        theta_dot = (wheel_r/wheel_dist) * (UR - UL)
        vel = math.sqrt(x_dot**2 + y_dot**2)
        D=0
        while t<1:
            t = t + dt
            # Xs = Xn
            # Ys = Yn
            # Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            # Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Xn += 0.5*wheel_r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5*wheel_r * (UL + UR) * math.sin(Thetan) * dt
            # Thetan += (r / L) * (UR - UL) * dt
            Thetan += (wheel_r/wheel_dist) * (UR - UL) * dt
            # plt.plot([Xs, Xn], [Ys, Yn], color="blue")
            # D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
            D+= math.sqrt((0.5*wheel_r * (UL + UR) * math.cos(Thetan) * dt)**2 + (0.5*wheel_r * (UL + UR) * math.sin(Thetan) * dt)**2)
        new_c2c = c2c+D  
        # Thetan = 180 * (Thetan) / 3.14
        Thetan = math.degrees(Thetan)
        return round(Xn), round(Yn), round(nrm_angle(Thetan)), round(new_c2c,1), round(D,2), round(vel,2), round(theta_dot,2)
    
    # Starting cost-to-come and parent node
    cost2c_start, parent_node = 0.0, None                                                                                  
    cost_2_go_start = euclidean_dist(start_x, start_y, goal_x, goal_y)
    vel_start, theta_dot_start, dist_start  = 0, 0, 0
    # creating tuple with cost to come and coordinate values (x,y, theta) 
    total_cost_start = cost2c_start + a_star_weight*cost_2_go_start
    n1 = (total_cost_start, cost2c_start,(start_x, start_y, start_theta))  

    #Push elements to heap queue which also simultaneously heapifies the queue
    hq.heappush(open_list, n1)
    # Update lowest cost and parent node maps with starting point info
    lowest_c2c_map[(start_x, start_y, start_theta)] = cost2c_start
    prnt_node_map[(start_x, start_y, start_theta)] = parent_node
    step_info_map[(start_x, start_y, start_theta)] = (vel_start, theta_dot_start, dist_start)
    # node_action_map[(start_x, start_y, start_theta)] =(0,0)

    # a_star while loop to generate new nodes
    while len(open_list) > 0:
        active_node = hq.heappop(open_list)  # remove one node from heapq
        _, curnt_c2c, curnt_x, curnt_y, curnt_theta = active_node[0], active_node[1], active_node[2][0], active_node[2][1], active_node[2][2]
        # Look to see if the node from the open_list has already been found (i.e is in the lowest_c2c_map), if it's cost-to-come is > then the one previous one, then ignore it. 
        if lowest_c2c_map.get((curnt_x, curnt_y, curnt_theta)) is not None and curnt_c2c > lowest_c2c_map.get((curnt_x, curnt_y, curnt_theta)):
            continue
        # Add rounds the x, y, theta to the nearest 0.5, then determine what its index would be if a matrix was used per page 14 of the project pdf, add it to the visited node set. 
        visited_matrix_idx.add((rnd2closest_pt5(curnt_x)/thr_x, rnd2closest_pt5(curnt_y)/thr_y, curnt_theta/thr_theta))
        # Check if we've reached the goal,if yes, backtrack to find path. If node is within 1.5 units of goal, then it's close enough. 
        if ((curnt_x - goal_x)**2 + (curnt_y - goal_y)**2) <= 1.5**2: #and (goal_theta-15 <= curnt_theta <= goal_theta+15):
            print(f"Goal point {(goal_x*10, (WINDOW_HEIGHT-goal_y)*10)} reached!")
            # Backtracking
            curnt_node = (curnt_x, curnt_y, curnt_theta)
            # Keep backtracking until start node reached
            while curnt_node is not None:
                path.appendleft(curnt_node)  # Add the current node to the pathits
                step_info.appendleft(step_info_map[curnt_node]) # Add the node's distance traveled per step, linear-velocity & theta-dot (or omega)
                # node_action_rpms.appendleft(node_action_map[curnt_node])
                curnt_node = prnt_node_map.get(curnt_node)  # Move to the parent node to now search for its parent
            print('Path found!')
            print(f'size of path deque: {len(path)}')  # only for trouble-shotting
            print(f'size of step_info que: {len(step_info)}')  # only for trouble-shotting
            print(f'size of parent_node dict: {len(prnt_node_map)} \n') # only for trouble-shotting
            print(f'size of step_info_map dict: {len(step_info_map)} \n') # only for trouble-shotting
            break
        actions = ((0,RPM_L), (RPM_L,0), (RPM_L,RPM_L), (0,RPM_R), (RPM_R,0), (RPM_R,RPM_R), (RPM_L,RPM_R), (RPM_R,RPM_L))
        # Explore neighbors with 5 possible actions
        for act in actions:
        # for nx, ny, n_theta, n_c2c in (move_0_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c), move_30_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c), move_60_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c), move_neg30_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c), move_neg60_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c)):
            nx, ny, n_theta, n_c2c, n_dist, n_vel, n_theta_dot = Move(curnt_x, curnt_y, curnt_theta, curnt_c2c, act[0], act[1])
            # Ignore node if it's in the obstacle space.
            if (nx,ny) in obstacle_pts:
                continue
            # Checks that the new node has not already been visited
            if (rnd2closest_pt5(nx)/thr_x, rnd2closest_pt5(ny)/thr_y, n_theta/thr_theta) not in visited_matrix_idx:
                # only add node to open list and other dictionaries, if the new node is not in the lowest_cost_dictionary or if the new node's c2c is lower than the one in the lowest_cost dictionary. 
                if (nx, ny, n_theta) not in lowest_c2c_map or n_c2c < lowest_c2c_map.get((nx, ny, n_theta)):
                    n_c2g = euclidean_dist(nx, ny, goal_x, goal_y)
                    n_tc = n_c2c + a_star_weight * n_c2g
                    lowest_c2c_map[(nx, ny, n_theta)] = n_c2c
                    prnt_node_map[(nx, ny, n_theta)] = (curnt_x, curnt_y, curnt_theta)
                    step_info_map[(nx, ny, n_theta)] = (n_vel, n_theta_dot, n_dist)
                    # node_action_map[(nx, ny, n_theta)] = (act[0], act[1])
                    hq.heappush(open_list, (n_tc, n_c2c, (nx, ny, n_theta)))
        # Stop if the algorithm can't find a solution
        if len(open_list)== 0:
            print('No solution could be found')
            break
    a_star_end_time = time.time()                              
    return a_star_end_time-a_star_strt_time   # how long did it take the algorithm to run

# return a_star_end_time-a_star_strt_time   # how long did it take the algorithm to run
#### ANIMATION SECTION ####
# Colours (R, G, B)
BACKGROUND = (0, 40, 255) # blue
RED = (255, 30, 70)
YELLOW = (255, 255, 0)
GREEN = (0, 100, 0)
WHITE = (255, 255, 255)
PURPLE = (93, 63, 211)
BLACK = (0, 0, 0)
# LIGHT_PURPLE = rgb(207, 159, 255), IRIS = (93, 63, 211), VIOLET = rgb(127, 0, 255)

# Game Setup
FPS = 60
fpsClock = pygame.time.Clock()

# Border rectangles to add 5 unit bloat to walls
#variableName = pygame.Rect(Xcoordinate, Ycoordinate, width, height)
wrect1 = pygame.Rect(0,0, b_rect1_x, bloat) # flat-rect at upper left corner
wrect2 = pygame.Rect(0, bloat, bloat, WINDOW_HEIGHT-bloat) # verticle rect on left-side border
wrect3 = pygame.Rect(bloat, WINDOW_HEIGHT-bloat, (2000+500)/10 -2*bloat, bloat) # rect at bottom-left corner
wrect4 = pygame.Rect(b_rect1_x + b_rect1_w, 0, WINDOW_WIDTH-(b_rect1_x+b_rect1_w), bloat) # flat rect along top border, after rect1 obstacle on left.
wrect5 = pygame.Rect(WINDOW_WIDTH-bloat, bloat, bloat, WINDOW_HEIGHT-bloat) # verticle rect along right border
wrect6 = pygame.Rect(b_rect2_x + b_rect2_w, WINDOW_HEIGHT-bloat, WINDOW_WIDTH-(b_rect2_x + b_rect2_w + bloat), bloat) #rect along bottom border from rect 2 obstacle all the way to right border.

def draw_environment(WINDOW):
    """ Draws the obstacles and background """
    WINDOW.fill(BACKGROUND)
    #bloat around rectangular obstacles. pygame.draw.rect(surface, colour, rectangleObject)
    pygame.draw.rect(WINDOW, YELLOW, (b_rect1_x, 0, b_rect1_w, b_rect1_l))  # 1st bloated rect obstacle on left
    pygame.draw.rect(WINDOW, YELLOW, (b_rect2_x, b_rect2_y, b_rect2_w, b_rect2_l)) # 2nd bloated rect obstacle on left
    # Drawing bloated walls
    pygame.draw.rect(WINDOW, YELLOW, wrect1)
    pygame.draw.rect(WINDOW, YELLOW, wrect2)
    pygame.draw.rect(WINDOW, YELLOW, wrect3)
    pygame.draw.rect(WINDOW, YELLOW, wrect4)
    pygame.draw.rect(WINDOW, YELLOW, wrect5)
    pygame.draw.rect(WINDOW, YELLOW, wrect6)
    # pygame.draw.circle(surface, colour, center, radius, width) # width is optional
    # Draw bloated circle obstacle
    #pygame.draw.circle(surface, colour, center, radius, width) # width is optional
    pygame.draw.circle(WINDOW, YELLOW, (center_x,center_y), bltd_circle_r)

    # Actual obstacles
    # pygame.draw.polygon(WINDOW, RED, (vertxA, vertxB, vertxC, vertxD, vertxE, vertxF))  # hexagon
    pygame.draw.rect(WINDOW, RED, (b_rect1_x+bloat, 0, rect1_w, rect1_l))
    pygame.draw.rect(WINDOW, RED, (b_rect2_x+bloat, WINDOW_HEIGHT- rect2_l, rect2_w, rect2_l))
    pygame.draw.circle(WINDOW, RED, (center_x,center_y), circle_r)
    pygame.display.update()

def draw_action_curve(surface, Xi, Yi, Thetai, UL, UR, color):
    t = 0
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = math.radians(Thetai)
    UL = UL * 2 * math.pi / 60  # convert rpm to rad/s
    UR = UR * 2 * math.pi / 60  # convert rpm to rad/s
    points = []
    while t < 1:
        t = t + dt
        Xn += 0.5 * wheel_r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * wheel_r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (wheel_r / wheel_dist) * (UR - UL) * dt
        if ((round(Xn), round(Yn)) in obstacle_pts) or (round(Xn)/thr_x, round(Yn)/thr_y, round(Thetan)/thr_theta) in animation_v_matrix_idx:
            return False
        points.append((int(Xn), int(Yn)))
    animation_v_matrix_idx.add((round(Xn)/thr_x, round(Yn)/thr_y, round(Thetan)/thr_theta))
    pygame.draw.lines(surface, color, False, points, 1)  # Draw on the curve surface
    return True

def animate_optimal_path(window, path):
    """Function to animate moving from start to goal using the optimal path, drawing lines between nodes."""
    # Convert nodes to int and also to (x,y) tuples and putting them in a list to use pygame.draw.lines
    path_points = [(int(node[0]), int(node[1])) for node in path]
    for node in path_points:
        pygame.draw.circle(window, BLACK, (node[0], node[1]), 3)
        pygame.display.update()
        pygame.time.delay(100)
    pygame.draw.lines(window, WHITE, False, path_points, 1)  # Draw lines connecting each point in path
    pygame.display.update()
    pygame.time.delay(10)  # delay for animation speed

######################### ROS PORTION ###############################################################
class RobotMovePubNode():
    """A class to create a publisher node to control the turtle_bot"""

    def __init__(self):
        super().__init__("pose_subscriber")
        # publisher to send velocity commands to the turtle_bot
        self.cmd_vel_publisher_ = self.create_publisher(
             Twist, "/cmd_vel", self.velocity_cmd_callback, 10)
  
    def velocity_cmd_callback(self):
        vel_cmd = Twist()
        for waypoints in step_info:
            vel_cmd.linear.x = float(waypoints[0]/100)  # divide by 100 to convert cm/s to m/s
            vel_cmd.angular.z = float(waypoints[1]) # already in rad/s
            self.cmd_vel_publisher_.publish(vel_cmd)
            sleep(0.1)

#### MAIN FUNCTION (start algorithm, then animates) ###############
def main(args=None):
# def main():
    """ Main function to start and animate the algorithm search"""
    global start_x, start_y
    global start_theta
    global goal_x, goal_y
    global RPM_L, RPM_R
    global clrnc
   
    a_star_duration = A_star(1, 1)

    print(f"A* Algorithm Execution Time: {a_star_duration} seconds")

    # if optimal path is found, create animation
    if len(path) > 0:
        animation_strt_time = time.time()
        animation_run_flag = True
        # Create Pygame window
        WINDOW = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Robot Path Animation with A* ")
        draw_environment(WINDOW)
        pygame.draw.circle(WINDOW, WHITE, (int(goal_x), int(goal_y)), int(round(1.5)))
        # Create a separate surface for drawing curves to avoid flickering effect if it curves were drawn directly.
        # curve_surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA) # pygam.srcalpha keeps the surface transparent
        nodes_list = list(prnt_node_map.keys())[:-1] # slicing to avoid drawing robot motion for the last node, since after it the goal is found
        if 0<len(nodes_list)<15000:
            nodes_per_frame = int(60000/5)
            dly = 2
        elif 15000<len(nodes_list)<30000:
            nodes_per_frame = int(60000/4)
            dly = 1
        elif 30000< len(nodes_list) < 45000:
            nodes_per_frame  = int(60000/2)
            dly =1
        elif 45000< len(nodes_list) < 60000: 
            nodes_per_frame = 45000
            dly = 0
        if len(nodes_list) >= 60000:
            nodes_per_frame = 60000
            dly = 0
        # nodes_per_frame = 60000 # might need to be adjusted
        for i in range(0, len(nodes_list), nodes_per_frame):
            for node in nodes_list[i:i+nodes_per_frame]:
                for action in actions:
                    # if draw_action_curve(curve_surface, node[0], node[1], node[2], action[0], action[1], GREEN):
                    if draw_action_curve(WINDOW, node[0], node[1], node[2], action[0], action[1], GREEN):
                        # WINDOW.blit(curve_surface, (0, 0))  # Blit the curve surface onto the main window
                        pygame.display.update()
                        pygame.time.delay(dly)  # Delay between each curve

        animate_optimal_path(WINDOW, path)
        animation_end_time = time.time()
        animation_run_time = animation_end_time - animation_strt_time
        print(f"Animation Execution Time: {animation_run_time} seconds, \n")
        print(f'Total execution time of search algorithm & animation {a_star_duration+animation_run_time} seconds')
        
        while animation_run_flag:
            for event in pygame.event.get():
                if event.type == QUIT:
                    animation_run_flag = False  # Set flag to False to exit loop & close display
            fpsClock.tick(FPS)
        pygame.quit() 
        start_x, start_y, start_theta = 500/10, 1000/10, 0.0 # fixing start point to be the same as where robot spawns in gazebo
        RPM_L, RPM_R = 22, 22
        clrnc = 29
        goal_x, goal_y = 5750/10, 1000/10

        open_list.clear()  # purging list used for heapq for nodes in the open-list
        visited_matrix_idx.clear() #set used to store the index of each visited node if they were added to a visted-node-matrix (see proj3 part 1 pdf page 14). replaces the closed-list from proj2
        prnt_node_map.clear() # clearing dictionary (key=node, val=parent) for all the nodes visited mapped to their parents to be used for backtracking
        lowest_c2c_map.clear() # clearing dictionary (key=node, val=c2c) to keep track of the nodes and their cost, used to ensure only lowest cost in open-list
        step_info_map.clear() # purging dictionary (key = node, val= (linear_velocity, angular_velocity, distance_covered) to keep track of the nodes, traveled distance, linear & angular velocities
        path.clear()          # deque used for storing nodes after ordering them using backtracking
        step_info.clear()     # clearing deque
        
        A_star(1,0)  # using A_star to find optimal path for robot in gazebo world
        step_info.append((0.0,0.0))
        print('starting ROS commands to move robot in gazebo')
        # print('start_x, start_y, start_theta=: ', start_x, start_y, start_theta)
        # print('goal_x, goaly =:', goal_x, goal_y)
        # print(f'size of step_info: {len(step_info)}')
        # print('50th item and 101 item in step_info', step_info[49], step_info[100])
        # Initialize and run the node
        rclpy.init(args=args)
        node = RobotMovePubNode()
        print('initialization complete, spinning the node')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()
    else:
        print('The algorithm could not find an optimal path')

if __name__ == "__main__":
    main()