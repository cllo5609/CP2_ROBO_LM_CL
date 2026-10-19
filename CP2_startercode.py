import time
import pybullet as p
import pybullet_data
import random
import math
from typing import List

random.seed(1)

### VARIABLES ###

# You can modify these variables for testing, but the final solution needs to be implemented with this setup #

GOAL = [4.5, 4.5]
GOAL_RADIUS = 0.3
START_POS = [1.5, 1.5, 0]
BLOCKS = [[1.5, 2.5], [4, 2], [3,5]]
SPEEDUP = 10.0


### PYBULLET SETUP ###

def setup_pybullet(goal):
    """
    This function setups the pybullet environment. You do not need to modify anything in this function.
    """

    # Setup physics client and turtlebot
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf") 
    startPos = START_POS
    turtleId = p.loadURDF("turtlebot.urdf", startPos, globalScaling=2)
    sphereId = p.loadURDF("sphere.urdf", [goal[0], goal[1], 0.01], globalScaling=0.25)
    wallId = p.loadURDF("arena_walls6.urdf") 
    obstacles = [wallId]
    blocks = BLOCKS

    for block in blocks:
        blockId = p.loadURDF("block.urdf", [block[0], block[1], 0]) 
        obstacles.append(blockId)

    # set camera to bird eye view
    # note: can't set camera pitch to 90 exact -> gimble lock
    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-89.99, cameraTargetPosition=[4, 4, 0])
    return turtleId, obstacles

### TURTLEBOT SETUP ##

class Turtlebot():
    """
    This class sets up your robot (a turtlebot!). 
    Helper functions to help you interact with the pybullet environment have been provided.
    You are welcome to modify any functions if you'd like as part of your solution but it is not necessary.
    """

    def __init__(self, turtleId, obstacles):
        self.position = None
        self.turtleId = turtleId
        self.obstacles = obstacles
        self.old_goals = []

    def get_position(self):
        """
        Inputs: none
        Outputs: turtlebot position as [x,y,z]
        """
        pos, orn = p.getBasePositionAndOrientation(self.turtleId)
        return pos
    
    def get_orientation(self):
        """
        Inputs: none
        Outputs: turtlebot orientation as a quaternion
        """
        pos, orn = p.getBasePositionAndOrientation(self.turtleId)
        return orn
    
    def collision_check(self):
        """
        Inputs: none
        Outputs: True/False, whether or not the turtlebot is in collision with a wall or block
        """
        collision_check = False
        for obstacle in self.obstacles: # check walls and blocks
            contacts = p.getContactPoints(bodyA=turtlebot.turtleId, bodyB=obstacle)
            if contacts:
                collision_check = True
                print(f"Collision detected at pose: {turtlebot.get_position()}")
                break
        return collision_check
        
    def teleport(self, pos, orn):
        """
        Teleports turtlebot to desired position/orientation

        Inputs:
        -pos: desired turtlebot positon
        -orn: desired turtlebot orientation

        Outputs: None
        """
        p.resetBasePositionAndOrientation(self.turtleId, pos, orn)

    def set_velocities(self, leftWheelVelocity, rightWheelVelocity):
        """
        Set turtlebot wheel velocities

        Inputs:
        -leftWheelVelocity
        -rightWheelVelocity

        Outputs: None
        """
        # set velocities
        p.setJointMotorControl2(self.turtleId,0,p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=1000)
        p.setJointMotorControl2(self.turtleId,1,p.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity,force=1000)

    def plot_path(self, color_string: str, path_points: List[List[float]]) -> None:
        """
        Debug function for plotting paths in pybullet

        Inputs:
        -color_string: color of path line
        -path_points: path line points to plot List[[x,y,z]]

        Outputs: None
        """

        # Supported colors
        if color_string == "red":
            color = [1, 0, 0]
            line_width = 2
            z_offset = 0.05
        elif color_string == "green":
            color = [0, 1, 0]
            line_width = 8
            # Draw above the red tree so the solution path isn't hidden
            z_offset = 0.1
        else:
            print("That color does not exist to be viewed. Not plotting Path.")
            return

        # Loop through path points and add debug line
        for i in range(1, len(path_points)):
            p.addUserDebugLine(
                [path_points[i-1][0], path_points[i-1][1], path_points[i-1][2] + z_offset],
                [path_points[i][0],   path_points[i][1],   path_points[i][2] + z_offset],
                color,
                lineWidth=line_width
            )
    
    def show_intermediate_goal(self, intermediate_goal: List[float]) -> None:
        """
        Show randomly generated goals in pybullet

        Inputs:
        -intermediate_goal: path line points to plot [x,y,z]

        Outputs: None
        """

        # Loop through old random goals and change them to black to show they are no longer the active goal
        size = 0.15
        if self.old_goals:
            goal, goal_cross_1, goal_cross_2 = self.old_goals[-1]
            p.removeUserDebugItem(goal_cross_1)
            p.removeUserDebugItem(goal_cross_2)
            goal_cross_1 = p.addUserDebugLine([goal[0] - size, goal[1], 0.1], [goal[0] + size, goal[1], 0.1], lineColorRGB=[0, 0, 0], lineWidth=3)
            goal_cross_2 = p.addUserDebugLine([goal[0], goal[1] - size, 0.1], [goal[0], goal[1] + size, 0.1], lineColorRGB=[0, 0, 0], lineWidth=3)
            self.old_goals[-1] = (goal, goal_cross_1, goal_cross_2)

        # If new random goal provided, plot blue cross marking it.
        if intermediate_goal != None:
            goal_cross_1 = p.addUserDebugLine([intermediate_goal[0] - size, intermediate_goal[1], 0.1], [intermediate_goal[0] + size, intermediate_goal[1], 0.1], lineColorRGB=[0, 0, 1], lineWidth=3)
            goal_cross_2 = p.addUserDebugLine([intermediate_goal[0], intermediate_goal[1] - size, 0.1], [intermediate_goal[0], intermediate_goal[1] + size, 0.1], lineColorRGB=[0, 0, 1], lineWidth=3)
            self.old_goals.append((intermediate_goal, goal_cross_1, goal_cross_2))


### MAIN - implement your solution here! ###
class RRTNode:
    def __init__(self, pos, orn, parent=None, action=None, traj=None):
        self.pos = pos # [x, y, z]
        self.orn = orn # [x, y, z, w]
        self.parent = parent
        # stores the tuple (v_left, v_right, duration)
        self.action = action
        self.traj = traj if traj is not None else []


if __name__ == "__main__":
    goal = GOAL
    # Setup pybullet
    turtleId, obstacles = setup_pybullet(goal)
    # Setup turtlebot
    turtlebot = Turtlebot(turtleId, obstacles)

    print('starting at {}'.format(turtlebot.get_position()))
    # Turn off real-time simulation for manual step control
    p.setRealTimeSimulation(0)

    # Start recording video of the search
    search_log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./search.mp4")

    start_pos = turtlebot.get_position()
    start_orn = p.getQuaternionFromEuler([0, 0, 0])
    root_node = RRTNode(start_pos, start_orn)
    tree = [root_node]
    goal_reached = False
    final_node = None

    # Implement your solution in this loop
    while p.isConnected() and not goal_reached:

        # set random target sample
        if random.random() < 0.5:
            q_rand = GOAL
        else:
            q_rand = [random.uniform(0, 6), random.uniform(0, 6)]
            turtlebot.show_intermediate_goal([q_rand[0], q_rand[1], 0])

        # find nearest node in search tree to randomly sampled point
        print(len(tree))
        q_near = tree[0]
        min_dist_nearest = math.dist(q_near.pos[:2], q_rand)
        for node in tree:
            dist = math.dist(node.pos[:2], q_rand)
            if dist < min_dist_nearest:
                min_dist_nearest = dist
                q_near = node

        # Monte-Carlo propagations
        best_rollout = None
        min_dist_to_rand = float('inf')

        err_free_props = 0
        attempts = 0
        while err_free_props < 5 and attempts < 100:
            turtlebot.teleport(q_near.pos, q_near.orn)
            p.resetBaseVelocity(turtlebot.turtleId, linearVelocity=[0,0,0], angularVelocity=[0,0,0])
            p.resetJointState(turtleId, 0, 0, 0)
            p.resetJointState(turtleId, 1, 0, 0)

            # varying random linear and angular velocity inputs
            linear_v = random.uniform(-2, 10)
            angular_w = random.uniform(-10, 10)

            # convert to differential drive wheel velocities
            v_left = linear_v - angular_w
            v_right = linear_v + angular_w

            # varying time durations
            duration = random.randint(20, 60)

            turtlebot.set_velocities(v_left, v_right)
            collision = False

            traj = [list(q_near.pos)]
            # step in simulation to see where we end up
            for step in range(duration):
                p.stepSimulation()
                if turtlebot.collision_check():
                    # discard if collision
                    collision = True
                    break
                if step % 5 == 0:
                    traj.append(list(turtlebot.get_position()))

            if not collision:
                new_pos = turtlebot.get_position()
                new_orn = turtlebot.get_orientation()
                traj.append(list(new_pos))
                turtlebot.plot_path("red", traj)
                # check resulting Euclidean distance to random sample
                dist_to_rand = math.dist(new_pos[:2], q_rand)

                # use best rollout
                if dist_to_rand < min_dist_to_rand:
                    min_dist_to_rand = dist_to_rand
                    # traj.append(list(turtlebot.get_position()))
                    best_rollout = (new_pos, new_orn, (v_left, v_right, duration), traj)
                err_free_props += 1
            attempts += 1

        # add the optimal propagation to the tree
        if best_rollout is not None:
            new_pos, new_orn, action, traj = best_rollout
            new_node = RRTNode(new_pos, new_orn, parent=q_near, action=action, traj=traj)
            tree.append(new_node)
            # check if distance within radius 0.3 of goal
            dist_to_goal = math.dist(new_pos[:2], GOAL)
            if dist_to_goal <= GOAL_RADIUS:
                print("Goal region reached!")
                goal_reached = True
                final_node = new_node

        # Some example code below to get familiar with the simulation loop
        # set turtlebot to move forward
        # turtlebot.set_velocities(leftWheelVelocity=10, rightWheelVelocity=10)

        # IMPORTANT - You need to run this command for every step in simulation
        # Example: if you comment this out your turtlebot will not move despite setting the wheel velocities
        p.stepSimulation()

    # Stop the search recording before starting the final path recording
    p.stopStateLogging(search_log_id)

    if final_node is not None:
        path = []
        solution = []
        node = final_node
        while node is not None:
            # Prepend to maintain forward order
            path.insert(0, node)
            if node.traj:
                solution = node.traj + solution
            node = node.parent
        turtlebot.plot_path("green", solution)
        print("Setting simulation to starting position for playback.")
        # Reset turtlebot to the start position for playback and reset its states
        turtlebot.teleport(start_pos, start_orn)
        p.resetBaseVelocity(turtlebot.turtleId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
        p.resetJointState(turtleId, 0, 0, 0)
        p.resetJointState(turtleId, 1, 0, 0)
        turtlebot.set_velocities(0, 0)

        # Begin recording for solution
        log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./final_path.mp4")

        solution_nodes = path[1:]
        # Align robot pose and velocity to match the start condition of this propagation branch
        for current_node in solution_nodes:
            v_left, v_right, duration = current_node.action
            parent_pos = current_node.parent.pos
            parent_orn = current_node.parent.orn
            turtlebot.teleport(parent_pos, parent_orn)
            p.resetBaseVelocity(turtlebot.turtleId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
            p.resetJointState(turtleId, 0, 0, 0)
            p.resetJointState(turtleId, 1, 0, 0)

            # apply wheel velocities
            turtlebot.set_velocities(v_left, v_right)
            for i in range(duration):
                p.stepSimulation()
                # Divide by SPEEDUP to speedup live display of solution
                time.sleep(1.0 / 240.0 / SPEEDUP)

        # Stop the Robot!
        turtlebot.set_velocities(0, 0)
        print("Playback finished! Turtlebot has reached the goal.")
        p.stopStateLogging(log_id)

    p.disconnect()