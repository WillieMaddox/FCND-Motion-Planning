import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
# from skimage.morphology import medial_axis
# from skimage.util import invert

from planning_utils import a_star, heuristic, create_grid, find_start_goal, prune
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.global_goal = np.array([0.0, 0.0, 0.0])
        self.local_goal = np.array([0.0, 0.0, 0.0])

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 2.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_goal[2] < 0.1:
                # This is redundant and also won't let us land on buildings.
                # if abs(self.local_position[2]) < 0.01:
                #     self.disarming_transition()
                self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 70
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # ###################################################################
        # (1) Can't modify self.local_position and (2) lat0, lon0 is
        # the same as the Sim default, so the next 4 TODO's have no effect.
        # ###################################################################
        # TODO: read lat0, lon0 from colliders into floating point values
        # with open('colliders.csv') as ifs:
        #     lat0, lon0 = ifs.readline().strip().split(',')
        # lat0 = float(lat0.split()[1])
        # lon0 = float(lon0.split()[1])
        # TODO: set home position to (lat0, lon0, 0)
        # self.set_home_position(lat0, lon0, 0)
        # TODO: retrieve current global position
        # global_position = (self._latitude, self._longitude, self._altitude)
        # global_position = self.global_position
        # TODO: convert to current local position using global_to_local()
        # local_position = global_to_local(global_position, self.global_home)
        # ###################################################################

        print(f'global home {self.global_home}, position {self.global_position}, local position {self.local_position}')

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print(f"North offset = {north_offset}, east offset = {east_offset}")

        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # self.global_goal = -122.397335, 37.792569, 0

        # TODO: convert start position to current position rather than map center
        grid_start_offset = tuple([int(np.around(self.local_position[0])), int(np.around(self.local_position[1]))])
        grid_start = (-north_offset + grid_start_offset[0], -east_offset + grid_start_offset[1])
        # TODO: adapt to set goal as latitude / longitude position and convert
        # Top of building 1 city block north. Found by manually flying drone through Sim.
        self.global_goal = np.array([-122.398124, 37.794221, 59])
        self.local_goal = global_to_local(self.global_goal, self.global_home)
        grid_goal_offset = tuple([int(np.around(self.local_goal[0])), int(np.around(self.local_goal[1]))])
        grid_goal = (-north_offset + grid_goal_offset[0], -east_offset + grid_goal_offset[1])
        print('Local Start and Goal: ', grid_start, grid_goal)

        # Run A* to find a path from start to goal
        path, cost = a_star(grid, heuristic, grid_start, grid_goal)
        # or move to a different search space such as a graph (not done here)
        # skeleton = medial_axis(invert(grid))
        # skel_start, skel_goal = find_start_goal(skeleton, grid_start, grid_goal)
        # path, cost = a_star(invert(skeleton).astype(int), heuristic, tuple(skel_start), tuple(skel_goal))
        # BUG: waffle #383: np.int64 waypoints raise TypeError: can't serialize 0
        # path = [(int(p1), int(p2)) for p1, p2 in path] # temp fix
        # if path[0] != grid_start:
        #     path = [grid_start] + path
        # if path[-1] != grid_goal:
        #     path = path + [grid_goal]
        print("Path length = {0}, path cost = {1}".format(len(path), cost))

        # TODO: prune path to minimize number of waypoints
        path = prune(path)
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
