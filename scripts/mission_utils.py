from enum import Enum
from dronekit import mavutil, Command
from offboard_utils import waypoint_relative
import time

class CommandType(Enum):
    START = 0
    WAYPOINT = 1
    DELAY = 2
    TAKEOFF = 3
    SEARCH = 5
    GRIPPER = 6
    PICKUP_PAYLOAD = 7
    FINISH = 8

class MissionItem:

    def __init__(self, command_type, target, title, speed=5, timeout=-1, obj_search=False, lnd_search=False):
        self.command = command_type
        self.target = target
        self.speed = speed
        self.timeout = timeout
        self.object_search = obj_search
        self.land_search = lnd_search
        self.title = title

    def add_commands(self, vehicle):

        if self.command == CommandType.START or self.command == CommandType.WAYPOINT or self.command == CommandType.SEARCH:
            speed_cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 1, self.speed, 0, 0, 0, 0, 0)

            cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, self.target.lat, self.target.lon, self.target.alt)
            vehicle.commands.add(speed_cmd)
            vehicle.commands.add(cmd)
        
        if self.command == CommandType.FINISH:
            cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            vehicle.commands.add(cmd)

def add_search_waypoints(cmds, center_pos, lane_size, loops, alt, speed, title, obj=True):
    r = lane_size
    for s in range(loops):
        S = s + 1
        cmds.append(MissionItem(CommandType.SEARCH, waypoint_relative(center_pos, r, r, alt), f"{title} - L{S}Start", speed, 20, obj, not obj))
        cmds.append(MissionItem(CommandType.SEARCH, waypoint_relative(center_pos, r, -r, alt), f"{title} - L{S}C1", speed, 20, obj, not obj))
        cmds.append(MissionItem(CommandType.SEARCH, waypoint_relative(center_pos, -r, -r, alt), f"{title} - L{S}C2", speed, 20, obj, not obj))
        cmds.append(MissionItem(CommandType.SEARCH, waypoint_relative(center_pos, -r, r, alt), f"{title} - L{S}C3", speed, 20, obj, not obj))
        cmds.append(MissionItem(CommandType.SEARCH, waypoint_relative(center_pos, r, r, alt), f"{title} - L{S}Finish", speed, 20, obj, not obj))
        r += lane_size