from dronekit import LocationGlobalRelative
from datetime import datetime

class MissionContext:
    def __init__(self, launch_pos: LocationGlobalRelative, mission_title: str = "", mission_start_time: str = None, rate=None):
        self.launch_pos = launch_pos
        self.mission_title = mission_title
        self.mission_start_time = mission_start_time or datetime.now().strftime('%H:%M:%S')
        self.rate = rate
        self.payload_expected = False
        self.has_payload = False
        self.descending = False
        self.target_alt = 4
        self.previous_command = -1
        self.item_start_time = ""
        self.waypoint_start_time = 0
        self.total_items = 0
        self.cmd_items = []
        # Add other common mission state variables as needed
