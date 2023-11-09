#!/usr/bin/env python
"""
ArduPilot Follow Mode Python Connection for command
"""
from __future__ import print_function
import math
import os
import time
import signal
import sys
import os
import getopt
import re
from contextlib import contextmanager

from pysim import util

from pymavlink.rotmat import Vector3

from follow import FollowTest

max_vel = 33.3333
min_vel = 5.5555

@contextmanager
def pushd(new_dir):
    previous_dir = os.getcwd()
    os.chdir(new_dir)
    try:
        yield
    finally:
        os.chdir(previous_dir)


def alarm_handler(signum, frame):
    """Handle test timeout."""
    util.pexpect_close_all()
    os.killpg(0, signal.SIGKILL)
    sys.exit(1)


def extract_numbers(input_string):
    """Extract distances from follow message"""
    pattern = r'Dist from v-target: x:([-]?\d+\.\d+) m; y:([-]?\d+\.\d+) m'
    match = re.search(pattern, input_string)
    
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        return None


def main(argv):
    # Load default parameters/arguments
    udp_gcs_ip="192.168.64.1"
    json_ip="192.168.64.1"
    vehicle_home_in = None
    offset = "%f,%f,%f,%f" % (0,0,0,0)
    wipe=True
    speedup=1

    # Parse arguments
    try:
        opts, args = getopt.getopt(argv, "", ["binary=", "home=", "offset=", "json-ip=", "gcs-ip="])
    except getopt.GetoptError:
        print("Usage: command_follow.py --home='45,73,0,0' --json-ip='192.168.64.1'")
        sys.exit(2)

    for opt, arg in opts:
        if opt == "--binary":
            binary = arg
        elif opt == "--home":
            vehicle_home_in = arg
        elif opt == "--offset":
            offset = arg
        elif opt == "--json-ip":
            json_ip = arg
        elif opt == "--gcs-ip":
            udp_gcs_ip = arg
    model = "json:%s" % json_ip

    # Generate FollowTest objects
    drone1 = FollowTest(binary, 0)
    vehic2 = FollowTest(binary, 1)

    # Define vehicle home
    if vehicle_home_in is not None:
        vehicle_home = vehicle_home_in
    else:
        HOME = drone1.sitl_start_location()
        vehicle_home = "%f,%f,%f,%f" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    
    # Define drone home
    home_tuple = tuple(map(float, vehicle_home.split(',')))
    offset_tuple = tuple(map(float, offset.split(',')))
    if len(home_tuple) == 4 and len(offset_tuple) == 4:
        drone_home_tuple = tuple(a + b for a, b in zip(home_tuple, offset_tuple))
        print(f"Result Drone Home: {drone_home_tuple}")
    else:
        print("Error: The 'home' and 'offset' argument must contain four values separated by commas.")
    drone_home = '%s,%s,%s,%s' % drone_home_tuple

    # Define AP param files
    default_params = "../Tools/autotest/default_params/copter.parm"
    follow_params = "follow.parm"
    leader_params = "leader.parm"

    # Create customisation string and start SITL (drone)
    drone1.progress("Starting Drone Simulator")
    customisations1 = [
        "--uartD", "udpclient:%s" % udp_gcs_ip, 
        "--uartC", "mcast:",
        "-I", "%s" % drone1.instance,
        "--defaults", "%s,%s" % (default_params, follow_params),
        ]
    with pushd('copter1'):
        drone1.start_SITL(model=model,
                    home=drone_home,
                    speedup=speedup,
                    customisations=customisations1,
                    wipe=wipe)
    
    # Create customisation string and start SITL (vehicle)
    vehic2.progress("Starting Target Simulator")
    customisations2 = [
        "--uartD", "udpclient:%s" % udp_gcs_ip, 
        "--uartC", "mcast:",
        "-I", "%s" % vehic2.instance,
        "--defaults", "%s,%s" % (default_params, leader_params),
        ]
    with pushd('copter2'):
        vehic2.start_SITL(model=model,
                    home=vehicle_home,
                    speedup=speedup,
                    customisations=customisations2,
                    wipe=wipe)
    
    try:
        # Start Mavlink connection between Python and both SITL instances
        drone1.get_mavlink_connection_going()
        drone1.mav.target_system = 1
        drone1.mav.target_component = 0
        drone1.mav.do_connect()

        vehic2.get_mavlink_connection_going()
        vehic2.mav.target_system = 2
        vehic2.mav.target_component = 0
        vehic2.mav.do_connect()
    
        drone1.progress('SITL Drone Started')
        vehic2.progress('SITL Target Started')

        # -------------------------- START OF TRIAL -------------------------- #

        # Wait arm ready
        drone1.change_mode('GUIDED')
        vehic2.change_mode('STABILIZE')
        drone1.wait_ready_to_arm()

        # Takeoff and start follow
        drone1.arm_vehicle()
        drone1.takeoff(5, mode='GUIDED')
        drone1.change_mode('FOLLOW')
        drone1.delay_sim_time(8)

        # Start vehicle RC commands and arm
        vehic2.set_rc(1,1500)
        vehic2.set_rc(2,1500)
        vehic2.set_rc(3,1000)
        vehic2.set_rc(4,1500)
        vehic2.set_rc(8,900)
        vehic2.arm_vehicle()
        vehic2.wait_armed()

        # Vehicle velocity command
        vehicle_vel = 110/3.6
        pwm_vel = round(((vehicle_vel-min_vel)/(max_vel-min_vel))*1000 + 1000)
        vehic2.set_rc(8,pwm_vel)
        vehic2.delay_sim_time(5)

        # Wait for drone to be near target for a short duration of time, then change to Throw mode
        while True:
            distance_string = drone1.wait_text("Dist from v-target:*", regex=True)
            distance_to_target = extract_numbers(distance_string)
            if distance_to_target:
                print(f"x: {distance_to_target[0]}, y: {distance_to_target[1]}")
                distance_to_target_xy = math.sqrt(distance_to_target[0]**2 + distance_to_target[1]**2)
                if distance_to_target_xy < 0.25:
                    if 'start_time' not in locals():
                        start_time = time.time()  # Record the start time when it first goes below 0.2
                    elapsed_time = time.time() - start_time
                    if elapsed_time >= 4.0:
                        break  # If it's under 0.2 for 5 seconds, exit the loop
                else:
                    if 'start_time' in locals():
                        del start_time  # Reset the timer if it goes above 0.2

        drone1.change_mode('THROW')
        vehic2.delay_sim_time(10)


        # -------------------------- END OF TRIAL -------------------------- #

    except Exception:
        drone1.end_test()
        vehic2.end_test()
        raise

    drone1.end_test()
    vehic2.end_test()
    os.killpg(0, signal.SIGKILL)
    sys.exit(1)


if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, alarm_handler)
    main(sys.argv[1:])
    