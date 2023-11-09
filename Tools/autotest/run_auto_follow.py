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
    pattern = r'Dist from v-target: x:([-]?\d+\.\d+) m; y:([-]?\d+\.\d+) m'
    match = re.search(pattern, input_string)
    
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        return None

def replace_commas_with_spaces(file_path):
    try:
        with open(file_path, 'r') as file:
            content = file.read()
        
        modified_content = content.replace(',', ' ')

        with open(file_path, 'w') as file:
            file.write(modified_content)
        
        print(f"Commas replaced with spaces in {file_path}")
    except FileNotFoundError:
        print(f"File not found: {file_path}")
    except Exception as e:
        print(f"An error occurred: {str(e)}")

def main(argv):

    udp_gcs_ip="192.168.64.1"
    json_ip="192.168.64.1"
    wipe=True
    speedup=1
    vehicle_home_in = None
    offset = "%f,%f,%f,%f" % (0,0,0,0)

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

    test1 = FollowTest(binary, 0)
    test2 = FollowTest(binary, 1)

    if vehicle_home_in is not None:
        vehicle_home = vehicle_home_in
    else:
        HOME = test1.sitl_start_location()
        vehicle_home = "%f,%f,%f,%f" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    
    default_params = "../Tools/autotest/default_params/copter.parm"
    real_follow_params = "../followParams/fastquad.param"
    real_leader_params = "../followParams/target.param"
    follow_params = "follow.parm"
    leader_params = "leader.parm"

    # current_directory = os.getcwd()
    # print(f"Current directory: {current_directory}")
    # replace_commas_with_spaces(current_directory+"/followParams/fastquad.parm")

    # replace_commas_with_spaces(current_directory+"/followParams/target.parm")

    home_tuple = tuple(map(float, vehicle_home.split(',')))
    offset_tuple = tuple(map(float, offset.split(',')))
    if len(home_tuple) == 4 and len(offset_tuple) == 4:
        drone_home_tuple = tuple(a + b for a, b in zip(home_tuple, offset_tuple))
        print(f"Result Drone Home: {drone_home_tuple}")
    else:
        print("Error: The 'home' and 'offset' argument must contain four values separated by commas.")

    drone_home = '%s,%s,%s,%s' % drone_home_tuple
    model = "json:%s" % json_ip

    test1.progress("Starting Drone Simulator")
    customisations1 = [
        "--uartD", "udpclient:%s" % udp_gcs_ip, 
        "--uartC", "mcast:",
        "-I", "%s" % test1.instance,
        "--defaults", "%s,%s" % (default_params, follow_params),
        ]
    with pushd('copter1'):
        test1.start_SITL(model=model,
                    home=drone_home,
                    speedup=speedup,
                    customisations=customisations1,
                    wipe=wipe)
    
    test2.progress("Starting Target Simulator")
    customisations2 = [
        "--uartD", "udpclient:%s" % udp_gcs_ip, 
        "--uartC", "mcast:",
        "-I", "%s" % test2.instance,
        "--defaults", "%s,%s" % (default_params, leader_params),
        ]
    with pushd('copter2'):
        test2.start_SITL(model=model,
                    home=vehicle_home,
                    speedup=speedup,
                    customisations=customisations2,
                    wipe=wipe)
    try:
        test1.get_mavlink_connection_going()
        test1.mav.target_system = 1
        test1.mav.target_component = 0
        test1.mav.do_connect()

        test2.get_mavlink_connection_going()
        test2.mav.target_system = 2
        test2.mav.target_component = 0
        test2.mav.do_connect()
    
        test1.progress('SITL Drone Started')
        test2.progress('SITL Target Started')

        test1.change_mode('GUIDED')
        test2.change_mode('STABILIZE')
        test1.wait_ready_to_arm()

        test1.arm_vehicle()
        test1.takeoff(5, mode='GUIDED')
        test1.change_mode('FOLLOW')
        test1.delay_sim_time(8)

        test2.set_rc(1,1500)
        test2.set_rc(2,1500)
        test2.set_rc(3,1000)
        test2.set_rc(4,1500)
        test2.set_rc(8,1000)
        test2.arm_vehicle()

        
        vehicle_vel = 50/3.6
        pwm_vel = round(((vehicle_vel-min_vel)/(max_vel-min_vel))*1000 + 1000)

        test2.set_rc(8,pwm_vel)
        test2.delay_sim_time(5)

        
        while True:
            distance_string = test1.wait_text("Dist from v-target:*", regex=True)
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
        
        
        # vehicle_vel = 10
        # pwm_vel = round(((vehicle_vel-min_vel)/(max_vel-min_vel))*1000 + 1000)
        # test2.set_rc(8,pwm_vel)
        test1.change_mode('THROW')
        test2.delay_sim_time(10)
        # test2.takeoff(10, mode='GUIDED')
        # test2.guided_local_velocity_target(5,0,0)
        # test1.wait_disarmed()
        # test2.wait_disarmed()

        


    except Exception:
        print(test1.mav.target_system)
        print(test1.mav.target_component)
        print(test1.mav.address)
        print(test2.mav.target_system)
        print(test2.mav.target_component)
        print(test2.mav.address)
        if test1.rc_thread is not None:
            test1.progress("Joining RC thread in __del__")
            test1.rc_thread_should_quit = True
            test1.rc_thread.join()
            test1.rc_thread = None
        if test2.rc_thread is not None:
            test2.progress("Joining RC thread in __del__")
            test2.rc_thread_should_quit = True
            test2.rc_thread.join()
            test2.rc_thread = None
        test1.close()
        test2.close()
        util.pexpect_close_all()
        raise

    if test1.rc_thread is not None:
        test1.progress("Joining RC thread in __del__")
        test1.rc_thread_should_quit = True
        test1.rc_thread.join()
        test1.rc_thread = None
    if test2.rc_thread is not None:
        test2.progress("Joining RC thread in __del__")
        test2.rc_thread_should_quit = True
        test2.rc_thread.join()
        test2.rc_thread = None
    test1.close()
    test2.close()
    util.pexpect_close_all()
    os.killpg(0, signal.SIGKILL)
    sys.exit(1)


if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, alarm_handler)
    main(sys.argv[1:])
    