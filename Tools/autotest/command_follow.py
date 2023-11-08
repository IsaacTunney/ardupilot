#!/usr/bin/env python
"""
ArduPilot Follow Mode Python Connection for command
"""
from __future__ import print_function
import copy
import math
import os
import shutil
import time
import numpy
import signal
import sys
import os
import getopt
import struct
import re
from contextlib import contextmanager

try:
    import queue as Queue
except ImportError:
    import Queue

from pymavlink import quaternion
from pymavlink import mavutil
from pymavlink import mavextra
from pymavlink import rotmat

from pysim import util
from pysim import vehicleinfo

from common import AutoTest
from common import NotAchievedException, AutoTestTimeoutException, PreconditionFailedException
from common import Test
from common import MAV_POS_TARGET_TYPE_MASK

from pymavlink.rotmat import Vector3


home = (45.19211630,-73.44609269,60.0,87.5)

testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(home[0],home[1],home[2],home[3])

max_vel = 33.3333
min_vel = 5.5555
binary = '/home/bobzwik/drone_stuff/ardupilot/build/sitl/bin/arducopter'

@contextmanager
def pushd(new_dir):
    previous_dir = os.getcwd()
    os.chdir(new_dir)
    try:
        yield
    finally:
        os.chdir(previous_dir)

class FollowTest(AutoTest):
    def __init__(self, veh_binary, instance):
        self.instance = instance
        super().__init__(veh_binary)

    def autotest_connection_string_to_ardupilot(self):
        return "tcp:127.0.0.1:%s" % (5760 + 10*self.instance)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sysid_thismav(self):
        return self.instance+1

    def set_streamrate(self, streamrate, timeout=20, stream=mavutil.mavlink.MAV_DATA_STREAM_ALL):
        '''set MAV_DATA_STREAM_ALL; timeout is wallclock time'''
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise NotAchievedException("Failed to set streamrate")
            self.mav.mav.request_data_stream_send(
                self.sysid_thismav(),
                self.sysid_thismav(),
                stream,
                streamrate,
                1)
            m = self.mav.recv_match(type='SYSTEM_TIME',
                                    blocking=True,
                                    timeout=1)
            if m is not None:
                break
    
    def user_takeoff(self, alt_min=30, timeout=30, max_err=5):
        '''takeoff using mavlink takeoff command'''
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, # param1
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     alt_min # param7
                     )
        self.wait_for_alt(alt_min, timeout=timeout, max_err=max_err)
    
    def takeoff(self,
                alt_min=30,
                takeoff_throttle=1700,
                require_absolute=True,
                mode="STABILIZE",
                timeout=120,
                max_err=5):
        """Takeoff get to 30m altitude."""
        self.progress("TAKEOFF")
        self.change_mode(mode)
        if not self.armed():
            self.wait_ready_to_arm(require_absolute=require_absolute, timeout=timeout)
            self.zero_throttle()
            self.arm_vehicle()
        if mode == 'GUIDED':
            self.user_takeoff(alt_min=alt_min, timeout=timeout, max_err=max_err)
        else:
            self.set_rc(3, takeoff_throttle)
        self.wait_for_alt(alt_min=alt_min, timeout=timeout, max_err=max_err)
        # self.hover()
        self.progress("TAKEOFF COMPLETE")

    def wait_for_alt(self, alt_min=30, timeout=30, max_err=5):
        """Wait for minimum altitude to be reached."""
        self.wait_altitude(alt_min - 1,
                           (alt_min + max_err),
                           relative=True,
                           timeout=timeout)
        
    def hover(self, hover_throttle=1500):
        self.set_rc(3, hover_throttle)

    def wait_statustext(self, text, timeout=20, the_function=None, check_context=False, regex=False, wallclock_timeout=False):
        """Wait for a specific STATUSTEXT, return that statustext message"""

        # Statustexts are often triggered by something we've just
        # done, so we have to be careful not to read any traffic that
        # isn't checked for being our statustext.  That doesn't work
        # well with getting the curent simulation time (which requires
        # a new SYSTEM_TIME message), so we install a message hook
        # which checks all incoming messages.

        # self.progress("Waiting for text : %s" % text.lower())
        if check_context:
            statustext = self.statustext_in_collections(text, regex=regex)
            if statustext:
                self.progress("Found expected text in collection: %s" % text.lower())
                return statustext

        global statustext_found
        global statustext_full
        statustext_full = None
        statustext_found = False

        def mh(mav, m):
            global statustext_found
            global statustext_full
            if m.get_type() != "STATUSTEXT":
                return
            if regex:
                self.re_match = re.match(text, m.text)
                if self.re_match:
                    statustext_found = True
                    statustext_full = m
            if text.lower() in m.text.lower():
                # self.progress("Received expected text: %s" % m.text.lower())
                statustext_found = True
                statustext_full = m

        self.install_message_hook(mh)
        if wallclock_timeout:
            tstart = time.time()
        else:
            tstart = self.get_sim_time()
        try:
            while not statustext_found:
                if wallclock_timeout:
                    now = time.time()
                else:
                    now = self.get_sim_time_cached()
                if now - tstart > timeout:
                    raise AutoTestTimeoutException("Failed to receive text: %s" %
                                                   text.lower())
                if the_function is not None:
                    the_function()
                self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=0.1)
        finally:
            self.remove_message_hook(mh)
        return statustext_full
    
    def guided_local_velocity_target(self, vx, vy, vz_up, timeout=3):
        " Check local target velocity being received by vehicle "
        self.progress("Setting local NED velocity target: (%f, %f, %f)" % (vx, vy, -vz_up))
        self.progress("Setting POSITION_TARGET_LOCAL_NED message rate to 10Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10)
        # mask specifying use only vx,vy,vz & accel. Even though we don't test acceltargets below currently
        #  a velocity only mask returns a velocity & accel mask
        target_typemask = (MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE | MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE)

        # Drain old messages and ignore the ramp-up to the required target velocity
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            # send velocity-control command
            self.mav.mav.set_position_target_local_ned_send(
                0, # timestamp
                self.sysid_thismav(), # target system_id
                1, # target component id
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
                0, # x
                0, # y
                0, # z
                vx, # vx
                vy, # vy
                -vz_up, # vz
                0, # afx
                0, # afy
                0, # afz
                0, # yaw
                0, # yawrate
            )
            m = self.assert_receive_message('POSITION_TARGET_LOCAL_NED')

            self.progress("Received local target: %s" % str(m))
    
    def rc_thread_main(self):
        chan16 = [1000] * 16

        sitl_output = mavutil.mavudp("127.0.0.1:%s" % (5501 + 10*self.instance), input=False)
        buf = None

        while True:
            if self.rc_thread_should_quit:
                break

            # the 0.05 here means we're updating the RC values into
            # the autopilot at 20Hz - that's our 50Hz wallclock, , not
            # the autopilot's simulated 20Hz, so if speedup is 10 the
            # autopilot will see ~2Hz.
            timeout = 0.02
            # ... and 2Hz is too slow when we now run at 100x speedup:
            timeout /= (self.speedup / 10.0)

            try:
                map_copy = self.rc_queue.get(timeout=timeout)

                # 16 packed entries:
                for i in range(1, 17):
                    if i in map_copy:
                        chan16[i-1] = map_copy[i]

            except Queue.Empty:
                pass

            buf = struct.pack('<HHHHHHHHHHHHHHHH', *chan16)

            if buf is None:
                continue

            sitl_output.write(buf)
        
def alarm_handler(signum, frame):
    """Handle test timeout."""
    util.pexpect_close_all()
    os.killpg(0, signal.SIGKILL)
    sys.exit(1)

def extract_numbers(input_string):
    pattern = r'Dist from v-target: x:(\d+\.\d+) m; y:(\d+\.\d+) m'
    match = re.search(pattern, input_string)
    
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        return None

def main(argv):
    udp_gcs_ip="192.168.64.1"
    json_ip="192.168.64.1"
    wipe=True
    speedup=1

    vehicle_home = '%s,%s,%s,%s' % home
    offset = '%s,%s,%s,%s' % (0,0,0,0)
    
    default_params = "/home/bobzwik/drone_stuff/ardupilot/Tools/autotest/default_params/copter.parm"
    follow_params = "follow.parm"
    leader_params = "leader.parm"

    try:
        opts, args = getopt.getopt(argv, "", ["home=", "offset=", "json-ip=", "gcs-ip="])
    except getopt.GetoptError:
        print("Usage: command_follow.py --home='45,73,0,0' --json-ip='192.168.64.1'")
        sys.exit(2)

    for opt, arg in opts:
        if opt == "--home":
            vehicle_home = arg
        elif opt == "--offset":
            offset = arg
        elif opt == "--json-ip":
            json_ip = arg
        elif opt == "--gcs-ip":
            udp_gcs_ip = arg
    
    home_tuple = tuple(map(float, vehicle_home.split(',')))
    offset_tuple = tuple(map(float, offset.split(',')))
    if len(home_tuple) == 4 and len(offset_tuple) == 4:
        drone_home_tuple = tuple(a + b for a, b in zip(home_tuple, offset_tuple))
        print(f"Result Drone Home: {drone_home_tuple}")
    else:
        print("Error: The 'home' and 'offset' argument must contain four values separated by commas.")

    drone_home = '%s,%s,%s,%s' % drone_home_tuple
    model = "json:%s" % json_ip

    test1 = FollowTest(binary, 0)
    test1.progress("Starting simulator")
    customisations1 = [
        "--uartD", "udpclient:%s" % udp_gcs_ip, 
        "--uartC", "mcast:",
        "-I", "%s" % test1.instance,
        "--defaults", "%s,%s" % (default_params, follow_params),
        ]

    test2 = FollowTest(binary, 1)
    test2.progress("Starting simulator")
    customisations2 = [
        "--uartD", "udpclient:%s" % udp_gcs_ip, 
        "--uartC", "mcast:",
        "-I", "%s" % test2.instance,
        "--defaults", "%s,%s" % (default_params, leader_params),
        ]
   
    with pushd('copter1'):
        test1.start_SITL(model=model,
                    home=drone_home,
                    speedup=speedup,
                    customisations=customisations1,
                    wipe=wipe)
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

        
        vehicle_vel = 20
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
        test2.delay_sim_time(20)
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
    