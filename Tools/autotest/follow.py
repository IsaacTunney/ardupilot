#!/usr/bin/env python
"""
Follow Autotest Framework
"""
from __future__ import print_function
import math
import os
import time
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

from pymavlink import mavutil

from pysim import util

from common import AutoTest
from common import NotAchievedException, AutoTestTimeoutException
from common import MAV_POS_TARGET_TYPE_MASK

from pymavlink.rotmat import Vector3

home = (45.19211630,-73.44609269,60.0,87.5)

testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(home[0],home[1],home[2],home[3])

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
        