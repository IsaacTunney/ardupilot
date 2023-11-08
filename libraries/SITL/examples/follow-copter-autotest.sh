#!/bin/bash

# Usage - From ardupilot root directory, run - libraries/SITL/examples/follow-copter-json.sh $GCS_IP
# $GCS_IP is the IP address of the system running the GCs, by default is 127.0.0.1

# IP address of bobzwik laptop is 192.168.64.1

# This scrips opens a gnome-terminal and runs "follow-mavproxy.sh"

# Kill all SITL binaries when exiting
trap "killall -9 arducopter" SIGINT SIGTERM EXIT

# Get the ArduPilot directory (ROOTDIR)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROOTDIR="$(dirname "$(dirname "$(dirname $SCRIPT_DIR)")")"
COPTER=$ROOTDIR/build/sitl/bin/arducopter

# Drones will be located here
HOMELAT=45.19211630
HOMELONG=-73.44609269
HOMEALT=60.0
HEADING=87.5

# Set JSON IP, to link to MATLAB
JSON_IP="192.168.64.1"

# Set GCS_IP address
if [ -z $1 ]; then
    GCS_IP="192.168.64.1"
else
    GCS_IP=$1
fi

# Check if SITL copter has been built
if [ -f "$COPTER" ]
then
   echo "Found SITL executable"
else
   echo "SITL executable not found ($COPTER). Exiting"
   exit
fi

# Check if Platform is Native Linux, WSL or Cygwin
# Needed for correct multicast addressing
unameOut="$(uname -s)"

if [ "$(expr substr $unameOut 1 5)" == "Linux" ]; then
    # Check for WSL
    if grep -q Microsoft /proc/version; then
        MCAST_IP_PORT="127.0.0.1:14550"

    # Native Linux
    else
        MCAST_IP_PORT="" # Use default IP port
    fi

elif [ "$(expr substr $unameOut 1 6)" == "CYGWIN" ]; then
    MCAST_IP_PORT="0.0.0.0:14550"
fi

BASE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/copter.parm"

[ -x "$COPTER" ] || {
	./waf configure --board sitl
	./waf copter
}

# Set number of extra copters to be simulated, change this for increasing the count
NCOPTERS="1"

# start up main (leader) copter in the subdir (copter1)
echo "Starting copter 2"
mkdir -p copter2

# create default parameter file for the leader
cat <<EOF > copter2/leader.parm
SYSID_THISMAV 2
FRAME_TYPE 1
LOG_DISARMED 1
SIM_RATE_HZ 1200
FLTMODE1 0
FLTMODE2 0
FLTMODE3 0
FLTMODE4 0
FLTMODE5 0
FLTMODE6 0
SERVO8_FUNCTION 58

AUTO_OPTIONS 7
WPNAV_SPEED 2500
LOIT_SPEED 2500
FOLL_ENABLE 0
EK3_DRAG_BCOEF_X 0
EK3_DRAG_BCOEF_Y 0
EOF

SYSID=1
echo "Starting copter $SYSID"
mkdir -p copter$SYSID

# create default parameter file for the follower
cat <<EOF > copter$SYSID/follow.parm
SYSID_THISMAV $SYSID
LOG_DISARMED 1
FRAME_TYPE 1
SIM_RATE_HZ 1200

AUTO_OPTIONS 7
FOLL_ENABLE 1
FOLL_DELAY 0
FOLL_OFS_X -6
FOLL_OFS_Z -6
FOLL_OFS_TYPE 1
FOLL_SYSID 2
FOLL_DIST_MAX 1000
FOLL_YAW_BEHAVE 3
FOLL_POS_P 0.3
FOLL_POS_D 0.0
FOLL_ALT_TYPE 1
FOLL_HD_ERR_D 90
FOLL_SPD_CMS 2500
LAND_SPEED 300
LAND_TYPE 3
LAND_RVT_PWM 1200
LAND_MNVR 1
LAND_PTZ_HGT_M 1
LAND_USE_RNGFND 0
WPNAV_SPEED 2500
WPNAV_SPEED_DN 300
LOIT_SPEED 2500
ANGLE_MAX 5000
EK3_DRAG_BCOEF_X 0
EK3_DRAG_BCOEF_Y 0
EOF


OFFSETLAT=0.00007
OFFSETLON=0
OFFSETALT=0
OFFSETHEAD=0


python Tools/autotest/command_follow.py --home="$HOMELAT,$HOMELONG,$HOMEALT,$HEADING" --offset="$OFFSETLAT,$OFFSETLON,$OFFSETALT,$OFFSETHEAD" --json-ip=$JSON_IP --gcs-ip=$GCS_IP &

# Check if a mavproxy process is already running (must have both a "mavproxy.py" string and a "pts/" string (in the 2nd column) of the same line)
if ps -e | grep 'mavproxy.py' | awk '{print $2}' | grep -q 'pts/'; then
    echo "mavproxy.py is already running!"
else
    echo "Starting MAVProxy!"   
    # gnome-terminal -- bash -c 'mavproxy.py --master=mcast: --console --map --cmd="map set showgpspos 0; map set showgps2pos 0;";'
    # gnome-terminal -- bash -c 'mavproxy.py --master=mcast: --console --map --load-module horizon;'
    gnome-terminal -- bash -c 'mavproxy.py --master=mcast: --console --map;'
fi

wait

