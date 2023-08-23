#!/bin/bash

# Usage - From ardupilot root directory, run - libraries/SITL/examples/follow-copter.sh $GCS_IP
# $GCS_IP is the IP address of the system running the GCs, by default is 127.0.0.1
# Use "follow-mavproxy.sh" to run MAVProxy with all vehicles
# Or connect your GCS using multicast UDP
# If you can't use multicast, you can connect via UDP on vehicle 1, which will relay telemetry
# from the other vehicles

# Start gazebo right away
echo "Starting Gazebo!"
gz sim -r iris_runway_2_drones.sdf &
sleep 3

# Kill all SITL binaries when exiting
trap "killall -9 arducopter" SIGINT SIGTERM EXIT

# Get the ArduPilot directory (ROOTDIR)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROOTDIR="$(dirname "$(dirname "$(dirname $SCRIPT_DIR)")")"
COPTER=$ROOTDIR/build/sitl/bin/arducopter

# Drones will be located here
HOMELAT=-35.280252
HOMELONG=149.005821
# HOMELAT=45.378083
# HOMELONG=-71.942257
HOMEALT=597.3

# Set GCS_IP address
if [ -z $1 ]; then
    GCS_IP="127.0.0.1"
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
        MCAST_IP_PORT="" # Use default IP, port
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
echo "Starting copter 1"
mkdir -p copter1

# create default parameter file for the leader
cat <<EOF > copter1/leader.parm
SYSID_THISMAV 1
AUTO_OPTIONS 7
WPNAV_SPEED 500
FOLL_ENABLE 0
EOF

pushd copter1
$COPTER --model JSON --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --uartA udpclient:$GCS_IP --uartC mcast:$MCAST_IP_PORT --defaults $BASE_DEFAULTS,leader.parm &
popd

SYSID=2
echo "Starting copter $SYSID"
mkdir -p copter$SYSID

# create default parameter file for the follower
cat <<EOF > copter2/follow.parm
SYSID_THISMAV $SYSID
FOLL_ENABLE 1
FOLL_OFS_X -6
FOLL_OFS_Z -4
FOLL_OFS_TYPE 1
FOLL_SYSID 1
FOLL_DIST_MAX 1000
FOLL_YAW_BEHAVE 3
FOLL_POS_P 0.3
FOLL_POS_D 0.0
FOLL_ALT_TYPE 1
FOLL_HD_ERR_D 90
AUTO_OPTIONS 7
LAND_SPEED 300
LAND_TYPE 3
LAND_RVT_PWM 1200
LAND_MNVR 1
LAND_PTZ_HGT_M 0.50
FOLL_SPD_CMS 2500
WPNAV_SPEED 2500
ANGLE_MAX 5000
EOF

#=================================================================================================
# FOR A SINGLE FOLLOWER:
#=================================================================================================
pushd copter2
LAT=$(echo "$HOMELAT - 0.0005" | bc -l)
LONG=$(echo "$HOMELONG" | bc -l)
$COPTER --model JSON --home=$LAT,$LONG,$HOMEALT,0 --uartA tcp:0 --uartC mcast:$MCAST_IP_PORT --instance 1 --defaults $BASE_DEFAULTS,follow.parm &
popd

#=================================================================================================
# FOR MULTIPLE FOLLOWERS:
#=================================================================================================
# now start other copters to follow the first, using
# a separate directory to keep the eeprom.bin and logs separate
# each copter will have an offset starting location (5*SYSID,5*SYSID)m from leader copter
# each copter will follow at SYSID*5m in X dir from leader
# for i in $(seq $NCOPTERS); do
#     SYSID=$(expr $i + 1)
    
#     echo "Starting copter $SYSID"
#     mkdir -p copter$SYSID

#     # create default parameter file for the follower
#     cat <<EOF > copter$i/follow.parm
#     SYSID_THISMAV $SYSID
#     FOLL_ENABLE 1
#     # FOLL_OFS_X $(echo "-5*$i" | bc -l)
#     FOLL_OFS_X -6
#     FOLL_OFS_Z -4
#     FOLL_OFS_TYPE 1
#     FOLL_SYSID 1
#     FOLL_DIST_MAX 1000
#     FOLL_YAW_BEHAVE 3
#     FOLL_ALT_TYPE 1
#     AUTO_OPTIONS 7
#     FOLL_SPD_CMS 1250
#     LAND_SPEED 400
#     LAND_TYPE 3
#     FOLL_POS_P 0.3
#     FOLL_POS_D 0.0
#     WPNAV_SPEED 1250
# EOF

#     pushd copter$i
#     LAT=$(echo "$HOMELAT + 0.0005*$i" | bc -l)
#     LONG=$(echo "$HOMELONG + 0.0005*$i" | bc -l)
#     $COPTER --model quad --home=$LAT,$LONG,$HOMEALT,0 --uartA tcp:0 --uartC mcast:$MCAST_IP_PORT --instance $i --defaults $BASE_DEFAULTS,follow.parm &
#     popd
# done


# gnome-terminal -- /bin/sh -c 'echo "Starting Gazebo!"; gz sim -r iris_runway_2_drones.sdf'
# sleep 3
# echo "Starting Mavproxy!"
# mavproxy.py --master=mcast: --console --map --load-module horizon

# gnome-terminal -- /bin/sh -c 'echo "Starting Mavproxy!";
# mavproxy.py --master=mcast: --console --map --load-module horizon;
# echo "vehicle 1";
# echo "mode guided";
# echo "vehicle 2";
# echo "mode guided";'

echo "Starting MAVProxy!"
gnome-terminal -- bash -c 'mavproxy.py --master=mcast: --console --map --load-module horizon;'

wait

