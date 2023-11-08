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

# start up main (leader) copter in the subdir (copter2)
SYSID=2
echo "Starting copter $SYSID"
mkdir -p copter$SYSID

# create default parameter file for the leader
cat <<EOF > copter$SYSID/leader.parm
SYSID_THISMAV $SYSID
LOG_DISARMED 1
SIM_RATE_HZ 800
AUTO_OPTIONS 7
WPNAV_SPEED 2500
LOIT_SPEED 2500
FOLL_ENABLE 0
EK3_DRAG_BCOEF_X 100
EK3_DRAG_BCOEF_Y 100
EOF

pushd copter$SYSID
# $COPTER --instance 1 --model quad --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --uartA udpclient:$GCS_IP --uartC mcast:$MCAST_IP_PORT --defaults $BASE_DEFAULTS,leader.parm &
# $COPTER --instance 1 --model json:$JSON_IP --home=$HOMELAT,$HOMELONG,$HOMEALT,$HEADING --uartA udpclient:$GCS_IP --uartC mcast:$MCAST_IP_PORT --defaults $BASE_DEFAULTS,leader.parm &
# gnome-terminal -- sh -c '../Tools/autotest/sim_vehicle.py -v ArduCopter --instance 1 -f JSON:"$1" -l "$2","$3","$4","$5" --sitl-instance-args="--uartA udpclient:$6 --uartC mcast:" --add-param-file="$7" --add-param-file=leader.parm --no-mavproxy -N;' sh "$JSON_IP" "$HOMELAT" "$HOMELONG" "$HOMEALT" "$HEADING" "$GCS_IP" "$BASE_DEFAULTS"
# gnome-terminal -- sh -c '../Tools/autotest/sim_vehicle.py -v ArduCopter --instance 1 -f JSON:"$1" -l "$2","$3","$4","$5" --sitl-instance-args="--uartA udpclient:$6 --uartC mcast:" --add-param-file="$7" --add-param-file=leader.parm --no-mavproxy -N; bash;' sh "$JSON_IP" "$HOMELAT" "$HOMELONG" "$HOMEALT" "$HEADING" "$GCS_IP" "$BASE_DEFAULTS"
# sim_vehicle.py -v ArduCopter --instance 1 -f JSON:$JSON_IP -l $HOMELAT,$HOMELONG,$HOMEALT,$HEADING --sitl-instance-args="--uartA udpclient:$GCS_IP --uartC mcast:" --add-param-file=$BASE_DEFAULTS --add-param-file=leader.parm --no-mavproxy -N &
$COPTER --instance 1 --model json:$JSON_IP --home=$HOMELAT,$HOMELONG,$HOMEALT,$HEADING --uartA udpclient:$GCS_IP --uartC mcast:$MCAST_IP_PORT --defaults $BASE_DEFAULTS,leader.parm &
popd

SYSID=1
echo "Starting copter $SYSID"
mkdir -p copter$SYSID

# create default parameter file for the follower
cat <<EOF > copter$SYSID/follow.parm
SYSID_THISMAV $SYSID
LOG_DISARMED 1
SIM_RATE_HZ 800
FRAME_TYPE 1
AUTO_OPTIONS 7
FOLL_ENABLE 1
FOLL_DELAY 80
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
LAND_PTZ_HGT_M 0.50
LAND_USE_RNGFND 0
WPNAV_SPEED 2500
LOIT_SPEED 2500
ANGLE_MAX 5000
EK3_DRAG_BCOEF_X 100
EK3_DRAG_BCOEF_Y 100
FLTMODE1 5
FLTMODE4 23
FLTMODE6 18
EOF

OFFSET=0.00007
LAT=$(echo "$HOMELAT + $OFFSET" | bc -l)
LONG=$(echo "$HOMELONG" | bc -l)
ALT=$(echo "$HOMEALT" | bc -l)
pushd copter$SYSID
# $COPTER --model quad --home=$LAT,$LONG,$HOMEALT,0 --uartA udpclient:$GCS_IP --uartC mcast:$MCAST_IP_PORT --defaults $BASE_DEFAULTS,follow.parm &
$COPTER --model json:$JSON_IP --home=$LAT,$LONG,$ALT,0 --uartA udpclient:$GCS_IP --uartC mcast:$MCAST_IP_PORT --defaults $BASE_DEFAULTS,follow.parm &
popd

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

