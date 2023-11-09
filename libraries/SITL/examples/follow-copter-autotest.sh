#!/bin/bash

# Usage - From ardupilot root directory, run - libraries/SITL/examples/follow-copter-autotest.sh $GCS_IP
# $GCS_IP is the IP address of the system running the GCs, by default is 127.0.0.1
# This startup script assumes the JSON IP address (for the MATLAB model) is the same as the $GCS_IP

# IP address of bobzwik laptop is 192.168.64.1

# This scrips opens a gnome-terminal and starts mavproxy.py

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

SIM_RATE=800

# Set GCS_IP address and JSON_IP to link to MATLAB
if [ -z $1 ]; then
    GCS_IP="192.168.64.1"
    JSON_IP="192.168.64.1"
else
    GCS_IP=$1
    JSON_IP=$1
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

# Create parameter file for the leader in the subdir (copter2)
SYSID=2
echo "Starting copter $SYSID"
mkdir -p copter$SYSID


cat <<EOF > copter$SYSID/leader.parm
SYSID_THISMAV   $SYSID
FRAME_TYPE      1
LOG_DISARMED    1
SIM_RATE_HZ     $SIM_RATE

FLTMODE1        0
FLTMODE2        0
FLTMODE3        0
FLTMODE4        0
FLTMODE5        0
FLTMODE6        0
SERVO8_FUNCTION 58
SERVO8_MAX      2000
SERVO8_MIN      1000

AUTO_OPTIONS    2

FOLL_ENABLE     0

SIM_GPS_HDG	    1
SIM_GPS_ACC     0.02
SIM_GPS_HZ      5
SIM_GPS_LAG_MS	120
SIM_GPS_NOISE	0.02
SIM_GPS_NUMSATS	24
SIM_GPS_VERR_X	0.05
SIM_GPS_VERR_Y	0.05
SIM_GPS_VERR_Z	0.05

EOF

# Create parameter file for the follower in the subdir (copter1)
SYSID=1
echo "Starting copter $SYSID"
mkdir -p copter$SYSID


cat <<EOF > copter$SYSID/follow.parm
SYSID_THISMAV   $SYSID
LOG_DISARMED    1
FRAME_TYPE      1
SIM_RATE_HZ     $SIM_RATE

AUTO_OPTIONS    2

FOLL_SPD_CMS    3500
WPNAV_SPEED     3500
LOIT_SPEED      3600
ANGLE_MAX       7500

FOLL_ENABLE     1
FOLL_DELAY      30
FOLL_OFS_X      -6
FOLL_OFS_Z      -8
FOLL_OFS_TYPE   1
FOLL_SYSID      2
FOLL_DIST_MAX   1000
FOLL_YAW_BEHAVE 3
FOLL_POS_P      0.3
FOLL_POS_P_100  0.3
FOLL_POS_D      0.0
FOLL_ALT_TYPE   1
FOLL_HD_ERR_D   90

LAND_SPEED      300
LAND_TYPE       3
LAND_RVT_PWM    1200
LAND_MNVR       1
LAND_PTZ_HGT_M  1
LAND_USE_RNGFND 0
LAND_COMMIT_HGT_ 1

WPNAV_ACCEL     1100
WPNAV_ACCEL_Z   50
WPNAV_JERK      10
WPNAV_RADIUS    200
WPNAV_RFND_USE  1
WPNAV_SPEED_UP  250
WPNAV_SPEED_DN  300

LOIT_ACC_MAX    1100
LOIT_ANG_MAX    0
LOIT_BRK_ACCEL  50
LOIT_BRK_DELAY  1
LOIT_BRK_JERK   250

EK3_ALT_M_NSE   2
EK3_POS_I_GATE  500
EK3_POSNE_M_NSE 0.3
EK3_VEL_I_GATE  500
EK3_VELD_M_NSE  0.2
EK3_VELNE_M_NSE 0.2
EK3_SRC1_POSXY  3
EK3_SRC1_POSZ   3
EK3_SRC1_VELXY  3
EK3_SRC1_VELZ   3
EK3_SRC1_YAW    1
EK3_DRAG_BCOEF_X 0
EK3_DRAG_BCOEF_Y 0

SIM_GPS_HDG	    0
SIM_GPS_ACC     0.02
SIM_GPS_HZ      5
SIM_GPS_LAG_MS	120
SIM_GPS_NOISE	0.02
SIM_GPS_NUMSATS	24
SIM_GPS_VERR_X	0.05
SIM_GPS_VERR_Y	0.05
SIM_GPS_VERR_Z	0.05

GPS_BLEND_MASK  5
GPS_BLEND_TC    10
GPS_COM_PORT    1
GPS_COM_PORT2   1
GPS_DELAY_MS    120
GPS_DRV_OPTIONS 0
GPS_GNSS_MODE   0
GPS_HDOP_GOOD   140
GPS_INJECT_TO   127
GPS_MB1_TYPE    0
GPS_MIN_DGPS    100
GPS_MIN_ELEV    -100
GPS_NAVFILTER   8
GPS_PRIMARY     0
GPS_TYPE        1

ATC_ACCEL_P_MAX     125000
ATC_ACCEL_R_MAX     125000
ATC_ACCEL_Y_MAX     27000
ATC_ANG_LIM_TC      1
ATC_ANG_PIT_P       7.5
ATC_ANG_RLL_P       7.5
ATC_ANG_YAW_P       4.5
ATC_ANGLE_BOOST     1
ATC_INPUT_TC        0.15
ATC_RAT_PIT_D       0.003
ATC_RAT_PIT_FF      0
ATC_RAT_PIT_FLTD    20
ATC_RAT_PIT_FLTE    0
ATC_RAT_PIT_FLTT    20
ATC_RAT_PIT_I       0.05
ATC_RAT_PIT_IMAX    0.5
ATC_RAT_PIT_P       0.05
ATC_RAT_PIT_SMAX    0
ATC_RAT_RLL_D       0.003
ATC_RAT_RLL_FF      0
ATC_RAT_RLL_FLTD    20
ATC_RAT_RLL_FLTE    0
ATC_RAT_RLL_FLTT    20
ATC_RAT_RLL_I       0.05
ATC_RAT_RLL_IMAX    0.5
ATC_RAT_RLL_P       0.05
ATC_RAT_RLL_SMAX    0
ATC_RAT_YAW_D       0
ATC_RAT_YAW_FF      0
ATC_RAT_YAW_FLTD    0
ATC_RAT_YAW_FLTE    2.5
ATC_RAT_YAW_FLTT    20
ATC_RAT_YAW_I       0.018
ATC_RAT_YAW_IMAX    0.5
ATC_RAT_YAW_P       0.18
ATC_RAT_YAW_SMAX    0
ATC_RATE_FF_ENAB    1
ATC_RATE_P_MAX      200
ATC_RATE_R_MAX      200
ATC_RATE_Y_MAX      0
ATC_SLEW_YAW        6000
ATC_THR_MIX_MAN     0.8
ATC_THR_MIX_MAX     0.8
ATC_THR_MIX_MIN     0.1

PSC_ACCZ_D      0.016
PSC_ACCZ_FF     0
PSC_ACCZ_FLTD   0
PSC_ACCZ_FLTE   20
PSC_ACCZ_FLTT   0
PSC_ACCZ_I      0.5
PSC_ACCZ_IMAX   400
PSC_ACCZ_P      0.25
PSC_ACCZ_SMAX   0
PSC_ANGLE_MAX   0
PSC_JERK_XY     12
PSC_JERK_Z      2
PSC_POSXY_P     1
PSC_POSZ_P      1
PSC_VELXY_D     0.5
PSC_VELXY_FF    0
PSC_VELXY_FLTD  5
PSC_VELXY_FLTE  5
PSC_VELXY_I     1
PSC_VELXY_IMAX  1000
PSC_VELXY_P     2.5
PSC_VELZ_D      0
PSC_VELZ_FF     0
PSC_VELZ_FLTD   5
PSC_VELZ_FLTE   5
PSC_VELZ_I      0
PSC_VELZ_IMAX   1000
PSC_VELZ_P      8

TERRAIN_ENABLE  0

LOG_BITMASK     145405

MOT_PWM_MAX     2000
MOT_PWM_MIN     1500
MOT_THST_HOVER  0.1340909
MOT_SPIN_ARM    0.1
MOT_SPIN_MAX    0.95
MOT_SPIN_MIN    0.15

SERVO1_FUNCTION     0
SERVO2_FUNCTION     0
SERVO3_FUNCTION     0
SERVO4_FUNCTION     0
SERVO5_FUNCTION     0
SERVO6_FUNCTION     0
SERVO7_FUNCTION     0
SERVO8_FUNCTION     0
SERVO9_FUNCTION     0
SERVO10_FUNCTION    33
SERVO10_MAX         1950
SERVO10_MIN         1500
SERVO10_REVERSED    0
SERVO10_TRIM        1750
SERVO11_FUNCTION    34
SERVO11_MAX         1950
SERVO11_MIN         1500
SERVO11_REVERSED    0
SERVO11_TRIM        1750
SERVO12_FUNCTION    35
SERVO12_MAX         1950
SERVO12_MIN         1500
SERVO12_REVERSED    0
SERVO12_TRIM        1750
SERVO13_FUNCTION    36
SERVO13_MAX         1950
SERVO13_MIN         1500
SERVO13_REVERSED    0
SERVO13_TRIM        1750
SERVO14_FUNCTION    0
SERVO15_FUNCTION    0
SERVO16_FUNCTION    0
EOF


OFFSETLAT=0.00007
OFFSETLON=0
OFFSETALT=0
OFFSETHEAD=0

# Launch autotest python script
python Tools/autotest/run_auto_follow.py --binary=$COPTER --home="$HOMELAT,$HOMELONG,$HOMEALT,$HEADING" --offset="$OFFSETLAT,$OFFSETLON,$OFFSETALT,$OFFSETHEAD" --json-ip=$JSON_IP --gcs-ip=$GCS_IP &

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

