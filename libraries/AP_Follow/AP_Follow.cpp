/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Follow.h"
#include <ctype.h>
#include <stdio.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
// #include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    3000    // position estimate timeout after 1 second
#define AP_FOLLOW_SYSID_TIMEOUT_MS 10000 // forget sysid we are following if we have not heard from them in 10 seconds

#define AP_FOLLOW_OFFSET_TYPE_NED       0   // offsets are in north-east-down frame
#define AP_FOLLOW_OFFSET_TYPE_RELATIVE  1   // offsets are relative to lead vehicle's heading

#define AP_FOLLOW_ALTITUDE_TYPE_RELATIVE  1 // relative altitude is used by default   

#define AP_FOLLOW_POS_P_DEFAULT 0.1f    // position error default gain P
#define AP_FOLLOW_POS_D_DEFAULT 0.001f  // position error default gain D

#define AP_FOLLOW_MAX_SPEED 1250        // Default max speed allowed for following, in cm/s (=45 km/h)

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_FOLLOW_ALT_TYPE_DEFAULT 0
#else
#define AP_FOLLOW_ALT_TYPE_DEFAULT AP_FOLLOW_ALTITUDE_TYPE_RELATIVE
#endif

AP_Follow *AP_Follow::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_Follow::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: Follow enable/disable
    // @Description: Enabled/disable following a target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Follow, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // 2 is reserved for TYPE parameter

    // @Param: _SYSID
    // @DisplayName: Follow target's mavlink system id
    // @Description: Follow target's mavlink system id
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_SYSID", 3, AP_Follow, _sysid, 2),

    // 4 is reserved for MARGIN parameter

    // @Param: _DIST_MAX
    // @DisplayName: Follow distance maximum
    // @Description: Follow distance maximum.  targets further than this will be ignored
    // @Units: m
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max, 100),

    // @Param: _OFS_TYPE
    // @DisplayName: Follow offset type
    // @Description: Follow offset type
    // @Values: 0:North-East-Down, 1:Relative to lead vehicle heading
    // @User: Standard
    AP_GROUPINFO("_OFS_TYPE", 6, AP_Follow, _offset_type, AP_FOLLOW_OFFSET_TYPE_NED),

    // @Param: _OFS_X
    // @DisplayName: Follow offsets in meters north/forward
    // @Description: Follow offsets in meters north/forward.  If positive, this vehicle fly ahead or north of lead vehicle.  Depends on FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Y
    // @DisplayName: Follow offsets in meters east/right
    // @Description: Follow offsets in meters east/right.  If positive, this vehicle will fly to the right or east of lead vehicle.  Depends on FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Z
    // @DisplayName: Follow offsets in meters down
    // @Description: Follow offsets in meters down.  If positive, this vehicle will fly below the lead vehicle
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_OFS", 7, AP_Follow, _offset, 0),

#if !(APM_BUILD_TYPE(APM_BUILD_Rover))
    // @Param: _YAW_BEHAVE
    // @DisplayName: Follow yaw behaviour
    // @Description: Follow yaw behaviour
    // @Values: 0:None,1:Face Lead Vehicle,2:Same as Lead vehicle,3:Direction of Flight
    // @User: Standard
    AP_GROUPINFO("_YAW_BEHAVE", 8, AP_Follow, _yaw_behave, 1),
#endif

    // @Param: _POS_P
    // @DisplayName: Follow position error P gain
    // @Description: Follow position error P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 0.01 1.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POS_", 9, AP_Follow, AC_P),

#if !(APM_BUILD_TYPE(APM_BUILD_Rover)) 
    // @Param: _ALT_TYPE
    // @DisplayName: Follow altitude type
    // @Description: Follow altitude type
    // @Values: 0:absolute, 1:relative
    // @User: Standard
    AP_GROUPINFO("_ALT_TYPE", 10, AP_Follow, _alt_type, AP_FOLLOW_ALT_TYPE_DEFAULT),
#endif

    // @Param: _GPSS_REQ
    // @DisplayName: Follow Mode's minimal GPS status requirement
    // @Description: Minimal GPS status requirement accepted to enter Follow Mode or "Landing_on_target" mode. Ex: Value of 6 means you need rtk fix to be allowed to do Following.
    // @Values: 3:GPS-Fix, 6:RTK-Fix
    // @User: Standard
    AP_GROUPINFO("_GPSS_REQ", 12, AP_Follow, _gpss_req, 3),

    // @Param: _SPD_CMS
    // @DisplayName: Follow Mode's maximum speed
    // @Description: Follow Mode's maximum horizontal speed in cm/s (instead of pos_control->get_max_speed_xy_cms)
    // @Values: 100 3500
    // @User: Standard
    AP_GROUPINFO("_SPD_CMS", 13, AP_Follow, _spd_cms, AP_FOLLOW_MAX_SPEED),

    // @Param: _HD_ERR_D
    // @DisplayName: Target's max heading error
    // @Description: Target's max heading error relative to its velocity vector, in degrees
    // @Values: 5 90
    // @User: Standard
    AP_GROUPINFO("_HD_ERR_D", 14, AP_Follow, _hd_err_d, 30),

    // @Param: POS_D
    // @DisplayName: D gain for position controller
    // @Description: D gain for position controller
    // @Values: 0 1
    // @User: Standard
    AP_GROUPINFO("_POS_D", 15, AP_Follow, _d_pos, AP_FOLLOW_POS_D_DEFAULT), //10x smaller than default P gain to start with

    // @Param: _DELAY
    // @DisplayName: Manual GPS delay
    // @Description: Manual GPS delay to be set by the user, based on user's feeling of how close the drone is from the target
    // @Values: 
    // @User: Standard
    AP_GROUPINFO("_DELAY", 16, AP_Follow, _gps_delay, 160),

    AP_GROUPEND
};

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Follow::AP_Follow() :
        _p_pos(AP_FOLLOW_POS_P_DEFAULT)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// restore offsets to zero if necessary, should be called when vehicle exits follow mode
void AP_Follow::clear_offsets_if_required()
{
    if (_offsets_were_zero) {
        _offset.set(Vector3f());
    }
    _offsets_were_zero = false;
}

// get target's estimated location
bool AP_Follow::get_target_location_and_velocity(Location &loc, Vector3f &vel_ned) 
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // calculate time since last actual position update
    const float dt = (AP_HAL::millis() - _last_location_update_ms) * 0.001f;

    // Estimate acceleration (based on last and second to last velocities)
    float dt_accel = (_last_location_update_ms - _last_last_location_update_ms) * 0.001f; // sec
    get_acceleration_ned(_target_accel_ned, dt_accel);

    // @TODO Add message to test dt_accel (makes sense??)

    // get target's velocity estimate using time since last update
    if (!get_velocity_ned(vel_ned, dt)) {
        return false;
    }

    // Project the current target position based on the last received target position
    Location last_loc = _target_location;
    last_loc.offset(vel_ned.x * dt, vel_ned.y * dt);
    last_loc.alt -= vel_ned.z * 100.0f * dt; // convert m/s to cm/s, multiply by dt.  minus because NED

    // Add offset caused by GPS delay (time it takes for the GPS on the target to make it's correction with
    // the RTK (processing time), then sends it to the target pixhawk's EKF to estimate the true position,
    // and then send it through 1 telemetry link to the drone follower.
    last_loc.offset(vel_ned.x * _gps_delay/1000, vel_ned.y * _gps_delay/1000);

    // return latest position estimate
    loc = last_loc;

    // Update previous target's velocity
    // update_target_velocity_prev(_target_velocity_prev_ned, _target_velocity_ned);
    _target_velocity_prev_ned = _target_velocity_ned;
    _last_last_location_update_ms = _last_location_update_ms;

    return true;
    
}

// get distance vector to target (in meters) and target's velocity all in NED frame
bool AP_Follow::get_target_dist_and_vel_ned(Vector3f &dist_ned, Vector3f &dist_with_offs, Vector3f &vel_ned)
{
    // get our location
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        clear_dist_and_bearing_to_target();
         return false;
    }

    // get target location and velocity
    Location target_loc;
    Vector3f veh_vel;
    if (!get_target_location_and_velocity(target_loc, veh_vel)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // change to altitude above home if relative altitude is being used
    if (target_loc.relative_alt == 1) {
        current_loc.alt -= AP::ahrs().get_home().alt;
    }

    // calculate difference
    const Vector3f dist_vec = current_loc.get_distance_NED(target_loc);

    // fail if too far
    if (is_positive(_dist_max.get()) && (dist_vec.length() > _dist_max)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // initialize offsets from distance vector if required
    init_offsets_if_required(dist_vec);

    // get offsets
    Vector3f offsets;
    if (!get_offsets_ned(offsets)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // calculate results
    dist_ned = dist_vec;
    dist_with_offs = dist_vec + offsets;
    vel_ned = veh_vel;

    // record distance and heading for reporting purposes
    if (is_zero(dist_with_offs.x) && is_zero(dist_with_offs.y)) {
        clear_dist_and_bearing_to_target();
    } else {
        _dist_to_target = safe_sqrt(sq(dist_with_offs.x) + sq(dist_with_offs.y));
        _bearing_to_target = degrees(atan2f(dist_with_offs.y, dist_with_offs.x));
    }

    return true;
}

// get target's heading in degrees (0 = north, 90 = east)
bool AP_Follow::get_target_heading_deg(float &heading) const
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_heading_update_ms == 0) || (AP_HAL::millis() - _last_heading_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // return latest heading estimate
    heading = _target_heading;
    return true;
}

// handle mavlink messages
void AP_Follow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!_enabled) {
        return;
    }

    // skip our own messages
    if (msg.sysid == mavlink_system.sysid) {
        return;
    }

    // skip message if not from our target
    if (_sysid != 0 && msg.sysid != _sysid) {
        if (_automatic_sysid) {
            // maybe timeout who we were following...
            if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_SYSID_TIMEOUT_MS)) {
                _sysid.set(0);
            }
        }
        return;
    }

    // decode global-position-int message
    bool mavlink_msg_updated = false;

    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            // decode message
            mavlink_global_position_int_t packet;
            mavlink_msg_global_position_int_decode(&msg, &packet);

            // ignore message if lat and lon are (exactly) zero
            if ((packet.lat == 0 && packet.lon == 0)) {
                return;
            }

            _target_location.lat = packet.lat;
            _target_location.lng = packet.lon;

            // select altitude source based on FOLL_ALT_TYPE param 
            if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE) {
                // above home alt
                _target_location.set_alt_cm(packet.relative_alt / 10, Location::AltFrame::ABOVE_HOME);
            } else {
                // absolute altitude
                _target_location.set_alt_cm(packet.alt / 10, Location::AltFrame::ABSOLUTE);
            }

            _target_velocity_ned.x = packet.vx * 0.01f; // velocity north
            _target_velocity_ned.y = packet.vy * 0.01f; // velocity east
            _target_velocity_ned.z = packet.vz * 0.01f; // velocity down

            // get a local timestamp with correction for transport jitter
            _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.time_boot_ms, AP_HAL::millis());
            if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
                _target_heading = packet.hdg * 0.01f;   // convert centi-degrees to degrees
                _last_heading_update_ms = _last_location_update_ms;
            }
            // initialize _sysid if zero to sender's id
            if (_sysid == 0) {
                _sysid.set(msg.sysid);
                _automatic_sysid = true;
            }
            mavlink_msg_updated = true;
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            // decode message
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(&msg, &packet);

            _target_gps_fix_type = packet.fix_type;

            // get a local timestamp with correction for transport jitter
            _last_gps_update_ms = _jitter.correct_offboard_timestamp_msec(packet.time_usec/1000, AP_HAL::millis());
            
            break;
        }
            // @TODO Use target acceleration to add feedforward to the control algorithm

        // case MAVLINK_MSG_ID_FOLLOW_TARGET:
        // {
        //     // decode message
        //     mavlink_follow_target_t packet;
        //     mavlink_msg_follow_target_decode(&msg, &packet);
        //     // ignore message if lat and lon are (exactly) zero
        //     if ((packet.lat == 0 && packet.lon == 0)) {
        //         return;
        //     }
        //     // require at least position
        //     if ((packet.est_capabilities & (1<<0)) == 0) {
        //         return;
        //     }
        //     Location new_loc = _target_location;
        //     new_loc.lat = packet.lat;
        //     new_loc.lng = packet.lon;
        //     new_loc.set_alt_cm(packet.alt*100, Location::AltFrame::ABSOLUTE);
        //     // FOLLOW_TARGET is always AMSL, change the provided alt to
        //     // above home if we are configured for relative alt
        //     if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE &&
        //         !new_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        //         return;
        //     }
        //     _target_location = new_loc;
        //     if (packet.est_capabilities & (1<<1)) {
        //         _target_velocity_ned.x = packet.vel[0]; // velocity north
        //         _target_velocity_ned.y = packet.vel[1]; // velocity east
        //         _target_velocity_ned.z = packet.vel[2]; // velocity down
        //     } else {
        //         _target_velocity_ned.zero();
        //     }
        //     // get a local timestamp with correction for transport jitter
        //     _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.timestamp, AP_HAL::millis());
        //     if (packet.est_capabilities & (1<<3)) {
        //         Quaternion q{packet.attitude_q[0], packet.attitude_q[1], packet.attitude_q[2], packet.attitude_q[3]};
        //         float r, p, y;
        //         q.to_euler(r,p,y);
        //         _target_heading = degrees(y);
        //         _last_heading_update_ms = _last_location_update_ms;
        //     }
        //     // initialise _sysid if zero to sender's id
        //     if (_sysid == 0) {
        //         _sysid.set(msg.sysid);
        //         _automatic_sysid = true;
        //     }
        //     updated = true;
        //     break;
        // }
    }

    // ADDED FOR TESTING
    uint32_t tnow = AP_HAL::millis();
    if (mavlink_msg_updated) // && !_updated_last)
    {
        _time_between_updates_ms = tnow - _time_since_last_update;
        _time_since_last_update = tnow;
        _num_of_msg_received += 1;
    }
    
    if (mavlink_msg_updated)
    {
        // get estimated location and velocity
        Location loc_estimate{};
        Vector3f vel_estimate;
        Vector3f dist_vec;      // vector to lead vehicle
        Vector3f dist_vec_offs; // vector to lead vehicle + offset
        Vector3f vel_of_target; // velocity of lead vehicle
        // float target_speed_bearing;
        UNUSED_RESULT(get_target_location_and_velocity(loc_estimate, vel_estimate));
        UNUSED_RESULT(get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target));

        // Log lead's estimated vs reported position
        // @LoggerMessage: FOLL
        // @Description: Follow library diagnostic data
        // @Field: Tus: Time since system startup
        // @Field: Lat: Target latitude
        // @Field: Lon: Target longitude
        // @Field: Alt: Target absolute altitude
        // @Field: VelN: Target earth-frame velocity, North
        // @Field: VelE: Target earth-frame velocity, East
        // @Field: VelD: Target earth-frame velocity, Down
        // @Field: LatE: Vehicle latitude
        // @Field: LonE: Vehicle longitude
        // @Field: AltE: Vehicle absolute altitude
        // @Field: DN: Distance vector with offset, North
        // @Field: DE: Distance vector with offset, East
        // @Field: DD: Distance vector with offset, Down
        // @Field: THD: Target's heading, 0° (north) to 359° clockwise
        AP::logger().WriteStreaming("FOLL",
                                               "TimeUS,Lat,Lon,Alt,VelN,VelE,VelD,LatE,LonE,AltE,DN,DE,DD",  // labels
                                               "sDUmnnnDUmmmm",    // units
                                               "F--B000--B000",    // mults
                                               "QLLifffLLifff",    // fmt
                                               AP_HAL::micros64(),
                                               _target_location.lat, // Dernière position reçue par msg mavlink
                                               _target_location.lng,
                                               _target_location.alt,
                                               (double)_target_velocity_ned.x, // Vitesse reçue par msg mavlink
                                               (double)_target_velocity_ned.y,
                                               (double)_target_velocity_ned.z,
                                               loc_estimate.lat, // Position projetée, sachant la vitesse
                                               loc_estimate.lng,
                                               loc_estimate.alt,
                                               (double)dist_vec_offs.x, // Distance entre véhicule et target WITH offset (doit tendre vers 0)
                                               (double)dist_vec_offs.y,
                                               (double)dist_vec_offs.z
                                               //(double)_target_heading
                                               // (double)target_speed_bearing
                                               );
    }
}

// get velocity estimate in m/s in NED frame using dt since last update
bool AP_Follow::get_velocity_ned(Vector3f &vel_ned, float dt) const
{
    // @TODO Re-enable the use of target_accel_ned
    vel_ned = _target_velocity_ned ; //+ (_target_accel_ned * dt);
    // gcs().send_text(MAV_SEVERITY_INFO, "Accel: %4.3f m/s/s", _target_accel_ned.length());
    return true;
}

// get accel estimate in m/s/s in NED frame using previous target velocity
bool AP_Follow::get_acceleration_ned(Vector3f &accel_ned, float dt) const
{
    if (dt >= 0.0001)
    {
        accel_ned = (_target_velocity_ned - _target_velocity_prev_ned) / dt;
        return true;
    }
    return false;
}

void AP_Follow::update_target_velocity_prev(Vector3f &vel_prev_ned, Vector3f &vel_ned)
{
    vel_prev_ned = vel_ned;
}

// initialise offsets to provided distance vector to other vehicle (in meters in NED frame) if required
void AP_Follow::init_offsets_if_required(const Vector3f &dist_vec_ned)
{
    // return immediately if offsets have already been set
    if (!_offset.get().is_zero()) {
        return;
    }
    _offsets_were_zero = true;

    float target_heading_deg;
    if ((_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE) && get_target_heading_deg(target_heading_deg)) {
        // rotate offsets from north facing to vehicle's perspective
        _offset.set(rotate_vector(-dist_vec_ned, -target_heading_deg));
        gcs().send_text(MAV_SEVERITY_INFO, "Relative follow offset loaded");
    } else {
        // initialise offset in NED frame
        _offset.set(-dist_vec_ned);
        // ensure offset_type used matches frame of offsets saved
        _offset_type.set(AP_FOLLOW_OFFSET_TYPE_NED);
        gcs().send_text(MAV_SEVERITY_INFO, "N-E-D follow offset loaded");
    }
}

// get offsets in meters in NED frame
bool AP_Follow::get_offsets_ned(Vector3f &offset) const
{
    const Vector3f &off = _offset.get();

    // if offsets are zero or type is NED, simply return offset vector
    if (off.is_zero() || (_offset_type == AP_FOLLOW_OFFSET_TYPE_NED)) {
        offset = off;
        return true;
    }

    // COMMENTED OUT FOR TESTING CODE BELOW
    // // offset type is relative, exit if we cannot get vehicle's heading
    // float target_heading_deg;
    // if (!get_target_heading_deg(target_heading_deg)) {
    //     return false;
    // }

    // NEW ALGO: USE VELOCITY BEARING AND HEADING TO CALCULATE OFFSET
    // FOR TESTING
    // Offset type is relative
    float target_heading_deg;
    float target_velocity;
    float target_speed_bearing = get_bearing_cd(Vector2f{}, _target_velocity_ned.xy())/100; // 0 to 360 deg
    target_velocity = safe_sqrt( sq(_target_velocity_ned.x) + sq(_target_velocity_ned.y) );

    if (get_target_heading_deg(target_heading_deg)) // If able to get target's heading
    {   
        if ( target_velocity >= 1.0 ) // Only calculate new heading if speed is significant enough
        {  
            float heading_offset = abs(target_speed_bearing - target_heading_deg);
            if ( heading_offset > (360 - 20) ) { heading_offset = 360 - heading_offset; } // Deal with "near-360-deg" zone
            
            if ( heading_offset > 20 ) // If large gap between target's heading and velocity bearing, trust only velocity heading!
            {
                target_heading_deg = target_speed_bearing;
            }
            else // Else, use both heading and velocity bearing with different weights
            {  
                float target_speed_bearing_weight = target_velocity/3; // Weight function of speed
                if (target_speed_bearing_weight >= 1) { target_speed_bearing_weight = 1; }
                target_heading_deg = target_speed_bearing * target_speed_bearing_weight + target_heading_deg * (1 - target_speed_bearing_weight);
            }
        }
    }
    else // Unable to get target's heading
    {
        // If can't get heading but speed is high enough, use velocity vector to estimate heading
        if ( target_velocity > 1 )
        {
            target_heading_deg = target_speed_bearing;
        }
        else { return false; } // exit if we cannot get vehicle's heading (either with heading or velocity bearing)
    }
    // END OF TEST

    // rotate offsets from vehicle's perspective to NED
    offset = rotate_vector(off, target_heading_deg); // Only when offset type is relative
    return true;
}

// rotate 3D vector clockwise by specified angle (in degrees)
Vector3f AP_Follow::rotate_vector(const Vector3f &vec, float angle_deg) const
{
    // rotate roll, pitch input from north facing to vehicle's perspective
    const float cos_yaw = cosf(radians(angle_deg));
    const float sin_yaw = sinf(radians(angle_deg));
    return Vector3f((vec.x * cos_yaw) - (vec.y * sin_yaw), (vec.y * cos_yaw) + (vec.x * sin_yaw), vec.z);
}

// set recorded distance and bearing to target to zero
void AP_Follow::clear_dist_and_bearing_to_target()
{
    _dist_to_target = 0.0f;
    _bearing_to_target = 0.0f;
}

// get target's estimated location and velocity (in NED), with offsets added
bool AP_Follow::get_target_location_and_velocity_ofs(Location &loc, Vector3f &vel_ned)
{
    Vector3f ofs;
    if (!get_offsets_ned(ofs) ||
        !get_target_location_and_velocity(loc, vel_ned)) {
        return false;
    }
    // apply offsets
    loc.offset(ofs.x, ofs.y);
    loc.alt -= ofs.z*100;
    return true;
}

// return true if we have a target
bool AP_Follow::have_target(void) const
{
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

namespace AP {

AP_Follow &follow()
{
    return *AP_Follow::get_singleton();
}

}
