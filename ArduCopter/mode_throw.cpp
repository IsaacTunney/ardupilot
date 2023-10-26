#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
#include <algorithm>
#include <set>

// @TODO Change all the speeds related to landing on moving vehicle for a single parameter
// Instead of having loit_speed, foll_spd and wpnav_speed, use only one: wpnav_speed

// Initialise land controller
bool ModeThrow::init(bool ignore_checks)
{   
    // For now, do not use rangefinder when landing on vehicle
    if (g.land_use_rf == 1)
    {
        use_rangefinder = true;
    }
    else
    {
        use_rangefinder = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "Not using rangefinder.");
    }

    // Test
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Commit height param set to: %4.2f m.", (double)g.land_commit_hgt_m);

    // Check (only once) if we have GPS to decide on which landing sequence to execute
    control_position = copter.position_ok();
    // **Éventuellement, seulement accepter le landing AVEC GPS et RANGEFINDER!

    // Set horizontal/vertical speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // Initialize the horizontal and vertical position controller
    if (control_position && !pos_control->is_active_xy())
    {
        pos_control->init_xy_controller();
    }
    if (!pos_control->is_active_z())
    {
        pos_control->init_z_controller();
    }

    state = INIT;
    landingOnVehicle_state = FOLLOWING;
    landingOnVehicle_previousState = FOLLOWING;
    switchLandingState = false;
    lsmCount = 1; // Landing state machine iterator
    runCount = 0; // Main run loop iterator

    shutdown_motors = false;
    activate_rvt_countertorque = false;
    activate_rvt = false;
    land_pause = false;

    rvt_duration = 6000;                                                                      // Duration of rvt after landing, milliseconds
    countdown_duration = (70.0 - (double)g.land_shutdown_cm) / (double)g.land_speed * 1000.0; // milliseconds
    gcs().send_text(MAV_SEVERITY_INFO, "Countdown duration (const): %4.2d milliseconds", (int)countdown_duration);
    dropping_max_duration = 500; // milliseconds

    copter.ap.land_repo_active = false; // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.prec_land_active = false; // this will be set true if prec land is later active
    auto_yaw.set_mode(AUTO_YAW_HOLD);   // initialize yaw

#if AP_FENCE_ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
#endif

#if PRECISION_LANDING == ENABLED
    // initialize precland state machine
    copter.precland_statemachine.init();
#endif

#if !(RANGEFINDER_ENABLED == ENABLED)
    gcs().send_text(MAV_SEVERITY_WARNING, "Rangefinder not enabled!"); // Set in the ardupilot code directly, NOT in parameters
#endif

    // Make sure wpnav_speed_dn is higher or equal to land_speed, otherwise there will be accel Z spike when vel_desired > wpnav_speed_dn
    float wpnav_speed_dn = wp_nav->get_default_speed_down();
    if (g.land_speed > wpnav_speed_dn)
    {
        gcs().send_text(MAV_SEVERITY_ALERT, "EXITING: Land speed (%4.2f cm/s) is higher than wpnav speed dn (%4.2f cm/s)", double(g.land_speed), wpnav_speed_dn);
        return false;
    }
   
    if (g.land_type == VEHICLE)
    {
        if (!g2.follow.enabled())
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Follow not enabled. Set FOLL_ENABLE = 1");
            return false;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Max Following speed: %4d cm/s", (int)g2.follow.get_max_speed_cms());
        gcs().send_text(MAV_SEVERITY_INFO, "Gps requirement: %4d", (int)g2.follow.get_gpss_req());
        gcs().send_text(MAV_SEVERITY_INFO, "Max target heading error: %4d deg", (int)g2.follow.get_heading_err_deg());

        // Check GPS status requirements
        if (AP::gps().status(0) < g2.follow.get_gpss_req()) // Check Follower GPS status
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Follower GPS Status: %2d. Requirements not satisfied", AP::gps().status(0));
            return false;
        }

        if (g2.follow.get_target_gps_fix_type() < g2.follow.get_gpss_req()) // Check Follower GPS status
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Target GPS Status: %2d. Requirements not satisfied", g2.follow.get_target_gps_fix_type());
            return false;
        }

        allow_following = true;
        target_was_acquired = true;
        msg_target_reached_sent = false;
        time_last_ms = 0;
        sentPitchToZeroMsgOnce = false;
        sentCommitMsgOnce = false;
        sentCancelMsgOnce = false;

        return ModeGuided::init(ignore_checks);
    }
    else
    {
        return true;
    }
}

// Perform cleanup required when leaving mode
void ModeThrow::exit()
{
    g2.follow.clear_offsets_if_required();
    shutdown_motors = false;
    activate_rvt_countertorque = false;
    activate_rvt = false;
    attitude_control->landing_controller_setRVT(shutdown_motors, activate_rvt, activate_rvt_countertorque, g.land_rvt_pwm);
    gcs().send_text(MAV_SEVERITY_WARNING, "Exit mode landing");
}

// land_run - runs the land controller at around 200 to 400 Hz
// flightmode->run() is updated in the fast_loop of Copter.cpp
void ModeThrow::run()
{
    if (use_rangefinder)
    {
        height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;
        for (int c = 8; c >= 0; c--) // Managing rangefinder queue
        {
            RFdistance_buffer[c + 1] = RFdistance_buffer[c];
        }
        RFdistance_buffer[0] = height_above_ground_cm;
    }

    // Change the structure below because if loose pos_control but land_type is vehicle, drone will land on the spot
    // instead of waiting.
    if (control_position)
    {
        if (g.land_type == VEHICLE)
        {
            landing_on_moving_vehicle_run();
        }
        else
        {
            landing_with_gps_run();
        }
    }
    else
    {
        landing_without_gps_run();
    }
    lsmCount++;
    runCount++;
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

// LAND CONTROLLER WITH GPS - Horizontal position controlled with loiter controller
// Frequency: 100hz or more
void ModeThrow::landing_with_gps_run()
{
    // Landing state machine:
    run_landing_state_machine();

    // Safety checks
    // Disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
        gcs().send_text(MAV_SEVERITY_INFO, "Ground idle detected; Disarming drone...");
    }

    // Flight controller during landing sequence:
    if (is_disarmed_or_landed())
    {
        make_safe_ground_handling();
        loiter_nav->init_target();
    }
    else // still flying
    {
        loiter_nav->clear_pilot_desired_acceleration();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED); // set motors to full range
        land_run_horiz_and_vert_control(land_pause);
        
        // Set boolean flag in the AP_Motors library to indicate to the motors_outputs when to activate reverse thrust
        attitude_control->landing_controller_setRVT(shutdown_motors, activate_rvt, activate_rvt_countertorque, g.land_rvt_pwm);

    }

}

// LAND CONTROLLER WITH NO GPS - Pilot controls roll and pitch angles
// Frequency: 100hz or more
void ModeThrow::landing_without_gps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0;

    process_pilot_inputs(target_roll, target_pitch, target_yaw_rate);

    // Landing state machine:
    run_landing_state_machine();

    // POST-LANDING SAFETY CHECKS: Landing detector
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
        gcs().send_text(MAV_SEVERITY_INFO, "Ground idle detected; Disarming drone...");
    }

    // Flight controller during landing sequence:
    if (is_disarmed_or_landed())
    {
        make_safe_ground_handling();
    }
    else // still flying
    {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        land_run_vertical_control(land_pause);
        
        // Call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        
        // Set boolean flag in the AP_Motors library to indicate to the motors_outputs when to activate reverse thrust
        attitude_control->landing_controller_setRVT(shutdown_motors, activate_rvt, activate_rvt_countertorque, g.land_rvt_pwm);

    }

}

// LAND CONTROLLER WITH GPS FOR LANDING ON MOVING TARGET - Guided mode commands for horizontal control and standard z-controller for altitude
// Frequency: 100hz or more
void ModeThrow::landing_on_moving_vehicle_run()
{
    // Safety checks
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED); // Disarm when the landing detector says we've landed
        gcs().send_text(MAV_SEVERITY_INFO, "Ground idle detected; Disarming drone...");
    }
    if (is_disarmed_or_landed())
    {
        make_safe_ground_handling();
        return;
    }

    // High-level state machine for landing on moving vehicle:
    switch (landingOnVehicle_state)
    {
    case FOLLOWING:
        if (switchLandingState == true)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "LANDING STATE: FOLLOWING");
            switchLandingState = false;
        }

        follow_target_3D();

        if (target_was_acquired == true && target_over_vehicle_has_been_reached())
        {
            landingOnVehicle_state = READY_FOR_DESCENT;
        }
        break;

    case READY_FOR_DESCENT: // Drone is within GPS landing area
        if (switchLandingState == true)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "LANDING STATE: READY FOR DESCENT");
            switchLandingState = false;
        }

        follow_target_3D();

        if (user_has_allowed_landing_on_vehicle())
        {
            landingOnVehicle_state = LANDING;
        } // With RC switch?
        break;

    case LANDING:
        if (switchLandingState == true)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "LANDING STATE: LANDING");
            switchLandingState = false;
        }

        if (g.land_mnvr == PITCH_TO_ZERO)
        {
            follow_target_2D_pitch_to_zero();
        }
        else
        {
            follow_target_2D();
        }
        // run_landing_state_machine();
        // land_run_vertical_control(land_pause);
        attitude_control->landing_controller_setRVT(shutdown_motors, activate_rvt, activate_rvt_countertorque, g.land_rvt_pwm);
        break;

    default:
        break;
    }

    // Managing the switch of landing states:
    if (landingOnVehicle_previousState != landingOnVehicle_state)
    {
        switchLandingState = true;
    }
    landingOnVehicle_previousState = landingOnVehicle_state;
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

void ModeThrow::follow_target_2D_pitch_to_zero()
{
    // if (is_disarmed_or_landed())
    // {
    //     make_safe_ground_handling();
    //     return;
    // }
    // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    bool use_yaw = false;
    float yaw_cd = 0.0f;
    float target_heading = 0.0f;
    Vector2f desired_velocity_ne_cms; // 2D Vector to be sent to velocity controller
    Vector3f dist_vec;                // vector to lead vehicle
    Vector3f dist_vec_offs;           // vector to lead vehicle + offset
    Vector3f vel_of_target;           // velocity of lead vehicle
    Vector3f vel_of_follower;

    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target) && allow_following)
    {
        dist_vec_last = dist_vec; // keep track of 

        target_was_acquired = true;

        // Convert dist_vec_offs to cm in NE:
        const Vector2f dist_vec_offs_ne(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f);


        // Position controller: PD controller with Feedforward
        // const float kp = g2.follow.get_pos_p().kP();
        float kp;
        float kp_static = g2.follow.get_pos_p().kP();
        float kp_100kmph  = g2.follow.get_pos_p_100kmph();

        if (vel_of_target.xy().length() <= 0.1) //m/s
        {
            kp = kp_static;
        }
        else if ((vel_of_target.xy().length() ) >= 100/3.6) //m/s
        {
            kp = kp_100kmph;
        }
        else // If in-between 0 and 100 km/h, interpolate
        {
            kp = (kp_100kmph - kp_static) * (vel_of_target.xy().length()) / (100/3.6) + kp_static;
        }
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "Gain kp: %4.2f.", kp);

        const float kd = g2.follow.get_pos_d();

        if (ahrs.get_velocity_NED(vel_of_follower)) // Ground speed, m/s
        {
            Vector3f vel_of_foll_neu_cms(vel_of_follower.x * 100.0f, vel_of_follower.y * 100.0f, -vel_of_follower.z * 100.0f);
            desired_velocity_ne_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_offs_ne.x * kp) - ((vel_of_foll_neu_cms.x - vel_of_target.x * 100) * kd);
            desired_velocity_ne_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_offs_ne.y * kp) - ((vel_of_foll_neu_cms.y - vel_of_target.y * 100) * kd);
        }
        else
        {
            desired_velocity_ne_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_offs_ne.x * kp);
            desired_velocity_ne_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_offs_ne.y * kp);
        }

        // Scale desired velocity to stay within horizontal speed limit:
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_ne_cms.x) + sq(desired_velocity_ne_cms.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > g2.follow.get_max_speed_cms()))
        {
            const float scalar_xy = g2.follow.get_max_speed_cms() / desired_speed_xy;
            desired_velocity_ne_cms.x *= scalar_xy;
            desired_velocity_ne_cms.y *= scalar_xy;
            desired_speed_xy = g2.follow.get_max_speed_cms();
        }

        // Calculate vehicle heading:
        target_heading = 0.0f;
        bool got_target_heading;
        got_target_heading = g2.follow.get_target_heading_deg(target_heading);
        
        // Calculate yaw command
        switch (g2.follow.get_yaw_behave())
        {
        case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE:
        {
            if (dist_vec.xy().length_squared() > 1.0)
            {
                yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE:
        {
            if (got_target_heading)
            {
                yaw_cd = target_heading * 100.0f;
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT:
        { // Direction du vecteur vitesse

            // If we set YAW_BEHAVE_DIR_OF_FLIGHT but vehicle is not moving yet, use YAW_BEHAVE_SAME_AS_LEAD_VEHICLE instead
            // This way, drone already points in the right direction when vehicle starts moving
            // if (desired_velocity_neu_cms.xy().length() > (150.0))
            if (vel_of_target.xy().length() > 2.0 && vel_of_follower.xy().length() > 2.0)
            {
                yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_ne_cms);
                use_yaw = true;
            }
            else // Do same thing as for case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE
            {
                if (got_target_heading)
                {
                    yaw_cd = target_heading * 100.0f;
                    use_yaw = true;
                }
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_NONE:
        default:
            // do nothing
            break;
        }

        // Check GPS status
        if (AP::gps().status(0) < g2.follow.get_gpss_req() || g2.follow.get_target_gps_fix_type() < g2.follow.get_gpss_req()) // Check Follower GPS status
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS Status requirements not satisfied.");
            // Reset à 0 toutes les commandes Guided
            desired_velocity_ne_cms.zero();
            use_yaw = false;
            yaw_cd = 0.0f;
            allow_following = false; // Set flag to block the pursuit of the landing sequence after a bug
        }

        // Check copter's heading against target's velocity heading (should match)
        // Éventuellement, accorder un poids plus important à la direction du vecteur vitesse à mesure que sa magnitude augmente.
        if (safe_sqrt(sq(vel_of_target.x) + sq(vel_of_target.y)) >= 2.0) // Only calculate if speed is significant enough
        {
            target_speed_bearing = get_bearing_cd(Vector2f{}, vel_of_target.xy()) / 100; // 0 to 360 deg
            float heading_offset = abs(target_speed_bearing - target_heading);
            if (heading_offset > (360 - g2.follow.get_heading_err_deg()))
            {
                heading_offset = 360 - heading_offset;
            }
            if (heading_offset > g2.follow.get_heading_err_deg()) // If offset is too large, there is a problem!
            {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Offset too large between heading and velocity vector. Killing Follow Task.");
                desired_velocity_ne_cms.zero();
                use_yaw = false;
                yaw_cd = 0.0f;
                allow_following = false; // Set flag to block the pursuit of the landing sequence after a bug
            }
        }
        else
        {
            target_speed_bearing = 0;
        } // Speed not high enough to determine a proper target speed bearing
    }
    else // Did not find target
    {
        if (target_was_acquired == true)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Did not find target...");
        }
        target_was_acquired = false;

        // @TODO Based on previous data, estimate target state and continue "blind" following for 10 loops
        // ...
        // ...
        // ...
    }

    // Log output at 10hz:
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0))
    {
        log_request = true;
        last_log_ms = now;
    }

    // If we have target lock, check dist_vec.z and decide when to do PTZ maneuver.
    // If we lose target lock, decide if you commit or cancel based on last known dist_vec.z.
    // If dist_vec.z < 1 m, commit to landing (either before or after PTZ, it doesn't matter)
    // Also count how many loops we make without target lock and estimate target state for these loops.
    // Allow for X number of loops to happen before cancelling the sequence.
    if (target_was_acquired)
    {
        // When desired height for pitch-to-zero is reached, switch to attitude control
        if (dist_vec.z <= g.land_ptz_hgt_m) // Positive axis is pointing down (in meters)
        {
            if (sentPitchToZeroMsgOnce==false)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Doing Pitch-to-zero manoeuver");
                gcs().send_text(MAV_SEVERITY_INFO, "Height above target: %4.2f m", dist_vec.z);
                sentPitchToZeroMsgOnce = true;
            }
            attitude_control->input_euler_angle_roll_pitch_yaw(0.0, 0.0, yaw_cd, true);
        }
        // Otherwise, use Guided mode for position control in 2D
        else
        {
            if (runCount % 200 == 0) { gcs().send_text(MAV_SEVERITY_INFO, "Doing normal follow 2D"); }
            ModeGuided::set_velocity_ne(desired_velocity_ne_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);
            ModeGuided::run();
        }
        run_landing_state_machine();
        land_run_vertical_control(land_pause);
    }
    else if (dist_vec_last.z <= g.land_commit_hgt_m) // Target lost, but drone within 1 meter of target in Z-axis
    {
        // If drone was within 1 meter of target in Z-axis, commit to descent and do PTZ right away
        if (sentCommitMsgOnce==false)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Target lost - commiting to landing anyway");
            sentCommitMsgOnce = true;
        }
        attitude_control->input_euler_angle_roll_pitch_yaw(0.0, 0.0, yaw_cd, true);
        run_landing_state_machine();
        land_run_vertical_control(land_pause);
    }
    else // Otherwise, cancel landing sequence and hover in place
    {
        if (sentCancelMsgOnce==false)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Target lost - cancelling sequence");
            sentCancelMsgOnce = true;
        }
        ModeGuided::set_velocity(Vector3f(), use_yaw, yaw_cd, false, 0.0f, false, log_request);
        ModeGuided::run();
        allow_following = false;
        // or 
        // landingOnVehicle_state = FOLLOWING; to make the sequence continue
    }
    // @TODO Add safety if drone gets past 1m below target to stop sequence
    // (drone should never be below target if the sequence is successful)

    // @TODO Add Rangefinder implementation (use it as another condition with gps height?)
}

void ModeThrow::follow_target_3D()
{
    // if (is_disarmed_or_landed())
    // {
    //     make_safe_ground_handling();
    //     return;
    // }
    // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    bool use_yaw = false;
    float yaw_cd = 0.0f;
    float target_heading = 0.0f;
    Vector3f desired_velocity_neu_cms; // Vector to be sent to velocity controller
    Vector3f dist_vec;                 // vector to lead vehicle, m
    Vector3f dist_vec_offs;            // vector to lead vehicle + offset, m
    Vector3f vel_of_target;            // velocity of lead vehicle, m/s
    Vector3f vel_of_follower;

    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target) && allow_following)
    {
        target_was_acquired = true;

        // Convert dist_vec_offs to cm in NEU:
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        horizontal_dist_from_target_with_offset_cm = dist_vec_offs_neu.xy(); // cm

        if (runCount % 500 == 0)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Dist from v-target: x:%4.3f m; y:%4.3f m; z:%4.3f m.", dist_vec_offs.x, dist_vec_offs.y, dist_vec_offs.z);
        }

        // Calculate desired velocity vector in cm/s in NEU:
        // const float kp = g2.follow.get_pos_p().kP();
        float kp;
        float kp_static = g2.follow.get_pos_p().kP();
        float kp_100kmph  = g2.follow.get_pos_p_100kmph();

        if (vel_of_target.xy().length() <= 0.1) //m/s
        {
            kp = kp_static;
        }
        else if ((vel_of_target.xy().length() ) >= 100/3.6) //m/s
        {
            kp = kp_100kmph;
        }
        else // If in-between 0 and 100 km/h, interpolate
        {
            kp = (kp_100kmph - kp_static) * (vel_of_target.xy().length()) / (100/3.6) + kp_static;
        }
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "Gain kp: %4.2f.", kp);

        const float kd = g2.follow.get_pos_d();
        // desired_velocity_neu_cms.x =  (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
        // desired_velocity_neu_cms.y =  (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);
        // desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);
        if (ahrs.get_velocity_NED(vel_of_follower)) // Ground velocity of follower, m/s
        {
            Vector3f vel_of_foll_neu_cms(vel_of_follower.x * 100.0f, vel_of_follower.y * 100.0f, -vel_of_follower.z * 100.0f);
            desired_velocity_neu_cms.x = ( vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp) - ((vel_of_foll_neu_cms.x - vel_of_target.x * 100) * kd);
            desired_velocity_neu_cms.y = ( vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp) - ((vel_of_foll_neu_cms.y - vel_of_target.y * 100) * kd);
            desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);
        }
        else
        {
            desired_velocity_neu_cms.x = ( vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
            desired_velocity_neu_cms.y = ( vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);
            desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);
        }

        // // Add compensation for when follower accelerates downwards, which results in a velocity loss
        // Vector3f accel_of_follower = ahrs.get_accel();
        // float kp_accel = 0.1;
        // desired_velocity_neu_cms.x = desired_velocity_neu_cms.x + accel_of_follower.z * kp_accel;
        // desired_velocity_neu_cms.y = desired_velocity_neu_cms.y + accel_of_follower.z * kp_accel;

        // Scale desired velocity to stay within horizontal speed limit:
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > g2.follow.get_max_speed_cms()))
        {
            const float scalar_xy = g2.follow.get_max_speed_cms() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed_xy = g2.follow.get_max_speed_cms();
        }
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

        // Limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down):
        const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -des_vel_z_max, des_vel_z_max);

        // Limit the velocity for obstacle/fence avoidance:
        copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // Calculate vehicle heading:
        target_heading = 0.0f;
        bool got_target_heading;
        got_target_heading = g2.follow.get_target_heading_deg(target_heading);

        // Calculate yaw command
        switch (g2.follow.get_yaw_behave())
        {
        case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE:
        {
            if (dist_vec.xy().length_squared() > 1.0)
            {
                yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                use_yaw = true;
            }
            break;
        }
        case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE:
        {
            if (got_target_heading)
            {
                yaw_cd = target_heading * 100.0f;
                use_yaw = true;
            }
            break;
        }
        case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT:
        { // Direction du vecteur vitesse

            // If we set YAW_BEHAVE_DIR_OF_FLIGHT but vehicle is not moving yet, use YAW_BEHAVE_SAME_AS_LEAD_VEHICLE instead
            // This way, drone already points in the right direction when vehicle starts moving
            // if (desired_velocity_neu_cms.xy().length() > (150.0))
            if (vel_of_target.xy().length() > 2.0 && vel_of_follower.xy().length() > 2.0)
            {
                yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_neu_cms.xy());
                use_yaw = true;
            }
            else // Do same thing as for case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE
            {
                if (got_target_heading)
                {
                    yaw_cd = target_heading * 100.0f;
                    use_yaw = true;
                }
            }
            break;
        }
        case AP_Follow::YAW_BEHAVE_NONE:
        default:
            // do nothing
            break;
        }

        // Check GPS status
        if (AP::gps().status(0) < g2.follow.get_gpss_req() || g2.follow.get_target_gps_fix_type() < g2.follow.get_gpss_req()) // Check Follower GPS status
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "GPSS req not satisfied. Killing Follow Task.");
            desired_velocity_neu_cms.zero();
            use_yaw = false;
            yaw_cd = 0.0f;
            allow_following = false; // Set flag to block the pursuit of the landing sequence after a bug
        }

        // Check heading against target velocity heading: (should match)
        if (safe_sqrt(sq(vel_of_target.x) + sq(vel_of_target.y)) >= 2.0) // Only calculate if speed is significant enough
        {
            target_speed_bearing = get_bearing_cd(Vector2f{}, vel_of_target.xy()) / 100; // 0 to 360 deg
            float heading_offset = abs(target_speed_bearing - target_heading);
            if (heading_offset > (360 - g2.follow.get_heading_err_deg()))
            {
                heading_offset = 360 - heading_offset;
            }
            if (heading_offset > g2.follow.get_heading_err_deg()) // If offset is too large, there is a problem!
            {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Heading error too large. Killing Follow Task.");
                desired_velocity_neu_cms.zero();
                use_yaw = false;
                yaw_cd = 0.0f;
                allow_following = false; // Set flag to block the pursuit of the landing sequence after a bug
            }
        }
        else
        {
            target_speed_bearing = 0;
        } // Speed not high enough to determine a proper target speed bearing
    }
    else // Did not find target
    {
        if (target_was_acquired == true)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Did not find target...");
        }
        target_was_acquired = false;
    }

    // Log output at 10hz:
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0))
    {
        log_request = true;
        last_log_ms = now;
    }

    // Send velocity commands to Guided Mode:
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);
    ModeGuided::run();
}

void ModeThrow::follow_target_2D()
{
    // if (is_disarmed_or_landed())
    // {
    //     make_safe_ground_handling();
    //     return;
    // }
    // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    bool use_yaw = false;
    float yaw_cd = 0.0f;
    float target_heading = 0.0f;
    Vector2f desired_velocity_ne_cms; // 2D Vector to be sent to velocity controller
    Vector3f dist_vec;                // vector to lead vehicle
    Vector3f dist_vec_offs;           // vector to lead vehicle + offset
    Vector3f vel_of_target;           // velocity of lead vehicle
    Vector3f vel_of_follower;

    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target) && allow_following)
    {

        target_was_acquired = true;

        // Convert dist_vec_offs to cm in NE:
        const Vector2f dist_vec_offs_ne(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f);

        if (runCount % 500 == 0)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Dist from landing target: x:%4.3f m; y:%4.3f m; z:%4.3f m.", dist_vec_offs.x, dist_vec_offs.y, dist_vec.z);
        }

        // Position controller: PD controller with Feedforward
        // const float kp = g2.follow.get_pos_p().kP();
        float kp;
        float kp_static = g2.follow.get_pos_p().kP();
        float kp_100kmph  = g2.follow.get_pos_p_100kmph();

        if (vel_of_target.xy().length() <= 0.1) //m/s
        {
            kp = kp_static;
        }
        else if ((vel_of_target.xy().length() ) >= 100/3.6) //m/s
        {
            kp = kp_100kmph;
        }
        else // If in-between 0 and 100 km/h, interpolate
        {
            kp = (kp_100kmph - kp_static) * (vel_of_target.xy().length()) / (100/3.6) + kp_static;
        }
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "Gain kp: %4.2f.", kp);
        
        const float kd = g2.follow.get_pos_d();

        // desired_velocity_ne_cms.x =  (vel_of_target.x * 100.0f) + (dist_vec_offs_ne.x * kp);
        // desired_velocity_ne_cms.y =  (vel_of_target.y * 100.0f) + (dist_vec_offs_ne.y * kp);

        if (ahrs.get_velocity_NED(vel_of_follower)) // Ground speed, m/s
        {
            Vector3f vel_of_foll_neu_cms(vel_of_follower.x * 100.0f, vel_of_follower.y * 100.0f, -vel_of_follower.z * 100.0f);
            desired_velocity_ne_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_offs_ne.x * kp) - ((vel_of_foll_neu_cms.x - vel_of_target.x * 100) * kd);
            desired_velocity_ne_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_offs_ne.y * kp) - ((vel_of_foll_neu_cms.y - vel_of_target.y * 100) * kd);
        }
        else
        {
            desired_velocity_ne_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_offs_ne.x * kp);
            desired_velocity_ne_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_offs_ne.y * kp);
        }

        // Scale desired velocity to stay within horizontal speed limit:
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_ne_cms.x) + sq(desired_velocity_ne_cms.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > g2.follow.get_max_speed_cms()))
        {
            const float scalar_xy = g2.follow.get_max_speed_cms() / desired_speed_xy;
            desired_velocity_ne_cms.x *= scalar_xy;
            desired_velocity_ne_cms.y *= scalar_xy;
            desired_speed_xy = g2.follow.get_max_speed_cms();
        }

        // Calculate vehicle heading:
        target_heading = 0.0f;
        bool got_target_heading;
        got_target_heading = g2.follow.get_target_heading_deg(target_heading);
        switch (g2.follow.get_yaw_behave())
        {
        case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE:
        {
            if (dist_vec.xy().length_squared() > 1.0)
            {
                yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE:
        {
            if (got_target_heading)
            {
                yaw_cd = target_heading * 100.0f;
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT:
        { // Direction du vecteur vitesse

            // If we set YAW_BEHAVE_DIR_OF_FLIGHT but vehicle is not moving yet, use YAW_BEHAVE_SAME_AS_LEAD_VEHICLE instead
            // This way, drone already points in the right direction when vehicle starts moving
            // if (desired_velocity_neu_cms.xy().length() > (150.0))
            if (vel_of_target.xy().length() > 2.0 && vel_of_follower.xy().length() > 2.0)
            {
                yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_ne_cms);
                use_yaw = true;
            }
            else // Do same thing as for case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE
            {
                if (got_target_heading)
                {
                    yaw_cd = target_heading * 100.0f;
                    use_yaw = true;
                }
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_NONE:
        default:
            // do nothing
            break;
        }

        // Check GPS status
        if (AP::gps().status(0) < g2.follow.get_gpss_req() || g2.follow.get_target_gps_fix_type() < g2.follow.get_gpss_req()) // Check Follower GPS status
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS Status requirements not satisfied.");
            // Reset à 0 toutes les commandes Guided
            desired_velocity_ne_cms.zero();
            use_yaw = false;
            yaw_cd = 0.0f;
            allow_following = false; // Set flag to block the pursuit of the landing sequence after a bug
        }

        // Check heading against target velocity heading (should match)
        // Éventuellement, accorder un poids plus important à la direction du vecteur vitesse à mesure que sa magnitude augmente.
        if (safe_sqrt(sq(vel_of_target.x) + sq(vel_of_target.y)) >= 2.0) // Only calculate if speed is significant enough
        {
            target_speed_bearing = get_bearing_cd(Vector2f{}, vel_of_target.xy()) / 100; // 0 to 360 deg
            float heading_offset = abs(target_speed_bearing - target_heading);
            if (heading_offset > (360 - g2.follow.get_heading_err_deg()))
            {
                heading_offset = 360 - heading_offset;
            }
            if (heading_offset > g2.follow.get_heading_err_deg()) // If offset is too large, there is a problem!
            {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Offset too large between heading and velocity vector. Killing Follow Task.");
                desired_velocity_ne_cms.zero();
                use_yaw = false;
                yaw_cd = 0.0f;
                allow_following = false; // Set flag to block the pursuit of the landing sequence after a bug
            }
        }
        else
        {
            target_speed_bearing = 0;
        } // Speed not high enough to determine a proper target speed bearing
    }
    else // Did not find target
    {
        if (target_was_acquired == true)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Did not find target...");
        }
        target_was_acquired = false;
    }

    // Log output at 10hz:
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0))
    {
        log_request = true;
        last_log_ms = now;
    }

    ModeGuided::set_velocity_ne(desired_velocity_ne_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);
    ModeGuided::run();
}

bool ModeThrow::target_over_vehicle_has_been_reached()
{
    // if (abs(horizontal_dist_from_target_with_offset_cm.x) >= 50.0 || abs(horizontal_dist_from_target_with_offset_cm.y) >= 50.0) // If drone is within landing range of target (100 cm square), start descent!
    // The range is a square of [ 2*g.land_trgt_rng_cm x 2*g.land_trgt_rng_cm ] around the virtual target point.
    if (abs(horizontal_dist_from_target_with_offset_cm.x) >= g.land_trgt_rng_cm || abs(horizontal_dist_from_target_with_offset_cm.y) >= g.land_trgt_rng_cm) // If drone is within landing range of target (100 cm square), start descent!
    {
        return false;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Target in landing range. Starting descent!");
    gcs().send_text(MAV_SEVERITY_INFO, "Dist (NE) to target over vehicle: x:%4.2f cm, y:%4.2f cm.", abs(horizontal_dist_from_target_with_offset_cm.x), abs(horizontal_dist_from_target_with_offset_cm.y));
    return true;
}

bool ModeThrow::user_has_allowed_landing_on_vehicle()
{
    return true; // Read rc channel to receive input from user
}

void ModeThrow::run_landing_state_machine()
{
    switch (state)
    {
    case INIT: // Make sure rangefinder readings are ok before allowing landing

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : INIT");
        }

        shutdown_motors = false;
        activate_rvt_countertorque = false;
        activate_rvt = false;

        if (use_rangefinder)
        {
            // Do pre-landing checks and wait for 100 iterations to populate Rangefinder buffer
            if (do_prelanding_verifications() && lsmCount % 100 == 0)
            {
                state = DESCENT;
                lsmCount = 0;
            }
        }
        else
        {
            state = DESCENT_WITHOUT_RF;
            lsmCount = 0;
        }

        break;

    case DESCENT: // Normal descent with rangefinder detection

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : DESCENT");
        }

        // height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;
        if (use_rangefinder)
        {
            if (lsmCount % 50 == 0)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "RNGFND DIST : %5.1f cm", (double)height_above_ground_cm);
            }

            if (RF_glitch_detected())
            {
                state = DESCENT_WITHOUT_RF;
                gcs().send_text(MAV_SEVERITY_WARNING, "Switched to landing without RF");
                lsmCount = 0;
                break;
            }

            if (height_above_ground_cm <= 70) // Continue normal landing procedure
            {
                if (!drone_was_too_far_from_ground())
                {
                    state = COUNTDOWN;
                    countdown_start = millis();
                    lsmCount = 0;
                }
                else // Rangefinder glitch detected, switch to landing without rangefinder
                {
                    state = DESCENT_WITHOUT_RF;
                    gcs().send_text(MAV_SEVERITY_WARNING, "Switched to landing without RF");
                    lsmCount = 0;
                }
            }
        }
        else
        {
            state = DESCENT_WITHOUT_RF;
            lsmCount = 0;
        }

        break;

    case DESCENT_WITHOUT_RF: // Descent without rangefinder - use IMU only to detect ground

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : DESCENT_WITHOUT_RF");
        }

        if (is_quad_touching_ground())
        {
            state = TOUCHING_GROUND;
            lsmCount = 0;
        } // Safety au cas où touche sol avant la fin du countdown

        break;

    case COUNTDOWN: // Countdown to reach desired height (out of rangefinder's range)

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : COUNTDOWN");
        }

        countdown_chrono = millis() - countdown_start;
        // if (lsmCount%50 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double)countdown_chrono ); }

        if (is_quad_touching_ground())
        {
            state = TOUCHING_GROUND;
            lsmCount = 0;
        } // Safety au cas où touche sol avant la fin du countdown

        if (countdown_chrono >= (uint32_t)countdown_duration)
        {
            state = DROPPING;
            dropping_start = millis();
            lsmCount = 0;
        }

        break;

    case DROPPING: // Drone is free falling

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : DROPPING");
        }

        shutdown_motors = true;
        dropping_chrono = millis() - dropping_start;
        // gcs().send_text(MAV_SEVERITY_INFO, "Dropping chrono : %4.2f milliseconds", (double)dropping_chrono );

        if (is_quad_touching_ground() || dropping_chrono >= dropping_max_duration)
        {
            state = TOUCHING_GROUND;
            lsmCount = 0;
        }

        break;

    case TOUCHING_GROUND: // Drone just hit the ground

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : TOUCHING GROUND");
        }

        state = RVT;
        rvt_start = millis();
        lsmCount = 0;

        break;

    case RVT: // Activate Full Reverse Thrust

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : RVT");
        }

        shutdown_motors = false;
        activate_rvt_countertorque = false;
        activate_rvt = true;

        rvt_chrono = millis() - rvt_start;

        if (is_quad_flipping())
        {
            state = FLIPPING;
            lsmCount = 0;
        }

        if (rvt_chrono > rvt_duration)
        {
            state = LANDED_BUT_STILL_ALERT;
            lsmCount = 0;
        } // Done with RVT, switch to LANDED state.

        if (lsmCount % 100 == 0)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "RVT Timer : %4.2f milliseconds", (double)rvt_chrono);
        }

        break;

    case FLIPPING: // Drone is flipping, but there is still time to recover

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FLIPPING");
        }

        activate_rvt = false;
        rvt_chrono = millis() - rvt_start;

        if (rvt_chrono <= rvt_duration)
        {
            activate_rvt_countertorque = true;
        }
        else
        {
            state = LANDED_BUT_STILL_ALERT;
            lsmCount = 0;
            break;
        } // Done with RVT, switch to LANDED state

        if (is_flipping_getting_worse())
        {
            state = LANDED_BUT_STILL_ALERT;
            lsmCount = 0;
        }
        if (is_lean_angle_stabilizing())
        {
            state = RVT;
            lsmCount = 0;
        } // Go back to RVT state

        gcs().send_text(MAV_SEVERITY_INFO, "RVT Timer : %4.2f milliseconds", (double)rvt_chrono);

        break;

    case LANDED_BUT_STILL_ALERT: // Future work : Add the "staying alert" functionality in case icerberg is rotating

        gcs().send_text(MAV_SEVERITY_INFO, "State : LANDED BUT STILL ALERT");

        activate_rvt = false;
        activate_rvt_countertorque = false;
        shutdown_motors = false;

        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        copter.arming.disarm(AP_Arming::Method::LANDED);
        gcs().send_text(MAV_SEVERITY_INFO, "All motors disarmed...");

        state = DONE;
        lsmCount = 0;

        break;

    case DONE: // Landing is done. Nothing else to do. Change mode to allow for takeoff.

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : DONE");
        }

        // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        // motors->output();
        // copter.arming.disarm(AP_Arming::Method::LANDED);

        break;

    case ABORT_LANDING: // Add "abort landing" functionalities at some point?

        if (lsmCount == 1)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "State : ABORT LANDING");
        }

        break;
    }
}

bool ModeThrow::do_prelanding_verifications() // For roofs and icebergs, not vehicles
{
    if (copter.rangefinder_alt_ok()) // need RF to start procedure
    {
        if (copter.rangefinder_state.alt_cm_glitch_protected >= 70) // make sure drone is not too low
        {
            return true;
        }
        else
        {
            if (lsmCount % 500 == 0)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Drone is too low to start landing procedure");
            }
        }
    }
    else
    {
        if (lsmCount % 100 == 0)
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "Rangefinder data is not available");
        }
    }
    return false;
}

bool ModeThrow::RF_glitch_detected()
{

    int max = RFdistance_buffer[0];
    int min = RFdistance_buffer[0];

    for (int c = 0; c < 10; c++)
    {
        if (RFdistance_buffer[c] > max)
        {
            max = RFdistance_buffer[c];
        }
        if (RFdistance_buffer[c] < min)
        {
            min = RFdistance_buffer[c];
        }
    }
    int delta_max = abs(height_above_ground_cm - max);
    int delta_min = abs(height_above_ground_cm - min);

    if (delta_min >= 75 || delta_max >= 75) // Difference of 75cm between two consecutive rangefinder values
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Rangefinder glitch detected - Delta too high!");
        return true;
    }
    else
    {
        return false;
    }
}

bool ModeThrow::drone_was_too_far_from_ground() // To prevent a rangefinder glitch to trigger motors to shutdown from high altitude
{
    int max = RFdistance_buffer[0];
    for (int c = 0; c < 10; c++)
    {
        if (RFdistance_buffer[c] > max)
        {
            max = RFdistance_buffer[c];
        }
    }
    if (max >= 100) // All the buffered values must be below 200 cm to consider the drone "not too far from ground"
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "Landing glitch detected - Drone too far to land");
        return true;
    }
    else
    {
        return false;
    }
}

bool ModeThrow::is_quad_tilting()
{
    // Check for lean angle over 20 degrees
    float lean_angle_deg = abs(degrees(acosf(ahrs.cos_roll() * ahrs.cos_pitch())));
    bool condition1 = (lean_angle_deg >= 20);
    return condition1;
}

bool ModeThrow::is_quad_dropping() // Returns true if speed > 10 cm/s
{
    bool condition1 = (ahrs.get_accel().length() <= 2); // acceleration of 0 detected while free-falling, m/s²
    // bool condition2 = ( fabsf(inertial_nav.get_velocity_z()) > 10 ); //vertical speed
    return condition1;
}

bool ModeThrow::is_quad_touching_ground() // Returns true if
{
    // bool condition1 = ( ( ahrs.get_gyro_latest().length() ) >= 3.1416 ); // Spike in angular rate (rad/s)
    bool condition2 = ((ahrs.get_accel().length()) >= 1.65 * 9.81); // Spike in acceleration
    // bool condition3 = ( speedZ <= XY ); // speed?
    return condition2;
}

bool ModeThrow::is_quad_flipping()
{
    // Check for lean angle over 55 degrees
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll() * ahrs.cos_pitch()));
    bool condition1 = (lean_angle_deg >= 55);
    return condition1;
}

bool ModeThrow::is_quad_sliding()
{
    // Check speed magnitude and angular rate
    Vector3f vel_ned;
    bool condition1 = false;
    if (ahrs.get_velocity_NED(vel_ned))
    {
        condition1 = (vel_ned.length() >= 0.35); // 35 cm/s
    }

    // Check to make sure not rotating (flipping)
    bool condition2 = (ahrs.get_gyro_latest().length() <= 0.6);
    return condition1 && condition2; // m/s
}

bool ModeThrow::quad_has_landed()
{
    // Check speed, acceleration, angular speed
    return copter.ap.land_complete; // using pre-existing flag
}

bool ModeThrow::is_flipping_getting_worse()
{
    // If angle reaches over 85° and angular rate still positive, flipping is getting worse
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll() * ahrs.cos_pitch()));
    bool condition1 = lean_angle_deg >= 88;
    bool condition2 = (ahrs.get_gyro_latest().length() >= 0.2);
    return condition1 && condition2;
}

bool ModeThrow::is_lean_angle_stabilizing()
{
    // If angular rate within a certain range of 0 and lean angle not too high, let's consider the angle
    // to have stabilized
    bool condition1 = (safe_sqrt(sq(ahrs.get_gyro_latest().x) + sq(ahrs.get_gyro_latest().y)) <= 0.2);
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll() * ahrs.cos_pitch()));
    bool condition2 = lean_angle_deg <= 65; // assuming landing on slopes with angles smaller than 65°
    return condition1 && condition2;
}

void ModeThrow::process_pilot_inputs(float trgt_roll, float trgt_pitch, float trgt_yaw_rate)
{
    if (!copter.failsafe.radio)
    { // if no radio failsafe active, we are good to go!
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR)
        {
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Exiting land mode because of throttle!!");
        }

        if (g.land_repositioning)
        {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(trgt_roll, trgt_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
        }

        // get pilot's desired yaw rate
        trgt_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(trgt_yaw_rate))
        {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }
}

// void ModeThrow::printMsgOnce(MAV_SEVERITY severity, const char* message, ...)
// {

//     va_list arg_list;
//     va_start(arg_list, message);
//     gcs().send_textv(severity, message, arg_list);
//     va_end(arg_list);

//     static std::set<const char*> printedMessages;

//     if (printedMessages.find(message) == printedMessages.end())
//     {
//         gcs().send_text(severity, message);
//         printedMessages.insert(message);
//     }
// }

// ##########################################################################################################
// ##########################################################################################################
// #############################  ANCIEN MODE THROW POUR TESTS EN SALLE LEUE  ###############################
// ##########################################################################################################
// ##########################################################################################################
// ##########################################################################################################
//  // ------------------------------------------------------------------------------------------------------
//  // Mode throw is the mode used to test reverse thrust in a controlled environment (salle LEUE).
//  // The previous mode throw was replaced by a custom mode because it won't be used in the context of this project.
//  // ------------------------------------------------------------------------------------------------------
//  #include "Copter.h"
//  #include <GCS_MAVLink/GCS.h>
//  // ------------------------------------------------------------------------------------------------------
//  // BENCH TEST STATE MACHINE TO EVALUATE PERFORMANCES - ADVANCED VERSION
//  // ------------------------------------------------------------------------------------------------------
//  bool ModeThrow::init(bool ignore_checks)
//  {
//      // Mode - proactive vs reactive
//      landingMode   = PROACTIVE;
//      dropping_time = 0; //milliseconds of drop time

//     // Initialize specific mode variables
//     state                        = INIT; //first state of state machine
//     activate_rvt                 = false;
//     activate_rvt_front_only      = false;
//     activate_max_rvt_for_sliding = false;
//     rvt_pwm                      = 1500; // Motors stopped (bidirectionnal ESCs)
//     ii                           = 0; //counter

//     // reset flag indicating if pilot has applied roll or pitch inputs during landing
//     copter.ap.land_repo_active = false;
//     #if !(RANGEFINDER_ENABLED == ENABLED)
//         gcs().send_text(MAV_SEVERITY_CRITICAL, "Rangefinder NOT enabled!");
//     #endif

//     return true;

//     //gcs().send_text(MAV_SEVERITY_CRITICAL, "Copter is armed, waiting 5 seconds...");
//     //hal.scheduler->delay(5000);
// }

// // Bench Test Main Loop
// // should be called at 100hz or more
// // --------------------------------------------------------------------------------------------
// void ModeThrow::run()
// {
//     //==========================================================================================
//     // REACTIVE MODE
//     //==========================================================================================
//     if (landingMode == REACTIVE)
//     {
//         switch (state)
//         {
//             case INIT: // Initialize some parameters
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "State : INIT - REACTIVE MODE");
//                 if (!copter.motors->armed())
//                 {
//                     copter.arming.arm(AP_Arming::Method::UNKNOWN);
//                     gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors armed!!!");
//                 }
//                 wait_timer = millis();
//                 state = WAIT;
//                 break;

//             case WAIT:
//                 if ( is_quad_dropping() ) { state=DROPPING; }
//                 if (ii == 1)
//                 {
//                     gcs().send_text(MAV_SEVERITY_CRITICAL, "State : WAITING FOR DROP");
//                     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
//                     motors->output();
//                     gcs().send_text(MAV_SEVERITY_CRITICAL, "Setting motors to unlimited throttle!");
//                 }
//                 break;

//             case DROPPING:
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DROPPING");
//                 if ( is_quad_touching_ground() ) { state = TOUCHING_GROUND; }
//                 break;

//             case TOUCHING_GROUND:
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "State : TOUCHING GROUND");
//                 state = FULL_RVT;
//                 full_rvt_timer=millis();
//                 break;

//             case FULL_RVT: // Full RVT for 2 secs (time to stabilize vehicle)
//                 if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FULL_RVT"); }
//                 activate_rvt_front_only = false;
//                 activate_rvt            = true;
//                 if ( is_quad_flipping() ) { state = FLIPPING; }
//                 if ( (millis()-full_rvt_timer) <= 3000 ) {  rvt_pwm = 1100; } //80% reverse throttle
//                 else { state = LANDED; } // Done with RVT, switch to LANDED state.
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "Full RVT Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );
//                 break;

//             case FLIPPING:
//                 if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FLIPPING"); }
//                 activate_rvt = false;
//                 if ( (millis()-full_rvt_timer) <= 3000 ) { activate_rvt_front_only = true; }
//                 else { state = LANDED; break; } // Done with RVT, switch to LANDED state
//                 // If lean angle keeps increasing, shutdown everything and disarm
//                 // If lean angle decreases, it's working. Keep appling RVT to upper motors.
//                 if ( is_flipping_getting_worse() ) { state = LANDED; } //angle overshoots 90°
//                 if ( is_lean_angle_stabilizing() ) { state = FULL_RVT; }
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );
//                 break;

//             case LANDED:
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "State : LANDED");
//                 // Add crash message; differentiate crashes and landings.
//                 activate_rvt            = false;
//                 activate_rvt_front_only = false;
//                 // Disarming motors
//                 motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
//                 motors->output();
//                 copter.arming.disarm(AP_Arming::Method::LANDED);
//                 state = DONE;
//                 break;

//             case DONE:
//                 if (ii%200 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Test is done."); }
//                 break;
//         }
//     }
//     //==========================================================================================
//     // PROACTIVE MODE
//     //==========================================================================================
//     else if (landingMode == PROACTIVE)
//     {
//         switch (state)
//         {
//             case INIT: // Initialize some parameters
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "State : INIT - PROACTIVE MODE");
//                 if (!copter.motors->armed())
//                 {
//                     copter.arming.arm(AP_Arming::Method::UNKNOWN);
//                     gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors armed!!!");
//                 }
//                 wait_timer = millis();
//                 state = WAIT;
//                 break;

//             case WAIT:
//                 if ( is_quad_dropping() ) { state=DROPPING;  dropping_start_timer=millis(); }
//                 if (ii == 1)
//                 {
//                     gcs().send_text(MAV_SEVERITY_CRITICAL, "State : WAITING FOR DROP");
//                     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
//                     motors->output();
//                     gcs().send_text(MAV_SEVERITY_CRITICAL, "Setting motors to unlimited throttle!");
//                 }
//                 break;

//             case DROPPING:
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DROPPING WITH COUNTDOWN");

//                 if ( (millis()-dropping_start_timer) >= dropping_time )
//                 {
//                     full_rvt_timer=millis();
//                     state = FULL_RVT;
//                 }
//                 else if ( is_quad_touching_ground() )
//                 {
//                     state = TOUCHING_GROUND;
//                 }
//                 break;

//             case TOUCHING_GROUND:
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "State : TOUCHING GROUND");
//                 state = FULL_RVT;
//                 full_rvt_timer=millis();
//                 break;

//             case FULL_RVT: // Full RVT for 2 secs (time to stabilize vehicle)
//                 if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FULL_RVT"); }
//                 activate_rvt_front_only = false;
//                 activate_rvt            = true;
//                 if ( is_quad_flipping() ) { state = FLIPPING; }
//                 if ( (millis()-full_rvt_timer) <= 3000 ) {  rvt_pwm = 1000; } //80% reverse throttle
//                 else { state = LANDED; } // Done with RVT, switch to LANDED state.
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "Full RVT Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );
//                 break;

//             case FLIPPING:
//                 if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FLIPPING"); }
//                 activate_rvt = false;
//                 if ( (millis()-full_rvt_timer) <= 3000 ) { activate_rvt_front_only = true; }
//                 else { state = LANDED; break; } // Done with RVT, switch to LANDED state
//                 // If lean angle keeps increasing, shutdown everything and disarm
//                 // If lean angle decreases, it's working. Keep appling RVT to upper motors.
//                 if ( is_flipping_getting_worse() ) { state = LANDED; } //angle overshoots 90°
//                 if ( is_lean_angle_stabilizing() ) { state = FULL_RVT; }
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );
//                 break;

//             case LANDED:
//                 gcs().send_text(MAV_SEVERITY_CRITICAL, "State : LANDED");
//                 // Add crash message; differentiate crashes and landings.
//                 activate_rvt            = false;
//                 activate_rvt_front_only = false;
//                 // Disarming motors
//                 motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
//                 motors->output();
//                 copter.arming.disarm(AP_Arming::Method::LANDED);
//                 state = DONE;
//                 break;

//             case DONE:
//                 if (ii%200 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Test is done."); }
//                 break;
//         }
//     }
//     ii++; //counter

//     // Land State Machine Determination
//     // if (is_disarmed_or_landed())
//     // {
//     //     make_safe_ground_handling();
//     //     gcs().send_text(MAV_SEVERITY_CRITICAL, "Making safe ground handling for other tests.");
//     // }
// }
// // ------------------------------------------------------------------------------------------------------
// // ------------------------------------------------------------------------------------------------------
// bool ModeThrow::is_quad_dropping() // Returns true if speed > 10 cm/s
// {
//     bool condition1 = ( ahrs.get_accel().length() <= 2 ); //acceleration of 0 detected while free-falling, m/s²
//     // bool condition2 = ( fabsf(inertial_nav.get_velocity_z()) > 10 ); //vertical speed
//     return condition1;
// }

// bool ModeThrow::is_quad_touching_ground() // Returns true if
// {
//     bool condition1 = ( ( ahrs.get_gyro_latest().length() ) >= 3.1416 ); // Spike in angular rate (rad/s)
//     bool condition2 = ( ( ahrs.get_accel(). length() ) >= 1.5*9.81 ); // Spike in acceleration
//     // bool condition3 = ( speedZ <= XY ); // speed?
//     return condition1 || condition2;

// }

// bool ModeThrow::is_quad_flipping()
// {
//     // Check for lean angle over 55 degrees
//     float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
//     bool condition1 = (lean_angle_deg >= 63);
//     return condition1;
// }

// bool ModeThrow::is_quad_sliding()
// {
//     // Check speed magnitude and angular rate
//     Vector3f vel_ned;
//     bool condition1 = false;
//     if (ahrs.get_velocity_NED(vel_ned))
//     {
//         condition1 = ( vel_ned.length() >= 0.35 ); // 35 cm/s
//     }

//     //Check to make sure not rotating (flipping)
//     bool condition2 = ( ahrs.get_gyro_latest().length() <= 0.6 );
//     return condition1 && condition2; //m/s
// }

// bool ModeThrow::quad_has_landed()
// {
//     // Check speed, acceleration, angular speed
//     return copter.ap.land_complete; //using pre-existing flag
// }

// bool ModeThrow::is_flipping_getting_worse()
// {
//     // If angle reaches over 85° and angular rate still positive, flipping is getting worse
//     float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
//     bool condition1 = lean_angle_deg>=88;
//     bool condition2 = ( ahrs.get_gyro_latest().length() >= 0.2 );
//     return condition1 && condition2;
// }

// bool ModeThrow::is_lean_angle_stabilizing()
// {
//     // If angular rate within a certain range of 0 and lean angle not too high, let's consider the angle
//     // to have stabilized
//     bool condition1 = ( safe_sqrt(sq(ahrs.get_gyro_latest().x)+sq(ahrs.get_gyro_latest().y)) <= 0.2 );
//     float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
//     bool condition2 = lean_angle_deg<=65; //assuming landing on slopes with angles smaller than 65°
//     return condition1 && condition2;
// }

// // Write values to the motors (override normal controller)
// void ModeThrow::output_to_motors()
// {
//     for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
//     {
//         if (motors->is_motor_enabled(i))
//         {
//             if (activate_rvt) //flag was set during ModeThrow custom state machine
//             {
//                 if (i==1 || i==3) { motors->rc_write(i, rvt_pwm); }
//                 else { motors->rc_write(i, 1000); }
//             }
//             else if (activate_rvt_front_only)
//             {
//                 if (i==1 || i==3) { motors->rc_write(i, 1500); }
//                 else { motors->rc_write(i, 1000); }
//             }
//             else
//             {
//                 motors->rc_write(i, 1500); // motors stopped
//             }
//         }
//     }
// }
// // ------------------------------------------------------------------------------------------------------
// // ------------------------------------------------------------------------------------------------------
// // ------------------------------------------------------------------------------------------------------