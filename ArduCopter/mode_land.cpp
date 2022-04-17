#include "Copter.h"
#include <GCS_MAVLink/GCS.h>

// custom_land_init - initialise land controller
bool ModeLand::init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do in the run method
    control_position = copter.position_ok();

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the horizontal position controller
    if (control_position && !pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }
    // else { return false; } si je veux empêcher l'accès au mode sans GPS...

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if ( !pos_control->is_active_z() ) { pos_control->init_z_controller(); }

    land_start_time     = millis();
    land_loop_time      = millis();
    land_GPSloop_time   = millis();
    land_NoGPSloop_time = millis();
    
    delay1 = millis();
    activate_reverse_thrust = false;

    start_countdown_before_drop = false;
    motorsShutDown = false;

    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    #if AC_FENCE == ENABLED
        // disable the fence on landing
        copter.fence.auto_disable_fence_for_landing();
    #endif

    #if !(RANGEFINDER_ENABLED == ENABLED)
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Rangefinder NOT enabled!");
    #endif

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void ModeLand::run()
{
    if (control_position)
    {
        if (millis()-land_GPSloop_time >= 2000)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Landing with GPS...");
            land_GPSloop_time = millis();
        }
        gps_run();
    }
    else
    {
        if (millis()-land_NoGPSloop_time >= 2000)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "No GPS! Pursuing landing task...");
            land_NoGPSloop_time = millis();
        }
        nogps_run();
    }
}

// land_gps_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void ModeLand::gps_run()
{
    // Get height above ground estimate
    int32_t height_above_ground_cm = copter.current_loc.alt;

    // STATE MACHINE DESCRIPTION :
    // Check if rangefinder data is usable.
    // If it is, get filtered distance from rangefinder.
    // If height above ground < 60 cm, continue descent for XX s.
    // Then, shut down motors and disarm drone.
    // (Eventually, add full reverse thrust for 3 seconds to secure landing)
    // Else (if rangefinder data not usable), continue with normal landing.
    if (copter.rangefinder_alt_ok())
    {
        height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;
        
        // Printing (on HUD) height from rangefinder every 1 sec (1000 millis)
        if (millis()-land_loop_time >= 200)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RNGFND DIST : %5.1f cm", (double)height_above_ground_cm);
            land_loop_time = millis();
        }

        // Check conditions to start countdown before drop
        // Keep in rangefinder.available case so that the state machine only starts when using rangefinder data
        // (Otherwise, follow normal landing procedures...)
        if (height_above_ground_cm <= 60 && height_above_ground_cm  >= 55 && start_countdown_before_drop == false)
        {
            start_countdown_before_drop = true;
            countdown_before_drop_time  = millis();
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Starting countdown before drop!");
        }
    }
    // // ICEBERG LANDING TESTS
    // // Activate drop sequence if estimated altitude is 50cm (for speed of 50 cm/s)
    // if ( start_countdown_before_drop == true && motorsShutDown == false )
    // {
    //     if ( (millis()-countdown_before_drop_time) >= 700 ) //50cm/s for 0.6s = 30cm so will drop at 30cm from ground
    //     {
    //         //Countdown done. Stop motors or apply reverse thrust for 2000 milliseconds.
    //         motorsShutDown = true;
    //         reverse_thrust_timer = millis();
    //     }
    //     else //Printing countdown on HUD
    //     {  
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double) (millis()-countdown_before_drop_time) );
    //     }
    // }

    // ICEBERG LANDING TESTS
    if (start_countdown_before_drop == true && is_quad_touching_ground() )
    {
        motorsShutDown = true;
        reverse_thrust_timer = millis();
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double) (millis()-countdown_before_drop_time) );
    }


    if ( motorsShutDown == true ) //Last part of the landing sequence
    {
        if ( millis()-reverse_thrust_timer <= 2000 ) // 2 seconds of full RVT or simply sertting motors to PWM of 1500 (forced stop)
        {
            activate_reverse_thrust = true; //Command sent to motor class (passing through land controller)
            // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::REVERSE_THRUST);
            // motors->output();
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Reverse thrust activated!");
        }
        else if ( !(is_disarmed_or_landed()) ) // After 2 seconds, disarm motors.
        {
            // Stop reverse thrust and shut down motors
            activate_reverse_thrust = false;
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
            copter.arming.disarm(AP_Arming::Method::LANDED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Shutting Down and Disarming!");
        }
        else
        {
            activate_reverse_thrust = false; //making sure this variable gets set to false if not in reverse thrust mode.
        }
    }


    // Disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed())
    {
        make_safe_ground_handling();
        loiter_nav->init_target();
    }
    else //still flying
    {
        loiter_nav->clear_pilot_desired_acceleration();
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        
        // pause before beginning land descent (currently deactivated in init method)
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) { land_pause = false; }
        
        land_run_horiz_and_vert_control(land_pause);
    }
    // The member "landing_controller_withPosition_with_RVT" has the only purpose of setting a bool flag in the AP_Motors library
    // to indicate to the motors_outputs functions to activate reverse thrust when the moment comes.
    attitude_control->landing_controller_setRVT(activate_reverse_thrust, g.rvt_pwm);
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void ModeLand::nogps_run()
{
    float   target_roll = 0.0f, target_pitch = 0.0f;
    float   target_yaw_rate = 0;
    int32_t height_above_ground_cm = copter.current_loc.alt; // Get height above ground estimate

    //------------------------------------------------------------------------------------------
    // process pilot inputs
    if (!copter.failsafe.radio) { //if no radio failsafe active, we are good to go!
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Exiting land mode because of throttle!!");
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }
    //------------------------------------------------------------------------------------------
    // LANDING STATE MACHINE :
    // Check if rangefinder data is usable.
    // If it is, get filtered distance from rangefinder.
    // If height above ground < 60 cm, continue descent for XX s.
    // Then, shut down motors and disarm drone.
    // (Eventually, add full reverse thrust for 3 seconds to secure landing)
    // Else (if rangefinder data not usable), continue with normal landing.
    if (copter.rangefinder_alt_ok())
    {
        height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;
        
        // Printing (on HUD) height from rangefinder every 200 milliseconds
        if (millis()-land_loop_time >= 200) { gcs().send_text(MAV_SEVERITY_CRITICAL, "RNGFND DIST : %5.1f cm", (double)height_above_ground_cm); land_loop_time = millis(); }

        // Check conditions to start countdown before drop.
        // First step of landing sequence.
        if (height_above_ground_cm <= 60 && height_above_ground_cm  >= 55 && start_countdown_before_drop == false)
        {
            start_countdown_before_drop = true;
            countdown_before_drop_time  = millis();
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Starting countdown before drop!");
        }
    }

    // // ICEBERG LANDING TESTS
    // // Activate drop sequence if estimated altitude is 50cm (for speed of 50 cm/s)
    // if ( start_countdown_before_drop == true && motorsShutDown == false )
    // {
    //     if ( (millis()-countdown_before_drop_time) >= 900 ) //50cm/s for 0.7s = 45cm so will drop at 15cm from ground
    //     {
    //         //Countdown done. Stop motors or apply reverse thrust for 2000 milliseconds.
    //         motorsShutDown = true;
    //         reverse_thrust_timer = millis();
    //     }
    //     else
    //     {
    //         //Printing the countdown on the HUD
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double) (millis()-countdown_before_drop_time) );
    //     }
    // }
    
    // ICEBERG LANDING TESTS
    if (start_countdown_before_drop == true && is_quad_touching_ground() )
    {
        motorsShutDown = true;
        reverse_thrust_timer = millis();
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double) (millis()-countdown_before_drop_time) );
    }

    if ( motorsShutDown == true ) //Last part of the landing sequence
    {
        if ( millis()-reverse_thrust_timer <= 2000 ) // 2 seconds of full RVT or simply sertting motors to PWM of 1500 (forced stop)
        {
            activate_reverse_thrust = true; //Command sent to motor class (passing through land controller)
            // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::REVERSE_THRUST);
            // motors->output();
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Reverse thrust activated!");
        }
        else if ( !(is_disarmed_or_landed()) ) // After 2 seconds, disarm motors.
        {
            // Stop reverse thrust and shut down motors
            activate_reverse_thrust = false;
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
            copter.arming.disarm(AP_Arming::Method::LANDED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Shutting Down and Disarming!");
        }
        else
        {
            activate_reverse_thrust = false; //making sure this variable gets set to false if not in reverse thrust mode.
        }
    }

    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed())
    {
        make_safe_ground_handling();
    }
    else //still flying
    {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        land_run_vertical_control(land_pause);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // Call custom function to set reverse thrust to g.rvt_pwm when activate_reverse_thrust == true
    attitude_control->landing_controller_setRVT(activate_reverse_thrust, g.rvt_pwm);
    // Added _wth_RVT to the "input_euler_angle_roll_pitch_euler_rate_yaw" function to differentiate the function that implementes reverse thrust.
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

bool ModeLand::is_quad_touching_ground()
{
    // Check for lean angle over 55 degrees
    float lean_angle_deg = abs(degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch())));
    bool condition1 = (lean_angle_deg >= 10);
    return condition1;
}

// UNUSED FUNCTIONS
//-------------------------------------------------------------------------

// do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
void ModeLand::do_not_use_GPS()
{
    control_position = false;
}
//-------------------------------------------------------------------------

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_land_with_pause(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);
    mode_land.set_land_pause(true);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}
//-------------------------------------------------------------------------
// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::landing_with_GPS()
{
    return (flightmode->mode_number() == Mode::Number::LAND &&
            mode_land.controlling_position());
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

// OLD LAND MODE WITH NO MODIFICATIONS
//-------------------------------------------------------------------------
//
// // land_init - initialize land controller
// bool ModeLand::init(bool ignore_checks)
// {

//     gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)3.142f);
//     // check if we have GPS and decide which LAND we're going to do
//     control_position = copter.position_ok();
//     if (control_position) {
//         // set target to stopping point
//         Vector2f stopping_point;
//         loiter_nav->get_stopping_point_xy(stopping_point);
//         loiter_nav->init_target(stopping_point);
//     }

//     // set vertical speed and acceleration limits
//     pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
//     pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
//     pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

//     // initialise the vertical position controller
//     if (!pos_control->is_active_z()) {
//         pos_control->init_z_controller();
//     }

//     land_start_time = millis();
//     land_pause = false;

//     // reset flag indicating if pilot has applied roll or pitch inputs during landing
//     copter.ap.land_repo_active = false;

//     // initialise yaw
//     auto_yaw.set_mode(AUTO_YAW_HOLD);

// #if LANDING_GEAR_ENABLED == ENABLED
//     // optionally deploy landing gear
//     copter.landinggear.deploy_for_landing();
// #endif

// #if AC_FENCE == ENABLED
//     // disable the fence on landing
//     copter.fence.auto_disable_fence_for_landing();
// #endif

// #if PRECISION_LANDING == ENABLED
//     // initialise precland state machine
//     copter.precland_statemachine.init();
// #endif

//     return true;
// }

// // land_run - runs the land controller
// // should be called at 100hz or more
// void ModeLand::run()
// {
//     if (control_position) {
//         gps_run();
//     } else {
//         nogps_run();
//     }
// }

// // land_gps_run - runs the land controller
// //      horizontal position controlled with loiter controller
// //      should be called at 100hz or more
// void ModeLand::gps_run()
// {
//     // disarm when the landing detector says we've landed
//     if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
//         copter.arming.disarm(AP_Arming::Method::LANDED);
//     }

//     // Land State Machine Determination
//     if (is_disarmed_or_landed()) {
//         make_safe_ground_handling();
//         loiter_nav->clear_pilot_desired_acceleration();
//         loiter_nav->init_target();
//     } else {
//         // set motors to full range
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

//         // pause before beginning land descent
//         if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
//             land_pause = false;
//         }

//         // run normal landing or precision landing (if enabled)
//         land_run_normal_or_precland(land_pause);
//     }
// }

// // land_nogps_run - runs the land controller
// //      pilot controls roll and pitch angles
// //      should be called at 100hz or more
// void ModeLand::nogps_run()
// {
//     float target_roll = 0.0f, target_pitch = 0.0f;
//     float target_yaw_rate = 0;

//     // process pilot inputs
//     if (!copter.failsafe.radio) {
//         if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
//             AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
//             // exit land if throttle is high
//             copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
//         }

//         if (g.land_repositioning) {
//             // apply SIMPLE mode transform to pilot inputs
//             update_simple_mode();

//             // get pilot desired lean angles
//             get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
//         }

//         // get pilot's desired yaw rate
//         target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
//         if (!is_zero(target_yaw_rate)) {
//             auto_yaw.set_mode(AUTO_YAW_HOLD);
//         }
//     }

//     // disarm when the landing detector says we've landed
//     if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
//         copter.arming.disarm(AP_Arming::Method::LANDED);
//     }

//     // Land State Machine Determination
//     if (is_disarmed_or_landed()) {
//         make_safe_ground_handling();
//     } else {
//         // set motors to full range
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

//         // pause before beginning land descent
//         if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
//             land_pause = false;
//         }

//         land_run_vertical_control(land_pause);
//     }

//     // call attitude controller
//     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
// }
