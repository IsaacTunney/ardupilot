#include "Copter.h"
#include <GCS_MAVLink/GCS.h>

// Initialise land controller
bool ModeLand::init(bool ignore_checks)
{
    // Check (only once) if we have GPS to decide on which landing sequence to execute
    control_position = copter.position_ok();
    // **Éventuellement, seulement accepter le landing AVEC GPS et RANGEFINDER!

    // Set horizontal/vertical speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // Initialize the horizontal and vertical position controller
    if (control_position && !pos_control->is_active_xy()) { pos_control->init_xy_controller(); }
    if ( !pos_control->is_active_z() ) { pos_control->init_z_controller(); }

    land_start_time     = millis();
    land_loop_time      = millis();
    
    state = DESCENT;
    flyingState = flying;

    start_countdown_before_drop = false;
    motorsShutDown              = false;
    land_pause                  = false;

    i                           = 0; //counter

    activate_rvt_countertorque  = false;
    activate_rvt                = false;
    rvt_duration                = 2000; // Duration of rvt after landing, milliseconds

    copter.ap.land_repo_active = false; // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.prec_land_active = false; // this will be set true if prec land is later active
    auto_yaw.set_mode(AUTO_YAW_HOLD); // initialise yaw

    #if AC_FENCE == ENABLED
        copter.fence.auto_disable_fence_for_landing(); // disable the fence on landing
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
    if (control_position) { gps_run();   }
    else                  { nogps_run(); }
}


// land_gps_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void ModeLand::gps_run()
{
    int32_t height_above_ground_cm = copter.current_loc.alt;

    // STATE MACHINE:
    switch (state)
    {
        case INIT: // Make sure rangefinder readings are ok before allowing landing

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : INIT"); }

            if ( do_prelanding_verifications() ) { state = DESCENT; i = 0; }

            break;

        case DESCENT: // Normal descent with rangefinder detection

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DESCENT"); }

            height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;

            if (i%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "RNGFND DIST : %5.1f cm", (double)height_above_ground_cm); }
            
            if (height_above_ground_cm <= 70)
            {
                state = COUNTDOWN;
                countdown_start  = millis();
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Starting countdown before drop!");
            }

            break;

        case COUNTDOWN: // Countdown to reach desired height (out of rangefinder's range)

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : COUNTDOWN"); }

            countdown_duration = (70 - g.shutdown_height_cm) / g.land_speed * 1000; // milliseconds
            countdown_chrono = millis() - countdown_start;

            if ( countdown_chrono >= countdown_duration )
            {
                state = DROPPING;
                i = 0;
            }
            else
            {
                if (i%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double)countdown_chrono ); }
            }

            // TO ADD : If is_quad_tilting() { state = TOUCHING_GROUND; i = 0; } // Quad reached ground before the end of the countdown

            break;

        case DROPPING: // Drone is free falling

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DROPPING"); }

            if (is_quad_touching_ground() ) { state = TOUCHING_GROUND; i = 0; }

            break;

        case TOUCHING_GROUND: // Drone just hit the ground

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : TOUCHING GROUND"); }

            state = RVT;
            rvt_start = millis();
            i = 0;

            break;

        case RVT: // Activate Full Reverse Thrust

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : RVT"); }
            
            activate_rvt_countertorque = false;
            activate_rvt               = true;
            rvt_chrono = millis()-rvt_start;

            if ( is_quad_flipping() ) { state = FLIPPING; i = 0; }

            if ( rvt_chrono <= rvt_duration ) { g.rvt_pwm = 1050; } // 90% reverse throttle
            else { state = LANDED_BUT_STILL_ALERT; i = 0; } // Done with RVT, switch to LANDED state.

            gcs().send_text(MAV_SEVERITY_CRITICAL, "RVT Timer : %4.2f milliseconds", (double)rvt_chrono );
            
            break;

        case FLIPPING: // Drone is flipping, but there is still time to recover

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FLIPPING"); }

            activate_rvt = false; 
            rvt_chrono = millis()-rvt_start;

            if ( rvt_chrono <= rvt_duration ) { activate_rvt_countertorque = true; }
            else { state = LANDED_BUT_STILL_ALERT; i = 0; break; } // Done with RVT, switch to LANDED state

            if ( is_flipping_getting_worse() ) { state = LANDED_BUT_STILL_ALERT; i = 0; }
            if ( is_lean_angle_stabilizing() ) { state = RVT; i = 0; } // Go back to RVT state

            gcs().send_text(MAV_SEVERITY_CRITICAL, "RVT Timer : %4.2f milliseconds", (double)rvt_chrono );

            break;

        case LANDED_BUT_STILL_ALERT: // Future work : Add the "staying alert" functionality in case icerberg is rotating

            gcs().send_text(MAV_SEVERITY_CRITICAL, "State : LANDED BUT STILL ALERT");

            activate_rvt               = false;
            activate_rvt_countertorque = false;

            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
            copter.arming.disarm(AP_Arming::Method::LANDED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "All motors disarmed...");

            state = DONE;
            i = 0;

            break;

        case DONE: // Landing is done. Nothing else to do. Change mode to allow for takeoff.

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DONE"); }

            break;

        case ABORT_LANDING: // Future work : Add "abort landing" functionalities

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : ABORT LANDING"); }

            break;
    }
    i++;

    // POST-LANDING SAFETY CHECKS:
    // Disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Flight controller during landing sequence:
    if (is_disarmed_or_landed())
    {
        make_safe_ground_handling();
        loiter_nav->init_target();
    }
    else //still flying
    {
        loiter_nav->clear_pilot_desired_acceleration();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED); // set motors to full range
        land_run_horiz_and_vert_control(land_pause);
    }

    // Set boolean flag in the AP_Motors library to indicate to the motors_outputs when to activate reverse thrust
    attitude_control->landing_controller_setRVT(activate_rvt, activate_rvt_countertorque, g.rvt_pwm);
}


// LAND CONTROLLER WITH NO GPS - pilot controls roll and pitch angles
// Frequency: 100hz or more
void ModeLand::nogps_run()
{
    float   target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0;
    int32_t height_above_ground_cm = copter.current_loc.alt;

    process_pilot_inputs(target_roll, target_pitch, target_yaw_rate);

    // STATE MACHINE:
    switch (state)
    {
        case INIT: // Make sure rangefinder readings are ok before allowing landing

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : INIT"); }

            if ( do_prelanding_verifications() ) { state = DESCENT; i = 0; }

            break;

        case DESCENT: // Normal descent with rangefinder detection

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DESCENT"); }

            height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;

            if (i%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "RNGFND DIST : %5.1f cm", (double)height_above_ground_cm); }
            
            if (height_above_ground_cm <= 70)
            {
                state = COUNTDOWN;
                countdown_start  = millis();
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Starting countdown before drop!");
            }

            break;

        case COUNTDOWN: // Countdown to reach desired height (out of rangefinder's range)

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : COUNTDOWN"); }

            countdown_duration = (70 - g.shutdown_height_cm) / g.land_speed * 1000; // milliseconds
            countdown_chrono = millis() - countdown_start;

            if ( countdown_chrono >= countdown_duration )
            {
                state = DROPPING;
                i = 0;
            }
            else
            {
                if (i%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double)countdown_chrono ); }
            }

            // TO ADD : If is_quad_tilting() { state = TOUCHING_GROUND; i = 0; } // Quad reached ground before the end of the countdown

            break;

        case DROPPING: // Drone is free falling

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DROPPING"); }

            if (is_quad_touching_ground() ) { state = TOUCHING_GROUND; i = 0; }

            break;

        case TOUCHING_GROUND: // Drone just hit the ground

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : TOUCHING GROUND"); }

            state = RVT;
            rvt_start = millis();
            i = 0;

            break;

        case RVT: // Activate Full Reverse Thrust

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : RVT"); }
            
            activate_rvt_countertorque = false;
            activate_rvt               = true;
            rvt_chrono = millis()-rvt_start;

            if ( is_quad_flipping() ) { state = FLIPPING; i = 0; }

            if ( rvt_chrono <= rvt_duration ) { g.rvt_pwm = 1050; } // 90% reverse throttle
            else { state = LANDED_BUT_STILL_ALERT; i = 0; } // Done with RVT, switch to LANDED state.

            gcs().send_text(MAV_SEVERITY_CRITICAL, "RVT Timer : %4.2f milliseconds", (double)rvt_chrono );
            
            break;

        case FLIPPING: // Drone is flipping, but there is still time to recover

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FLIPPING"); }

            activate_rvt = false; 
            rvt_chrono = millis()-rvt_start;

            if ( rvt_chrono <= rvt_duration ) { activate_rvt_countertorque = true; }
            else { state = LANDED_BUT_STILL_ALERT; i = 0; break; } // Done with RVT, switch to LANDED state

            if ( is_flipping_getting_worse() ) { state = LANDED_BUT_STILL_ALERT; i = 0; }
            if ( is_lean_angle_stabilizing() ) { state = RVT; i = 0; } // Go back to RVT state

            gcs().send_text(MAV_SEVERITY_CRITICAL, "RVT Timer : %4.2f milliseconds", (double)rvt_chrono );

            break;

        case LANDED_BUT_STILL_ALERT: // Future work : Add the "staying alert" functionality in case icerberg is rotating

            gcs().send_text(MAV_SEVERITY_CRITICAL, "State : LANDED BUT STILL ALERT");

            activate_rvt               = false;
            activate_rvt_countertorque = false;

            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
            copter.arming.disarm(AP_Arming::Method::LANDED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "All motors disarmed...");

            state = DONE;
            i = 0;

            break;

        case DONE: // Landing is done. Nothing else to do. Change mode to allow for takeoff.

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DONE"); }

            break;

        case ABORT_LANDING: // Future work : Add "abort landing" functionalities

            if (i == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : ABORT LANDING"); }

            break;
    }
    i++;
    
    // if (copter.rangefinder_alt_ok())
    // {
    //     height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;
        
    //     // Printing (on HUD) height from rangefinder every 200 milliseconds
    //     if (millis()-land_loop_time >= 200) { gcs().send_text(MAV_SEVERITY_CRITICAL, "RNGFND DIST : %5.1f cm", (double)height_above_ground_cm); land_loop_time = millis(); }

    //     // Check conditions to start countdown before drop.
    //     // First step of landing sequence.
    //     if (height_above_ground_cm <= 60 && height_above_ground_cm  >= 55 && start_countdown_before_drop == false)
    //     {
    //         start_countdown_before_drop = true;
    //         countdown_before_drop_time  = millis();
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "Starting countdown before drop!");
    //     }
    // }
    
    // // ICEBERG LANDING TESTS
    // if (start_countdown_before_drop == true && is_quad_touching_ground() )
    // {
    //     motorsShutDown = true;
    //     reverse_thrust_timer = millis();
    // }
    // else
    // {
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double) (millis()-countdown_before_drop_time) );
    // }

    // if ( motorsShutDown == true ) //Last part of the landing sequence
    // {
    //     if ( millis()-reverse_thrust_timer <= 2000 ) // 2 seconds of full RVT or simply sertting motors to PWM of 1500 (forced stop)
    //     {
    //         activate_rvt = true; //Command sent to motor class (passing through land controller)
    //         // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::REVERSE_THRUST);
    //         // motors->output();
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "Reverse thrust activated!");
    //     }
    //     else if ( !(is_disarmed_or_landed()) ) // After 2 seconds, disarm motors.
    //     {
    //         // Stop reverse thrust and shut down motors
    //         activate_rvt = false;
    //         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    //         motors->output();
    //         copter.arming.disarm(AP_Arming::Method::LANDED);
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "Shutting Down and Disarming!");
    //     }
    //     else
    //     {
    //         activate_rvt = false; //making sure this variable gets set to false if not in reverse thrust mode.
    //     }
    // }

    // POST-LANDING SAFETY CHECKS: Landing detector
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Flight controller during landing sequence:
    if (is_disarmed_or_landed())
    {
        make_safe_ground_handling();
    }
    else //still flying
    {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        land_run_vertical_control(land_pause);
    }

    // Call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // Set boolean flag in the AP_Motors library to indicate to the motors_outputs when to activate reverse thrust
    attitude_control->landing_controller_setRVT(activate_rvt, activate_rvt_countertorque, g.rvt_pwm);
}


//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
bool ModeLand::do_prelanding_verifications()
{
    if ( copter.rangefinder_alt_ok() )  // need RF to start procedure
    {
        if ( copter.rangefinder_state.alt_cm_glitch_protected >= 70 ) //make sure drone is not too low
        {
            return true;
        }
        else
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Drone is too low to start landing procedure");
        }
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Rangefinder data is not available");
    }
    return false;
}

bool ModeLand::is_quad_tilting()
{
    // Check for lean angle over 55 degrees
    float lean_angle_deg = abs(degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch())));
    bool condition1 = (lean_angle_deg >= 10);
    return condition1;
}

bool ModeLand::is_quad_dropping() // Returns true if speed > 10 cm/s
{
    bool condition1 = ( ahrs.get_accel().length() <= 2 ); //acceleration of 0 detected while free-falling, m/s²
    // bool condition2 = ( fabsf(inertial_nav.get_velocity_z()) > 10 ); //vertical speed
    return condition1;
}

bool ModeLand::is_quad_touching_ground() // Returns true if 
{
    bool condition1 = ( ( ahrs.get_gyro_latest().length() ) >= 3.1416 ); // Spike in angular rate (rad/s)
    bool condition2 = ( ( ahrs.get_accel(). length() ) >= 1.5*9.81 ); // Spike in acceleration
    // bool condition3 = ( speedZ <= XY ); // speed?
    return condition1 || condition2;
    
}

bool ModeLand::is_quad_flipping()
{
    // Check for lean angle over 55 degrees
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    bool condition1 = (lean_angle_deg >= 63);
    return condition1;
}

bool ModeLand::is_quad_sliding()
{
    // Check speed magnitude and angular rate
    Vector3f vel_ned;
    bool condition1 = false;
    if (ahrs.get_velocity_NED(vel_ned))
    {
        condition1 = ( vel_ned.length() >= 0.35 ); // 35 cm/s
    }
    
    
    //Check to make sure not rotating (flipping)
    bool condition2 = ( ahrs.get_gyro_latest().length() <= 0.6 );
    return condition1 && condition2; //m/s
}

bool ModeLand::quad_has_landed()
{
    // Check speed, acceleration, angular speed
    return copter.ap.land_complete; //using pre-existing flag
}

bool ModeLand::is_flipping_getting_worse()
{
    // If angle reaches over 85° and angular rate still positive, flipping is getting worse
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    bool condition1 = lean_angle_deg>=88;
    bool condition2 = ( ahrs.get_gyro_latest().length() >= 0.2 );
    return condition1 && condition2;
}

bool ModeLand::is_lean_angle_stabilizing()
{
    // If angular rate within a certain range of 0 and lean angle not too high, let's consider the angle
    // to have stabilized
    bool condition1 = ( sqrt(sq(ahrs.get_gyro_latest().x)+sq(ahrs.get_gyro_latest().y)) <= 0.2 );
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    bool condition2 = lean_angle_deg<=65; //assuming landing on slopes with angles smaller than 65°
    return condition1 && condition2;
}

void ModeLand::process_pilot_inputs(float trgt_roll, float trgt_pitch, float trgt_yaw_rate)
{
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
            get_pilot_desired_lean_angles(trgt_roll, trgt_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
        }

        // get pilot's desired yaw rate
        trgt_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(trgt_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }
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
