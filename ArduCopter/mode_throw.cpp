// ------------------------------------------------------------------------------------------------------
// MODE THROW :
// Mode throw is the mode used to test reverse thrust in a controlled environment (salle LEUE).
// The previous mode throw was replaced by a custom mode because it won't be used in the context of this project.
// ------------------------------------------------------------------------------------------------------
#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
// ------------------------------------------------------------------------------------------------------
// BENCH TEST STATE MACHINE TO EVALUATE PERFORMANCES - ADVANCED VERSION
// ------------------------------------------------------------------------------------------------------
bool ModeThrow::init(bool ignore_checks)
{
    // Mode - proactive vs reactive
    landingMode   = PROACTIVE;
    dropping_time = 0; //milliseconds of drop time
    
    // Initialize specific mode variables
    state                        = INIT; //first state of state machine
    activate_rvt                 = false;
    activate_rvt_front_only      = false;
    activate_max_rvt_for_sliding = false;
    rvt_pwm                      = 1500; // Motors stopped (bidirectionnal ESCs)
    ii                           = 0; //counter

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;
    #if !(RANGEFINDER_ENABLED == ENABLED)
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Rangefinder NOT enabled!");
    #endif
    
    return true;

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "Copter is armed, waiting 5 seconds...");
    //hal.scheduler->delay(5000);
}

// Bench Test Main Loop
// should be called at 100hz or more
// -----------------------------------------------
void ModeThrow::run()
{
    // EXTRA PROTECTIONS : In any situation, if quad goes over a certain angle, disarm
    // and let the drone crash...
    // bool exit_condition_1 = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch())) >= 90; //angle over 90°
    // bool exit_condition_2 = ( (millis()-mode_timer) >= 6000 ); // Timer of 6s (code running for too long, unexpected behavior)
    // if (exit_condition_1 || exit_condition_2)
    // {
    //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    //     motors->output();
    //     copter.arming.disarm(AP_Arming::Method::LANDED);
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "Forcing shutdown as a safety measure! : Conditions : %1.0f, %1.0f", (double) exit_condition_1, (double) exit_condition_2);
    // }

    // // MOST BASIC POSSIBLE STATE MACHINE FOR TESTING PERFROMANCES WITH RVT
    // switch (state)
    // {
    //     case INIT: // Initialize some parameters
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "State : INIT");
    //         //copter.set_auto_armed(true);
    //         if (!copter.motors->armed())
    //         {
    //             copter.arming.arm(AP_Arming::Method::UNKNOWN);
    //             gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors armed!!!");
    //         }
    //         wait_timer = millis();
    //         state = WAIT;
    //         ii=0;
    //         break;
    //     case WAIT:
    //         if ( (millis()-wait_timer) >= 3000 )
    //         {
    //             state = FULL_RVT;
    //             full_rvt_timer = millis();
    //         }
    //         if (ii == 1)
    //         {
    //             motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    //             motors->output();
    //             gcs().send_text(MAV_SEVERITY_CRITICAL, "Setting motors to unlimited throttle!");
    //         }

    //         // Printing sensor data
    //         if (ii%10 == 0)
    //         {
    //             // gcs().send_text(MAV_SEVERITY_CRITICAL, "Angle : %4.2f", degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch())));
    //             // float linearAccelMag = ahrs.get_accel().length();
    //             // gcs().send_text(MAV_SEVERITY_CRITICAL, "Linear Accel Length : %4.2f m/s²", linearAccelMag);
    //             float angularRateMag = ahrs.get_gyro_latest().length();
    //             gcs().send_text(MAV_SEVERITY_CRITICAL, "Angular Rate Length : %4.2f rad/s", angularRateMag);
    //         }
            
    //         break;

    //     case FULL_RVT:
    //         // Printing state to HUD
    //         if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FULL_RVT"); }
            
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "Full RVT Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );

    //         // Full RVT for 2 secs (time to stabilize vehicle)
    //         if ( (millis()-full_rvt_timer) <= 2000 )
    //         {
    //             // Activate full RVT and set desired PWM :
    //             activate_rvt = true;
    //             rvt_pwm = 1100;
    //         }
    //         else { state = LANDED; } // Done with RVT, switch to LANDED state.
    //         break;

    //     case LANDED:
    //         // Printing state to HUD
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "State : LANDED");

    //         activate_rvt = false;

    //         // Disarming motors
    //         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    //         motors->output();
    //         copter.arming.disarm(AP_Arming::Method::LANDED);
    //         state = DONE;
    //         break;
    //     case DONE:
    //         // Printing state to HUD
    //         if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Test is done."); }
    //         break;
    // }

    //==========================================================================================
    // REACTIVE MODE
    //==========================================================================================
    if (landingMode == REACTIVE)
    {
        switch (state)
        {
            case INIT: // Initialize some parameters
                gcs().send_text(MAV_SEVERITY_CRITICAL, "State : INIT - REACTIVE MODE");
                if (!copter.motors->armed())
                {
                    copter.arming.arm(AP_Arming::Method::UNKNOWN);
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors armed!!!");
                }
                wait_timer = millis();
                state = WAIT;
                break;

            case WAIT:
                if ( is_quad_dropping() ) { state=DROPPING; }
                if (ii == 1)
                {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "State : WAITING FOR DROP");
                    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
                    motors->output();
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Setting motors to unlimited throttle!");
                }     
                break;

            case DROPPING:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DROPPING");
                
                // To add eventually : Attitude control while landing (land_run_vertical_control?) (?)
                // ...

                if ( is_quad_touching_ground() ) { state = TOUCHING_GROUND; }
                break;

            case TOUCHING_GROUND:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "State : TOUCHING GROUND");

                state = FULL_RVT;
                full_rvt_timer=millis();

                break;

            case FULL_RVT: // Full RVT for 2 secs (time to stabilize vehicle)
                if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FULL_RVT"); }

                activate_rvt_front_only = false;
                activate_rvt            = true;

                if ( is_quad_flipping() ) { state = FLIPPING; }

                if ( (millis()-full_rvt_timer) <= 3000 ) {  rvt_pwm = 1100; } //80% reverse throttle
                else { state = LANDED; } // Done with RVT, switch to LANDED state.
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Full RVT Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );
                
                break;

            case FLIPPING:
                if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FLIPPING"); }
                activate_rvt = false; 
                if ( (millis()-full_rvt_timer) <= 3000 ) { activate_rvt_front_only = true; }
                else { state = LANDED; break; } // Done with RVT, switch to LANDED state

                // If lean angle keeps increasing, shutdown everything and disarm
                // If lean angle decreases, it's working. Keep appling RVT to upper motors.
                if ( is_flipping_getting_worse() ) { state = LANDED; } //angle overshoots 90°
                if ( is_lean_angle_stabilizing() ) { state = FULL_RVT; }
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );
                
                break;

            case LANDED:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "State : LANDED");

                // Add crash message; differentiate crashes and landings.
                
                activate_rvt            = false;
                activate_rvt_front_only = false;

                // Disarming motors
                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
                motors->output();
                copter.arming.disarm(AP_Arming::Method::LANDED);
                state = DONE;
                break;

            case DONE:
                if (ii%200 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Test is done."); }
                break;
        }
    }
    //==========================================================================================
    // PROACTIVE MODE
    //==========================================================================================
    else if (landingMode == PROACTIVE)
    {
        switch (state)
        {
            case INIT: // Initialize some parameters
                gcs().send_text(MAV_SEVERITY_CRITICAL, "State : INIT - PROACTIVE MODE");
                if (!copter.motors->armed())
                {
                    copter.arming.arm(AP_Arming::Method::UNKNOWN);
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors armed!!!");
                }
                wait_timer = millis();
                state = WAIT;
                break;

            case WAIT:
                if ( is_quad_dropping() ) { state=DROPPING;  dropping_start_timer=millis(); }
                if (ii == 1)
                {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "State : WAITING FOR DROP");
                    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
                    motors->output();
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Setting motors to unlimited throttle!");
                }     
                break;

            case DROPPING:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DROPPING WITH COUNTDOWN");
                
                if ( (millis()-dropping_start_timer) >= dropping_time )
                {
                    full_rvt_timer=millis();
                    state = FULL_RVT;
                }
                else if ( is_quad_touching_ground() )
                {
                    state = TOUCHING_GROUND;
                }
                break;
            
            case TOUCHING_GROUND:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "State : TOUCHING GROUND");

                state = FULL_RVT;
                full_rvt_timer=millis();
                break;
            
            case FULL_RVT: // Full RVT for 2 secs (time to stabilize vehicle)
                if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FULL_RVT"); }

                activate_rvt_front_only = false;
                activate_rvt            = true;

                if ( is_quad_flipping() ) { state = FLIPPING; }

                if ( (millis()-full_rvt_timer) <= 3000 ) {  rvt_pwm = 1000; } //80% reverse throttle
                else { state = LANDED; } // Done with RVT, switch to LANDED state.
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Full RVT Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );
                
                break;

            case FLIPPING:
                if (ii%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FLIPPING"); }
                activate_rvt = false; 
                if ( (millis()-full_rvt_timer) <= 3000 ) { activate_rvt_front_only = true; }
                else { state = LANDED; break; } // Done with RVT, switch to LANDED state

                // If lean angle keeps increasing, shutdown everything and disarm
                // If lean angle decreases, it's working. Keep appling RVT to upper motors.
                if ( is_flipping_getting_worse() ) { state = LANDED; } //angle overshoots 90°
                if ( is_lean_angle_stabilizing() ) { state = FULL_RVT; }
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Timer : %4.2f milliseconds", (double) (millis()-full_rvt_timer) );
                
                break;

            case LANDED:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "State : LANDED");

                // Add crash message; differentiate crashes and landings.
                
                activate_rvt            = false;
                activate_rvt_front_only = false;

                // Disarming motors
                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
                motors->output();
                copter.arming.disarm(AP_Arming::Method::LANDED);
                state = DONE;
                break;

            case DONE:
                if (ii%200 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Test is done."); }
                break;
        }
    }

    // Land State Machine Determination
    // if (is_disarmed_or_landed())
    // {
    //     make_safe_ground_handling();
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "Making safe ground handling for other tests.");
    // }

    ii++; //counter
}
// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
bool ModeThrow::is_quad_dropping() // Returns true if speed > 10 cm/s
{
    bool condition1 = ( ahrs.get_accel().length() <= 2 ); //acceleration of 0 detected while free-falling, m/s²
    // bool condition2 = ( fabsf(inertial_nav.get_velocity_z()) > 10 ); //vertical speed
    return condition1;
}

bool ModeThrow::is_quad_touching_ground() // Returns true if 
{
    bool condition1 = ( ( ahrs.get_gyro_latest().length() ) >= 3.1416 ); // Spike in angular rate (rad/s)
    bool condition2 = ( ( ahrs.get_accel(). length() ) >= 1.5*9.81 ); // Spike in acceleration
    // bool condition3 = ( speedZ <= XY ); // speed?
    return condition1 || condition2;
    
}

bool ModeThrow::is_quad_flipping()
{
    // Check for lean angle over 55 degrees
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    bool condition1 = (lean_angle_deg >= 63);
    return condition1;
}

bool ModeThrow::is_quad_sliding()
{
    // Check speed magnitude and angular rate
    Vector3f vel_ned;
    ahrs.get_velocity_NED(vel_ned);
    bool condition1 = ( vel_ned.length() >= 0.35 ); // 35 cm/s
    
    //Check to make sure not rotating (flipping)
    bool condition2 = ( ahrs.get_gyro_latest().length() <= 0.6 );
    return condition1 && condition2; //m/s
}

bool ModeThrow::quad_has_landed()
{
    // Check speed, acceleration, angular speed
    return copter.ap.land_complete; //using pre-existing flag
}

bool ModeThrow::is_flipping_getting_worse()
{
    // If angle reaches over 85° and angular rate still positive, flipping is getting worse
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    bool condition1 = lean_angle_deg>=88;
    bool condition2 = ( ahrs.get_gyro_latest().length() >= 0.2 );
    return condition1 && condition2;
}

bool ModeThrow::is_lean_angle_stabilizing()
{
    // If angular rate within a certain range of 0 and lean angle not too high, let's consider the angle
    // to have stabilized
    bool condition1 = ( sqrt(sq(ahrs.get_gyro_latest().x)+sq(ahrs.get_gyro_latest().y)) <= 0.2 );
    float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    bool condition2 = lean_angle_deg<=65; //assuming landing on slopes with angles smaller than 65°
    return condition1 && condition2;
}

// Write values to the motors (override normal controller)
void ModeThrow::output_to_motors()
{
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motors->is_motor_enabled(i))
        {
            if (activate_rvt) //flag was set during ModeLand custom state machine
            {
                if (i==1 || i==3) { motors->rc_write(i, rvt_pwm); }
                else { motors->rc_write(i, 1000); }
            }
            else if (activate_rvt_front_only)
            {
                if (i==1 || i==3) { motors->rc_write(i, 1500); }
                else { motors->rc_write(i, 1000); }
            }
            else
            {
                motors->rc_write(i, 1500); // motors stopped
            }
        }
    }
}
// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
// MODE THROW (initial code)
// ------------------------------------------------------------------------------------------------------
// #include "Copter.h"

// #if MODE_THROW_ENABLED == ENABLED

// // throw_init - initialise throw controller
// bool ModeThrow::init(bool ignore_checks)
// {
// #if FRAME_CONFIG == HELI_FRAME
//     // do not allow helis to use throw to start
//     return false;
// #endif

//     // do not enter the mode when already armed or when flying
//     if (motors->armed()) {
//         return false;
//     }

//     // init state
//     stage = Throw_Disarmed;
//     nextmode_attempted = false;

//     // initialise pos controller speed and acceleration
//     pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);
//     pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);

//     // set vertical speed and acceleration limits
//     pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
//     pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

//     return true;
// }

// // runs the throw to start controller
// // should be called at 100hz or more
// void ModeThrow::run()
// {
//     /* Throw State Machine
//     Throw_Disarmed - motors are off
//     Throw_Detecting -  motors are on and we are waiting for the throw
//     Throw_Uprighting - the throw has been detected and the copter is being uprighted
//     Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
//     Throw_PosHold - the copter is kept at a constant position and height
//     */

//     if (!motors->armed()) {
//         // state machine entry is always from a disarmed state
//         stage = Throw_Disarmed;

//     } else if (stage == Throw_Disarmed && motors->armed()) {
//         gcs().send_text(MAV_SEVERITY_INFO,"waiting for throw");
//         stage = Throw_Detecting;

//     } else if (stage == Throw_Detecting && throw_detected()){
//         gcs().send_text(MAV_SEVERITY_INFO,"throw detected - spooling motors");
//         stage = Throw_Wait_Throttle_Unlimited;

//         // Cancel the waiting for throw tone sequence
//         AP_Notify::flags.waiting_for_throw = false;

//     } else if (stage == Throw_Wait_Throttle_Unlimited &&
//                motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
//         gcs().send_text(MAV_SEVERITY_INFO,"throttle is unlimited - uprighting");
//         stage = Throw_Uprighting;
//     } else if (stage == Throw_Uprighting && throw_attitude_good()) {
//         gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
//         stage = Throw_HgtStabilise;

//         // initialise the z controller
//         pos_control->init_z_controller_no_descent();

//         // initialise the demanded height to 3m above the throw height
//         // we want to rapidly clear surrounding obstacles
//         if (g2.throw_type == ThrowType::Drop) {
//             pos_control->set_pos_target_z_cm(inertial_nav.get_altitude() - 100);
//         } else {
//             pos_control->set_pos_target_z_cm(inertial_nav.get_altitude() + 300);
//         }

//         // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
//         copter.set_auto_armed(true);

//     } else if (stage == Throw_HgtStabilise && throw_height_good()) {
//         gcs().send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
//         stage = Throw_PosHold;

//         // initialise position controller
//         pos_control->init_xy_controller();

//         // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
//         copter.set_auto_armed(true);
//     } else if (stage == Throw_PosHold && throw_position_good()) {
//         if (!nextmode_attempted) {
//             switch ((Mode::Number)g2.throw_nextmode.get()) {
//                 case Mode::Number::AUTO:
//                 case Mode::Number::GUIDED:
//                 case Mode::Number::RTL:
//                 case Mode::Number::LAND:
//                 case Mode::Number::BRAKE:
//                 case Mode::Number::LOITER:
//                     set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
//                     break;
//                 default:
//                     // do nothing
//                     break;
//             }
//             nextmode_attempted = true;
//         }
//     }

//     // Throw State Processing
//     switch (stage) {

//     case Throw_Disarmed:

//         // prevent motors from rotating before the throw is detected unless enabled by the user
//         if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
//             motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
//         } else {
//             motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
//         }

//         // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
//         attitude_control->reset_yaw_target_and_rate();
//         attitude_control->reset_rate_controller_I_terms();
//         attitude_control->set_throttle_out(0,true,g.throttle_filt);
//         break;

//     case Throw_Detecting:

//         // prevent motors from rotating before the throw is detected unless enabled by the user
//         if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
//             motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
//         } else {
//             motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
//         }

//         // Hold throttle at zero during the throw and continually reset the attitude controller
//         attitude_control->reset_yaw_target_and_rate();
//         attitude_control->reset_rate_controller_I_terms();
//         attitude_control->set_throttle_out(0,true,g.throttle_filt);

//         // Play the waiting for throw tone sequence to alert the user
//         AP_Notify::flags.waiting_for_throw = true;

//         break;

    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity = inertial_nav.get_velocity().length();
        const float velocity_z = inertial_nav.get_velocity().z;
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        const bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > Throw_PosHold) || throw_position_good();
        
// // @LoggerMessage: THRO
// // @Description: Throw Mode messages
// // @URL: https://ardupilot.org/copter/docs/throw-mode.html
// // @Field: TimeUS: Time since system startup
// // @Field: Stage: Current stage of the Throw Mode
// // @Field: Vel: Magnitude of the velocity vector
// // @Field: VelZ: Vertical Velocity
// // @Field: Acc: Magnitude of the vector of the current acceleration
// // @Field: AccEfZ: Vertical earth frame accelerometer value
// // @Field: Throw: True if a throw has been detected since entering this mode
// // @Field: AttOk: True if the vehicle is upright 
// // @Field: HgtOk: True if the vehicle is within 50cm of the demanded height
// // @Field: PosOk: True if the vehicle is within 50cm of the demanded horizontal position
        
//         AP::logger().WriteStreaming(
//             "THRO",
//             "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk",
//             "s-nnoo----",
//             "F-0000----",
//             "QBffffbbbb",
//             AP_HAL::micros64(),
//             (uint8_t)stage,
//             (double)velocity,
//             (double)velocity_z,
//             (double)accel,
//             (double)ef_accel_z,
//             throw_detect,
//             attitude_ok,
//             height_ok,
//             pos_ok);
//     }
// }

// bool ModeThrow::throw_detected()
// {
//     // Check that we have a valid navigation solution
//     nav_filter_status filt_status = inertial_nav.get_filter_status();
//     if (!filt_status.flags.attitude || !filt_status.flags.horiz_pos_abs || !filt_status.flags.vert_pos) {
//         return false;
//     }

    // Check for high speed (>500 cm/s)
    bool high_speed = inertial_nav.get_velocity().length() > THROW_HIGH_SPEED;

    // check for upwards or downwards trajectory (airdrop) of 50cm/s
    bool changing_height;
    if (g2.throw_type == ThrowType::Drop) {
        changing_height = inertial_nav.get_velocity().z < -THROW_VERTICAL_SPEED;
    } else {
        changing_height = inertial_nav.get_velocity().z > THROW_VERTICAL_SPEED;
    }

//     // Check the vertical acceleraton is greater than 0.25g
//     bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

//     // Check if the accel length is < 1.0g indicating that any throw action is complete and the copter has been released
//     bool no_throw_action = copter.ins.get_accel().length() < 1.0f * GRAVITY_MSS;

//     // High velocity or free-fall combined with increasing height indicate a possible air-drop or throw release
//     bool possible_throw_detected = (free_falling || high_speed) && changing_height && no_throw_action;

    // Record time and vertical velocity when we detect the possible throw
    if (possible_throw_detected && ((AP_HAL::millis() - free_fall_start_ms) > 500)) {
        free_fall_start_ms = AP_HAL::millis();
        free_fall_start_velz = inertial_nav.get_velocity().z;
    }

    // Once a possible throw condition has been detected, we check for 2.5 m/s of downwards velocity change in less than 0.5 seconds to confirm
    bool throw_condition_confirmed = ((AP_HAL::millis() - free_fall_start_ms < 500) && ((inertial_nav.get_velocity().z - free_fall_start_velz) < -250.0f));

//     // start motors and enter the control mode if we are in continuous freefall
//     return throw_condition_confirmed;
// }

// bool ModeThrow::throw_attitude_good() const
// {
//     // Check that we have uprighted the copter
//     const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
//     return (rotMat.c.z > 0.866f); // is_upright
// }

// bool ModeThrow::throw_height_good() const
// {
//     // Check that we are within 0.5m of the demanded height
//     return (pos_control->get_pos_error_z_cm() < 50.0f);
// }

// bool ModeThrow::throw_position_good() const
// {
//     // check that our horizontal position error is within 50cm
//     return (pos_control->get_pos_error_xy_cm() < 50.0f);
// }

// #endif
