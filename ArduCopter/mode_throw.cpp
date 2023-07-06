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
// --------------------------------------------------------------------------------------------
void ModeThrow::run()
{
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
    ii++; //counter

    // Land State Machine Determination
    // if (is_disarmed_or_landed())
    // {
    //     make_safe_ground_handling();
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "Making safe ground handling for other tests.");
    // }    
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
    bool condition1 = false;
    if (ahrs.get_velocity_NED(vel_ned))
    {
        condition1 = ( vel_ned.length() >= 0.35 ); // 35 cm/s
    }
    
    
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
    bool condition1 = ( safe_sqrt(sq(ahrs.get_gyro_latest().x)+sq(ahrs.get_gyro_latest().y)) <= 0.2 );
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