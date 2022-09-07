#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
#include <algorithm>

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

    target_acquired                = false;
    state                          = INIT;
    landingOnVehicle_state         = FOLLOWING;
    landingOnVehicle_previousState = FOLLOWING;
    switchLandingState             = false;
    lsmCount                       = 1; // counter
    runCount                       = 0; // Main run loop iterator

    shutdown_motors             = false;
    activate_rvt_countertorque  = false;
    activate_rvt                = false;
    land_pause                  = false;

    rvt_duration                = 2000; // Duration of rvt after landing, milliseconds
    countdown_duration          = ( 70.0 - (double)g.shutdown_height_cm ) / (double)g.land_speed * 1000.0; // milliseconds
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown duration (const): %4.2d milliseconds", (int)countdown_duration);
    dropping_max_duration       = 500; // milliseconds

    copter.ap.land_repo_active = false; // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.prec_land_active = false; // this will be set true if prec land is later active
    auto_yaw.set_mode(AUTO_YAW_HOLD); // initialise yaw

    #if AC_FENCE == ENABLED
        copter.fence.auto_disable_fence_for_landing(); // disable the fence on landing
    #endif

    #if !(RANGEFINDER_ENABLED == ENABLED)
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Rangefinder NOT enabled!");
    #endif

    if (g.landing_type == VEHICLE)
    {
        if (!g2.follow.enabled())
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
            return false;
        }
        return ModeGuided::init(ignore_checks);
    }
    else { return true; }
}


// Perform cleanup required when leaving mode
void ModeLand::exit()
{
    g2.follow.clear_offsets_if_required();
    shutdown_motors             = false;
    activate_rvt_countertorque  = false;
    activate_rvt                = false;
    attitude_control->landing_controller_setRVT(shutdown_motors, activate_rvt, activate_rvt_countertorque, g.rvt_pwm);
}


// land_run - runs the land controller
// should be called at 100hz or more
void ModeLand::run()
{
    if (control_position)
    {
        if (g.landing_type == VEHICLE) { landing_on_moving_vehicle_run(); }
        else { landing_with_gps_run(); }
    }
    else { landing_without_gps_run(); }
    lsmCount++;
    runCount++;
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

// LAND CONTROLLER WITH GPS - Horizontal position controlled with loiter controller
// Frequency: 100hz or more
void ModeLand::landing_with_gps_run()
{
    // height_above_ground_cm = copter.current_loc.alt;
    height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;

    // STATE MACHINE:
    run_landing_state_machine();

    // POST-LANDING SAFETY CHECKS:
    // Disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Ground idle detected; Disarming drone...");
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
    attitude_control->landing_controller_setRVT(shutdown_motors, activate_rvt, activate_rvt_countertorque, g.rvt_pwm);

    // Managing buffer for rangefinder distances:
    for (int c = 8; c >= 0; c--) { RFdistance_buffer[c+1] = RFdistance_buffer[c]; }
    RFdistance_buffer[0] = height_above_ground_cm;
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "RF buffer: %4ld, %4ld, %4ld, %4ld, %4ld, %4ld, %4ld, %4ld, %4ld, %4ld", RFdistance_buffer[0], RFdistance_buffer[1], RFdistance_buffer[2], RFdistance_buffer[3], RFdistance_buffer[4], RFdistance_buffer[5], RFdistance_buffer[6], RFdistance_buffer[7], RFdistance_buffer[8], RFdistance_buffer[9]);
}

// LAND CONTROLLER WITH NO GPS - Pilot controls roll and pitch angles
// Frequency: 100hz or more
void ModeLand::landing_without_gps_run()
{
    float   target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0;
    // height_above_ground_cm = copter.current_loc.alt;
    height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;

    process_pilot_inputs(target_roll, target_pitch, target_yaw_rate);

    // STATE MACHINE:
    run_landing_state_machine();

    // POST-LANDING SAFETY CHECKS: Landing detector
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Ground idle detected; Disarming drone...");
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
    attitude_control->landing_controller_setRVT(shutdown_motors, activate_rvt, activate_rvt_countertorque, g.rvt_pwm);

    // Managing buffer for rangefinder distances:
    for (int c = 8; c >= 0; c--) { RFdistance_buffer[c+1] = RFdistance_buffer[c]; }
    RFdistance_buffer[0] = height_above_ground_cm;
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "RF buffer: %4ld, %4ld, %4ld, %4ld, %4ld", ...
    //       ... RFdistance_buffer[0], RFdistance_buffer[1], RFdistance_buffer[2], RFdistance_buffer[3], RFdistance_buffer[4]);
}

// LAND CONTROLLER WITH GPS FOR LANDING ON MOVING TARGET - Guided mode commands for horizontal control and standard z-controller for altitude
// Frequency: 100hz or more
void ModeLand::landing_on_moving_vehicle_run()
{
    height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;
    
    // SAFETY CHECKS:
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED); // Disarm when the landing detector says we've landed
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Ground idle detected; Disarming drone...");
    }
    if (is_disarmed_or_landed()) { make_safe_ground_handling(); return; }

    // High-level state machine for landing on moving vehicle:
    switch (landingOnVehicle_state)
    {
        case FOLLOWING:
            follow_target_3D();
            if (target_acquired == true && target_over_vehicle_has_been_reached() ) { landingOnVehicle_state = READY_FOR_DESCENT; }
            break;

        case READY_FOR_DESCENT: //Drone is within GPS landing area
            if (switchLandingState == true) { gcs().send_text(MAV_SEVERITY_CRITICAL, "LANDING STATE: READY FOR DESCENT"); switchLandingState = false; }
            follow_target_3D();
            if ( user_has_allowed_landing_on_vehicle() ) { landingOnVehicle_state = LANDING; } //With RC switch?
            break;

        case LANDING:
            if (switchLandingState == true) { gcs().send_text(MAV_SEVERITY_CRITICAL, "LANDING STATE: LANDING"); switchLandingState = false; }
            follow_target_2D();
            run_landing_state_machine();
            land_run_vertical_control(land_pause);
            attitude_control->landing_controller_setRVT(shutdown_motors, activate_rvt, activate_rvt_countertorque, g.rvt_pwm);
            break;
        default:
            break;
    }

    // Landing and post-landing safety checks:
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        copter.arming.disarm(AP_Arming::Method::LANDED);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Ground idle detected; Disarming drone...");
    }
    if (is_disarmed_or_landed()) { make_safe_ground_handling(); }

    // Managing buffer for rangefinder distances:
    for (int c = 8; c >= 0; c--) { RFdistance_buffer[c+1] = RFdistance_buffer[c]; }
    RFdistance_buffer[0] = height_above_ground_cm;

    // Managing the switch of landing states:
    if (landingOnVehicle_previousState != landingOnVehicle_state) { switchLandingState = true; }
    landingOnVehicle_previousState = landingOnVehicle_state;
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

void ModeLand::follow_target_3D()
{
    if (is_disarmed_or_landed()) { make_safe_ground_handling(); return; }
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f desired_velocity_neu_cms; // Vector to be sent to velocity controller
    Vector3f dist_vec;                 // vector to lead vehicle
    Vector3f dist_vec_offs;            // vector to lead vehicle + offset
    Vector3f vel_of_target;            // velocity of lead vehicle

    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target))
    {
        target_acquired = true;

        // Convert dist_vec_offs to cm in NEU:
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        horizontal_dist_from_target_with_offset_cm = dist_vec_offs_neu.xy(); // cm
        
        if (runCount%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Dist (NEU) to target over vehicle: x:%4.3f m; y:%4.3f m; z:%4.3f m.", dist_vec_offs_neu.x/100.0, dist_vec_offs_neu.y/100.0, dist_vec_offs_neu.z/100.0); }

        // Calculate desired velocity vector in cm/s in NEU:
        const float kp = g2.follow.get_pos_p().kP();
        desired_velocity_neu_cms.x =  (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
        desired_velocity_neu_cms.y =  (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);
        desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);

        // Scale desired velocity to stay within horizontal speed limit:
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_max_speed_xy_cms())) {
            const float scalar_xy = pos_control->get_max_speed_xy_cms() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed_xy = pos_control->get_max_speed_xy_cms();
        }
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

        // REMOVED EVERYTHING CONCERNING SLOW DOWN NEAR TARGET (only useful for follow mode):
        // // Create unit vector towards target position (i.e. vector to lead vehicle + offset):
        // Vector3f dir_to_target_neu = dist_vec_offs_neu;
        // const float dir_to_target_neu_len = dir_to_target_neu.length();
        // if (!is_zero(dir_to_target_neu_len)) { dir_to_target_neu /= dir_to_target_neu_len; }
        // // Create horizontal desired velocity vector (required for slow down calculations):
        // Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);
        // // Create horizontal unit vector towards target (required for slow down calculations):
        // Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
        // if (!dir_to_target_xy.is_zero()) { dir_to_target_xy.normalize(); }
        // // Slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down):
        // const float dist_to_target_xy = Vector2f(dist_vec_offs_neu.x, dist_vec_offs_neu.y).length();
        // copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // // Copy horizontal velocity limits back to 3d vector:
        // desired_velocity_neu_cms.x = desired_velocity_xy_cms.x;
        // desired_velocity_neu_cms.y = desired_velocity_xy_cms.y;

        // Limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down):
        const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -des_vel_z_max, des_vel_z_max);

        // Limit the velocity for obstacle/fence avoidance:
        copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // Calculate vehicle heading:
        switch (g2.follow.get_yaw_behave())
        {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                if (dist_vec.xy().length_squared() > 1.0) {
                    yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                float target_hdg = 0.0f;
                if (g2.follow.get_target_heading_deg(target_hdg)) {
                    yaw_cd = target_hdg * 100.0f;
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                if (desired_velocity_neu_cms.xy().length_squared() > (100.0 * 100.0)) {
                    yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_neu_cms.xy());
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
               break;

        }
    
    }
    else
    {
        if (runCount%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Did not find target..."); }
        target_acquired = false;

        // If target gets lost during pursuit, tell drone to simply keep its position and do nothing:
        desired_velocity_neu_cms.x = 0;
        desired_velocity_neu_cms.y = 0;
        desired_velocity_neu_cms.z = 0;

    }

    // Log output at 10hz:
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) { log_request = true; last_log_ms = now; }

    // Re-use guided mode's velocity controller (takes NEU):
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);
    ModeGuided::run();
}

void ModeLand::follow_target_2D()
{
    if (is_disarmed_or_landed()) { make_safe_ground_handling(); return; }
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    Vector2f desired_velocity_ne_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;       // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {

        target_acquired = true;
        
        // Convert dist_vec_offs to cm in NE:
        const Vector2f dist_vec_offs_ne(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f);

        if (runCount%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Horizontal dist (NE) to target over vehicle: x:%4.3f m; y:%4.3f m.", dist_vec_offs_ne.x/100.0, dist_vec_offs_ne.y/100.0); }
                
        // Calculate desired velocity vector in cm/s in NE:
        const float kp = g2.follow.get_pos_p().kP();
        desired_velocity_ne_cms.x =  (vel_of_target.x * 100.0f) + (dist_vec_offs_ne.x * kp);
        desired_velocity_ne_cms.y =  (vel_of_target.y * 100.0f) + (dist_vec_offs_ne.y * kp);

        // Scale desired velocity to stay within horizontal speed limit:
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_ne_cms.x) + sq(desired_velocity_ne_cms.y));
        if ( !is_zero(desired_speed_xy) && ( desired_speed_xy > pos_control->get_max_speed_xy_cms() ) )
        {
            const float scalar_xy = pos_control->get_max_speed_xy_cms() / desired_speed_xy;
            desired_velocity_ne_cms.x *= scalar_xy;
            desired_velocity_ne_cms.y *= scalar_xy;
            desired_speed_xy = pos_control->get_max_speed_xy_cms();
        }

        // REMOVED EVERYTHING CONCERNING SLOW DOWN NEAR TARGET (only useful for follow mode):
        // // Create unit vector towards target position (i.e. vector to lead vehicle + offset):
        // Vector2f dir_to_target_ne = dist_vec_offs_ne;
        // const float dir_to_target_ne_len = dir_to_target_ne.length();
        // if (!is_zero(dir_to_target_ne_len)) { dir_to_target_ne /= dir_to_target_ne_len; }
        // // Create horizontal desired velocity vector and unit vector towards target (required for slow down calculations):
        // Vector2f desired_velocity_xy_cms(desired_velocity_ne_cms.x, desired_velocity_ne_cms.y);
        // Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
        // if (!dir_to_target_xy.is_zero()) { dir_to_target_xy.normalize(); }
        // // Slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down):
        // const float dist_to_target_xy = Vector2f(dist_vec_offs_ne.x, dist_vec_offs_ne.y).length();
        // copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // desired_velocity_ne_cms.x = desired_velocity_xy_cms.x;
        // desired_velocity_ne_cms.y = desired_velocity_xy_cms.y;

        // Calculate vehicle heading:
        switch (g2.follow.get_yaw_behave())
        {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                if (dist_vec.xy().length_squared() > 1.0) {
                    yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                float target_hdg = 0.0f;
                if (g2.follow.get_target_heading_deg(target_hdg)) {
                    yaw_cd = target_hdg * 100.0f;
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                if (desired_velocity_ne_cms.length_squared() > (100.0 * 100.0)) {
                    yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_ne_cms);
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
               break;

        }
    }
    else
    {
        if (runCount%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Did not find target..."); }
        target_acquired = false;

        // If target gets lost during pursuit, tell drone to simply keep its position and do nothing:
        desired_velocity_ne_cms.x = 0;
        desired_velocity_ne_cms.y = 0;
        
        // Add something here to tell z_controller to stop descending???
    }

    // Log output at 10hz:
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) { log_request = true; last_log_ms = now; }

    ModeGuided::set_velocity_ne(desired_velocity_ne_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);
    ModeGuided::run();
}

bool ModeLand::target_over_vehicle_has_been_reached()
{
    if (abs(horizontal_dist_from_target_with_offset_cm.x) >= 50.0 || abs(horizontal_dist_from_target_with_offset_cm.y) >= 50.0) // If drone is within landing range of target (100 cm square), start descent!
    {
        return false;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Target in landing range. Starting descent!");
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Dist (NE) to target over vehicle: x:%4.2f cm, y:%4.2f cm.", abs(horizontal_dist_from_target_with_offset_cm.x), abs(horizontal_dist_from_target_with_offset_cm.y) );
    return true;
}

bool ModeLand::user_has_allowed_landing_on_vehicle()
{
    return true; //read rc channel to receive input from user
}

void ModeLand::run_landing_state_machine()
{
    switch (state)
    {
        case INIT: // Make sure rangefinder readings are ok before allowing landing

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : INIT"); }

            shutdown_motors            = false;
            activate_rvt_countertorque = false;
            activate_rvt               = false;

            // Do pre-landing checks and wait for 100 iterations to populate Rangefinder buffer:
            if ( do_prelanding_verifications() && lsmCount%100==0 ) { state = DESCENT; lsmCount = 0; }

            break;

        case DESCENT: // Normal descent with rangefinder detection

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DESCENT"); }

            height_above_ground_cm = copter.rangefinder_state.alt_cm_glitch_protected;

            if (lsmCount%50 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "RNGFND DIST : %5.1f cm", (double)height_above_ground_cm); }

            if ( RF_glitch_detected() )
            {
                state = DESCENT_WITHOUT_RF;
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Switch to landing without RF");
                lsmCount = 0;           
            }
            
            if (height_above_ground_cm <= 70) // Continue normal landing procedure
            {
                if (!drone_was_too_far_from_ground())
                {
                    state = COUNTDOWN;
                    countdown_start  = millis();
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Start countdown before drop");
                    lsmCount = 0;
                }
                else // Rangefinder glitch detected, switch to landing without rangefinder
                {
                    state = DESCENT_WITHOUT_RF;
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Switch to landing without RF");
                    lsmCount = 0;
                }
            }

            break;

        case DESCENT_WITHOUT_RF: // Descent without rangefinder - use IMU only to detect ground

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DESCENT_WITHOUT_RF"); }

            if ( is_quad_touching_ground() || is_quad_tilting() ) { state = TOUCHING_GROUND; lsmCount = 0; } // Safety au cas où touche sol avant la fin du countdown

            break;

        case COUNTDOWN: // Countdown to reach desired height (out of rangefinder's range)

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : COUNTDOWN"); }

            countdown_chrono = millis() - countdown_start;
            if (lsmCount%50 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Countdown : %4.2f milliseconds", (double)countdown_chrono ); }

            if ( is_quad_touching_ground() || is_quad_tilting() ) { state = TOUCHING_GROUND; lsmCount = 0; } // Safety au cas où touche sol avant la fin du countdown

            if ( countdown_chrono >= (uint32_t)countdown_duration )
            {
                state = DROPPING;
                dropping_start = millis();
                lsmCount = 0;
            }

            break;

        case DROPPING: // Drone is free falling

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DROPPING"); }

            shutdown_motors = true;
            dropping_chrono = millis() - dropping_start;
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Dropping chrono : %4.2f milliseconds", (double)dropping_chrono );

            if (is_quad_touching_ground() || dropping_chrono >= dropping_max_duration) { state = TOUCHING_GROUND; lsmCount = 0; }

            break;

        case TOUCHING_GROUND: // Drone just hit the ground

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : TOUCHING GROUND"); }

            state = RVT;
            rvt_start = millis();
            lsmCount = 0;

            break;

        case RVT: // Activate Full Reverse Thrust

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : RVT"); }
            
            shutdown_motors            = false;
            activate_rvt_countertorque = false;
            activate_rvt               = true;

            rvt_chrono = millis()-rvt_start;

            if ( is_quad_flipping() ) { state = FLIPPING; lsmCount = 0; }

            //if ( rvt_chrono <= rvt_duration ) { g.rvt_pwm = 1400; } // 90% reverse throttle
            //else
            if ( rvt_chrono > rvt_duration ) { state = LANDED_BUT_STILL_ALERT; lsmCount = 0; } // Done with RVT, switch to LANDED state.

            if(lsmCount%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "RVT Timer : %4.2f milliseconds", (double)rvt_chrono ); }
            
            break;

        case FLIPPING: // Drone is flipping, but there is still time to recover

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : FLIPPING"); }

            activate_rvt = false; 
            rvt_chrono = millis()-rvt_start;

            if ( rvt_chrono <= rvt_duration ) { activate_rvt_countertorque = true; }
            else { state = LANDED_BUT_STILL_ALERT; lsmCount = 0; break; } // Done with RVT, switch to LANDED state

            if ( is_flipping_getting_worse() ) { state = LANDED_BUT_STILL_ALERT; lsmCount = 0; }
            if ( is_lean_angle_stabilizing() ) { state = RVT; lsmCount = 0; } // Go back to RVT state

            gcs().send_text(MAV_SEVERITY_CRITICAL, "RVT Timer : %4.2f milliseconds", (double)rvt_chrono );

            break;

        case LANDED_BUT_STILL_ALERT: // Future work : Add the "staying alert" functionality in case icerberg is rotating

            gcs().send_text(MAV_SEVERITY_CRITICAL, "State : LANDED BUT STILL ALERT");

            activate_rvt               = false;
            activate_rvt_countertorque = false;
            shutdown_motors            = false;

            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
            copter.arming.disarm(AP_Arming::Method::LANDED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "All motors disarmed...");

            state = DONE;
            lsmCount = 0;

            break;

        case DONE: // Landing is done. Nothing else to do. Change mode to allow for takeoff.

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : DONE"); }

            // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            // motors->output();
            // copter.arming.disarm(AP_Arming::Method::LANDED);

            break;

        case ABORT_LANDING: // Future work : Add "abort landing" functionalities

            if (lsmCount == 1) { gcs().send_text(MAV_SEVERITY_CRITICAL, "State : ABORT LANDING"); }

            break;
    } 
}

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

bool ModeLand::RF_glitch_detected()
{

    int max = RFdistance_buffer[0];
    int min = RFdistance_buffer[0];

    for (int c = 0; c < 10; c++)
    {
        if (RFdistance_buffer[c] > max) { max = RFdistance_buffer[c]; }
        if (RFdistance_buffer[c] < min) { min = RFdistance_buffer[c]; }
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

bool ModeLand::drone_was_too_far_from_ground() // To prevent a rangefinder glitch to trigger motors to shutdown from high altitude
{
    int max = RFdistance_buffer[0];
    for (int c = 0; c < 10; c++)
    {
        if (RFdistance_buffer[c] > max) { max = RFdistance_buffer[c]; }
    }
    if (max >= 100) //All the buffered values must be below 200 cm to consider the drone "not too far from ground"
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Landing glitch detected - Drone to far to land");
        return true;
    }
    else
    {
        return false;
    }
}

bool ModeLand::is_quad_tilting()
{
    // Check for lean angle over 20 degrees
    float lean_angle_deg = abs(degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch())));
    bool condition1 = (lean_angle_deg >= 20);
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
    bool condition1 = (lean_angle_deg >= 55);
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
    return condition1 && condition2; // m/s
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

