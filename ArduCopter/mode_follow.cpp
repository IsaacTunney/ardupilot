#include "Copter.h"
#include "GCS_Mavlink.h"

#if MODE_FOLLOW_ENABLED == ENABLED

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AC_AVOID_ENABLED is true because we rely on it velocity limiting functions
 */

// Initialize follow mode
bool ModeFollow::init(const bool ignore_checks)
{
    if (!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

    gcs().send_text(MAV_SEVERITY_CRITICAL, "Max Following speed: %4d cm/s", (int)g2.follow.get_max_speed_cms() );
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Gps requirement: %4d", (int)g2.follow.get_gpss_req() );
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Max target heading error: %4d deg", (int)g2.follow.get_heading_err_deg() );

    // Check GPS status requirements before starting run loop:
    if ( AP::gps().status(0) < g2.follow.get_gpss_req() ) // Check Follower GPS status
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Follower GPS Status: %2d. Requirements not satisfied", AP::gps().status(0));
        return false;
    }
    if ( g2.follow.get_target_gps_fix_type() < g2.follow.get_gpss_req() ) // Check Follower GPS status
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Target GPS Status: %2d. Requirements not satisfied", g2.follow.get_target_gps_fix_type());
        return false;
    }
    allow_following         = true;
    target_was_acquired     = true; 
    msg_target_reached_sent = false;
    runCount                = 0;
    time_last_ms            = 0;

    // Re-use guided mode's initialization
    return ModeGuided::init(ignore_checks);
}

// Perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
}

// Should run at around 200 to 400 Hz
// Called by the main fast_loop in copter.cpp
void ModeFollow::run()
{
    if (is_disarmed_or_landed()) { make_safe_ground_handling(); return; }
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Variables to be sent to velocity controller
    bool use_yaw = false;
    float yaw_cd = 0.0f;
    float target_heading = 0.0f;
    Vector3f desired_velocity_neu_cms; // Vector to be sent to velocity controller
    Vector3f dist_vec;                 // vector to lead vehicle
    Vector3f dist_vec_offs;            // vector to lead vehicle + offset
    Vector3f vel_of_target;            // velocity of lead vehicle
    Vector3f vel_of_follower;

    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target) && allow_following)
    {
        // Convert dist_vec_offs to cm in NEU
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        if (runCount%200 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Dist from target: x:%4.3f m; y:%4.3f m", dist_vec_offs.x, dist_vec_offs.y); }
        if (abs(dist_vec_offs_neu.x) <= 50.0 && abs(dist_vec_offs_neu.y) <= 50.0 && msg_target_reached_sent == false) 
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "TARGET REACHED!");
            msg_target_reached_sent = true;
        }
        
        // Calculate desired velocity vector in cm/s in NEU
        const float kp = g2.follow.get_pos_p().kP();
        const float kd = g2.follow.get_pos_d();
        
        // // Position controller: P controller with Feedforward
        // desired_velocity_neu_cms.x =  (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
        // desired_velocity_neu_cms.y =  (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);
        // desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);

        // New position controller: PD controller with Feedforward
        if (ahrs.get_velocity_NED(vel_of_follower) ) // Ground speed, m/s
        {
            Vector3f vel_of_foll_neu_cms(vel_of_follower.x * 100.0f, vel_of_follower.y * 100.0f, -vel_of_follower.z * 100.0f);
            desired_velocity_neu_cms.x =  (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp) - ( (vel_of_foll_neu_cms.x - vel_of_target.x*100) * kd);
            desired_velocity_neu_cms.y =  (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp) - ( (vel_of_foll_neu_cms.y - vel_of_target.y*100) * kd);
        }
        else
        {
            desired_velocity_neu_cms.x =  (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
            desired_velocity_neu_cms.y =  (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);
        }
        desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);
        //

        // Scale desired velocity to stay within speed limits
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y));
        // if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_max_speed_xy_cms())) {
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > g2.follow.get_max_speed_cms())) {
            // const float scalar_xy = pos_control->get_max_speed_xy_cms() / desired_speed_xy;
            const float scalar_xy =  g2.follow.get_max_speed_cms() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            // desired_speed_xy = pos_control->get_max_speed_xy_cms();
            desired_speed_xy = g2.follow.get_max_speed_cms();            
        }
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

        // THIS COMMMENT SECTION CONCERNS SLOW DOWN CALCULATIONS TO SLOW DOWN NEAR TARGET
        // // Unit vector towards target position (i.e. vector to lead vehicle + offset)
        // Vector3f dir_to_target_neu = dist_vec_offs_neu;
        // const float dir_to_target_neu_len = dir_to_target_neu.length();
        // if (!is_zero(dir_to_target_neu_len)) { dir_to_target_neu /= dir_to_target_neu_len; }
        // // Create horizontal desired velocity vector (required for slow down calculations)
        // Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);
        // // Create horizontal unit vector towards target (required for slow down calculations)
        // Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
        // if (!dir_to_target_xy.is_zero()) { dir_to_target_xy.normalize(); }
        // Slow down horizontally as we approach target
        // Default: 1/2 of maximum deceleration for gentle slow down
        // const float dist_to_target_xy = Vector2f(dist_vec_offs_neu.x, dist_vec_offs_neu.y).length();
        // // copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 1.0f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // // Copy horizontal velocity limits back to 3d vector
        // desired_velocity_neu_cms.x = desired_velocity_xy_cms.x;
        // desired_velocity_neu_cms.y = desired_velocity_xy_cms.y;

        // Limit vertical desired_velocity_neu_cms to slow as we approach target
        // Default: 1/2 of maximum deceleration for gentle slow down
        // const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
        const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 1.0f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -des_vel_z_max, des_vel_z_max);

        // Limit the velocity for obstacle/fence avoidance
        copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // Calculate vehicle heading
        target_heading = 0.0f;
        bool got_target_heading;
        got_target_heading = g2.follow.get_target_heading_deg(target_heading);
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
                if (got_target_heading) {
                    yaw_cd = target_heading * 100.0f;
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

        // Check GPS status
        if ( AP::gps().status(0) < g2.follow.get_gpss_req() || g2.follow.get_target_gps_fix_type() < g2.follow.get_gpss_req() ) // Check Follower GPS status
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS Status requirements not satisfied.");
            // Reset à 0 toutes les commandes Guided
            desired_velocity_neu_cms.zero();
            use_yaw = false;
            yaw_cd = 0.0f;
            allow_following = false; // Set flag to block the pursuit of the landing sequence after a bug
        }

        // Check heading against target velocity heading: (should match)
        //     *Éventuellement, accorder un poids plus important à la direction du vecteur vitesse à mesure que sa magnitude augmente.
        //      (Degré de confiance proportionnel à son amplitude)
        //    **Éventuellement, pour augmenter vitesse calcul, mettre ces checks avant le reste des opérations ci-dessus
        if ( sqrt( sq(vel_of_target.x) + sq(vel_of_target.y) ) >= 2.0 ) // Only calculate if speed is significant enough
        {
            target_speed_bearing = get_bearing_cd(Vector2f{}, vel_of_target.xy())/100; // 0 to 360 deg
            float heading_offset = abs(target_speed_bearing - target_heading);
            if ( heading_offset > (360 - g2.follow.get_heading_err_deg()) ) { heading_offset = 360 - heading_offset; }
            if ( heading_offset > g2.follow.get_heading_err_deg() ) // If offset is too large, there is a problem!
            {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Offset too large between heading and velocity vector. Killing Follow Task.");
                desired_velocity_neu_cms.zero();
                use_yaw = false;
                yaw_cd = 0.0f;
                allow_following = false; // Set flag to block the pursuit of the landing sequence after a bug
            }
        }
        else
        {
            target_speed_bearing = 0; // Speed not high enough to determine a proper target speed bearing
        }
        target_was_acquired = true;
    }
    else // Could not get target's distance and velocity
    {
        if (target_was_acquired == true) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Did not find target..."); }
        target_was_acquired = false;
    }

    // Printing stuff to the HUD:
    // if (i%200 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Target speed bearing: %4.2f deg", target_speed_bearing); }
    // average_time_between_updates_ms += g2.follow.get_time_between_updates_ms();
    uint32_t time_now_ms = AP_HAL::millis();
    if (runCount%400 == 0)
    {
        uint32_t avg_time_ms = (time_now_ms-time_last_ms) / g2.follow.get_num_of_msg_received();
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Target msgs updates freq: %3ld Hz", 1000/avg_time_ms );
        g2.follow.reset_num_of_msg_received(); // Reset mavlink msg counter to zero
        time_last_ms = time_now_ms;
    }

    // Log output at 10hz for ModeGuided commands
    // Note: Logs specific to ModeFollow are in AP_Follow library
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0))
    {
        log_request = true;
        last_log_ms = now;
    }

    // Send velocity commands to Guided Mode:
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);
    ModeGuided::run();
    
    runCount++;
}

uint32_t ModeFollow::wp_distance() const
{
    return g2.follow.get_distance_to_target() * 100;
}

int32_t ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
 */
bool ModeFollow::get_wp(Location &loc) const
{
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_FOLLOW_ENABLED == ENABLED
