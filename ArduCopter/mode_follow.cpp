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
    allow_following = true;
    i = 0;

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
    // If not armed, set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // // Checking rates (should expect around 200 to 400 Hz)
    // uint64_t time_now = AP_HAL::micros64();
    // if (i%100==0)
    // {
    //     float period = (time_now-last_run_loop_ms)/100;
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "Loop rate: %5.2f Hz", (1000000/period) );
    //     last_run_loop_ms = time_now;
    // }

    // Set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Variables to be sent to velocity controller
    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;
    float target_heading = 0.0f;

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target) && allow_following)
    {
        // Convert dist_vec_offs to cm in NEU
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        if (i%200 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Dist from target: x:%4.3f m; y:%4.3f m", dist_vec_offs.x, dist_vec_offs.y); }

        // Calculate desired velocity vector in cm/s in NEU
        const float kp = g2.follow.get_pos_p().kP();
        
        desired_velocity_neu_cms.x =  (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
        desired_velocity_neu_cms.y =  (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);
        desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);

        // Scale desired velocity to stay within horizontal speed limit
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_max_speed_xy_cms())) {
            const float scalar_xy = pos_control->get_max_speed_xy_cms() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed_xy = pos_control->get_max_speed_xy_cms();
        }

        // Limit desired velocity to be between maximum climb and descent rates
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

        // Unit vector towards target position (i.e. vector to lead vehicle + offset)
        Vector3f dir_to_target_neu = dist_vec_offs_neu;
        const float dir_to_target_neu_len = dir_to_target_neu.length();
        if (!is_zero(dir_to_target_neu_len)) {
            dir_to_target_neu /= dir_to_target_neu_len;
        }

        // Create horizontal desired velocity vector (required for slow down calculations)
        Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);

        // Create horizontal unit vector towards target (required for slow down calculations)
        Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
        if (!dir_to_target_xy.is_zero()) {
            dir_to_target_xy.normalize();
        }

        // Slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
        const float dist_to_target_xy = Vector2f(dist_vec_offs_neu.x, dist_vec_offs_neu.y).length();
        copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // copy horizontal velocity limits back to 3d vector
        desired_velocity_neu_cms.x = desired_velocity_xy_cms.x;
        desired_velocity_neu_cms.y = desired_velocity_xy_cms.y;

        // Limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
        const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
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
            gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS Status requirements not satisfied. ");
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
        if ( sqrt( sq(vel_of_target.x) + sq(vel_of_target.y) ) >= 1.0 ) // Only calculate if speed is significant enough
        {
            target_speed_bearing = get_bearing_cd(Vector2f{}, vel_of_target.xy())/100; // 0 to 360 deg
            // g2.follow.get_target_heading_deg(target_heading); // 0 to 360 deg
            if ( abs(target_speed_bearing - target_heading) > 30 ) // Offset is too large, there is a problem!
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
    }
    else // Could not get target's distance and velocity
    {
        if (i%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Did not find target..."); }
    }

    // Printing stuff to the HUD:
    if (i%200 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Target speed bearing: %4.2f deg", target_speed_bearing); }
    if (i%100 == 0) { gcs().send_text(MAV_SEVERITY_CRITICAL, "Time in between target msg updates: %4ld ms", g2.follow.get_time_between_updates_ms() ); }

    // Log output at 10hz for ModeGuided commands
    // Note: Logs specific to ModeFollow are in AP_Follow library
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0))
    {
        log_request = true;
        last_log_ms = now;
    }

    // Re-use guided mode's velocity controller (takes NEU)
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);
    ModeGuided::run();

    i++;
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
