#include "Copter.h"

// land_init - initialise land controller
bool ModeLand::init(bool ignore_checks)
{
    if (copter.failsafe.radio) {
        _fs_brake = true;
        _fs_brake_exit = false;
        // ініціалізуємо pos_control для гальмування як ModeBrake
        pos_control->set_max_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), FS_BRAKE_DECEL_RATE);
        pos_control->set_correction_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), FS_BRAKE_DECEL_RATE);
        pos_control->init_xy_controller();
        pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
        pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "LAND: FS brake init");
        return true;
    }

    _fs_brake = false;
    _fs_brake_exit = false;

    // check if we have GPS and decide which LAND we're going to do
    control_position = copter.position_ok();

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the horizontal position controller
    if (control_position && !pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    land_start_time = millis();
    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AC_PRECLAND_ENABLED
    // initialise precland state machine
    copter.precland_statemachine.init();
#endif

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void ModeLand::run()
{
    if (_fs_brake) {
        fs_brake_run();
        return;
    }
    if (control_position) {
        gps_run();
    } else {
        nogps_run();
    }
}

void ModeLand::fs_brake_run()
{
    // відкладений вихід — RC відновилось на попередньому циклі,
    // тепер безпечно міняємо режим
    if (_fs_brake_exit) {
        _fs_brake_exit = false;
        if (!copter.set_mode(Mode::Number::POSHOLD, ModeReason::RC_COMMAND)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND);
        }
        return;
    }

    // RC відновився — плануємо вихід на наступному циклі
    if (!copter.failsafe.radio) {
        _fs_brake_exit = true;
        return;
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // гальмуємо через pos_control
    Vector2f vel;
    Vector2f accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    pos_control->update_z_controller();

    // коли загальмували — переходимо в POSHOLD або ALT_HOLD
    const float speed_xy = inertial_nav.get_velocity_neu_cms().xy().length();
    if (speed_xy < 50.0f) {
        if (!copter.set_mode(Mode::Number::POSHOLD, ModeReason::RADIO_FAILSAFE)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::RADIO_FAILSAFE);
        }
    }
}

// land_gps_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void ModeLand::gps_run()
{
    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        // run normal landing or precision landing (if enabled)
        land_run_normal_or_precland(land_pause);
    }
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void ModeLand::nogps_run()
{    // TEST
    if (copter.failsafe.radio) {
        if (copter.position_ok()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "LAND: FS -> LOITER");
            copter.set_mode(Mode::Number::LOITER, ModeReason::RADIO_FAILSAFE);
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "LAND: FS -> ALT_HOLD");
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::RADIO_FAILSAFE);
        }
        return;
    }

    float target_roll = 0.0f, target_pitch = 0.0f;

    // process pilot inputs
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
        }
    }

    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        land_run_vertical_control(land_pause);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, auto_yaw.get_heading().yaw_rate_cds);
}

// do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
void ModeLand::do_not_use_GPS()
{
    control_position = false;
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_land_with_pause(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);
    mode_land.set_land_pause(true);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::landing_with_GPS()
{
    return (flightmode->mode_number() == Mode::Number::LAND &&
            mode_land.controlling_position());
}