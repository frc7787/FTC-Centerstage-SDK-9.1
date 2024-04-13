package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Properties {
    // ---------- ARM PROPERTIES ---------- //

    // TELEOP POSITIONS

    public static volatile int BOTTOM_EXT_POS = 1580;
    public static volatile int BOTTOM_ROT_POS = 1250;

    public static volatile int LOW_EXT_POS = 1782;
    public static volatile int LOW_ROT_POS = 1432;

    public static volatile int MED_EXT_POS = 2107;
    public static volatile int MED_ROT_POS = 1713;

    public static volatile int HIGH_EXT_POS = 2415;
    public static volatile int HIGH_ROT_POS = 1822;

    public static volatile int TOP_EXT_POS = 2667;
    public static volatile int TOP_ROT_POS = 1909;

    public static volatile int ENDGAME_POSITION = 2133;

    public static volatile double DEFAULT_WORM_POWER     = 1.0;
    public static volatile double DEFAULT_ELEVATOR_POWER = 1.0; // 1.0;
    public static volatile double ELEVATOR_HOMING_POWER  = 0.8; // Absolute value
    public static volatile double WORM_HOMING_POWER      = 0.8; // Absolute value

    // AUTO

    public static volatile int AUTO_INITIAL_WORM_POSITION          = 1400;
    public static volatile int YELLOW_PIXEL_WORM_POSITION          = 1100;
    public static volatile int YELLOW_PIXEL_ELEVATOR_POSITION      = 2200;

    public static volatile int YELLOW_PIXEL_CLEARING_WORM_POSITION = 1000;

    public static volatile double ELEVATOR_EXTENSION_SPEED_AUTO = 0.75;
    public static volatile double ELEVATOR_RETRACTION_SPEED_AUTO = 1.0;


    // ----------- DRIVE PROPERTIES ---------- //

    public static volatile double DEAD_ZONE_LOW  = 0.9;
    public static volatile double DEAD_ZONE_HIGH = 0.9;
    public static volatile double STRAFE_OFFSET  = 1.1;


    // ----------- INTAKE PROPERTIES ---------- //

    public static volatile double DEFAULT_INTAKE_POWER       = 1.0;
    public static volatile double DEFAULT_OUTTAKE_POWER      = 0.4;

    // --------- DELIVERY TRAY PROPERTIES ---------- //

    public static volatile double TRAY_DOOR_OPEN_POS   = 0.25;
    public static volatile double TRAY_DOOR_CLOSED_POS = 0.0;

    // ---------- LAUNCHER PROPERTIES ---------- //

    public static volatile double LAUNCHER_SERVO_ZERO_POS   = 0.00;
    public static volatile double LAUNCHER_SERVO_LAUNCH_POS = 0.60;

    // ---------- HANGER PROPERTIES ---------- //

    public static volatile double HANGER_SERVO_POSITION = 1.0;

    // ----------- CAMERA PROPERTIES ------------ //

    public static volatile int CAMERA_WIDTH = 320;

    public static volatile Size CAMERA_RESOLUTION = new Size(640, 480);

    public static volatile double LEFT_X  = 0.25 * (double) CAMERA_WIDTH;
    public static volatile double RIGHT_X = 0.75 * (double) CAMERA_WIDTH;

    public static volatile int EXPOSURE_MS   = 2;
    public static volatile int GAIN          = 0;
    public static volatile int WHITE_BALANCE = 4000;

    // ----------- AUTO PID / TOLERANCE PROPERTIES ---------- //

    public static volatile double DRIVE_GAIN  = 0.027;
    public static volatile double STRAFE_GAIN = 0.07;
    public static volatile double TURN_GAIN   = 0.03;

    public static volatile double DRIVE_D  = 0.0025;
    public static volatile double STRAFE_D = 0.00002;
    public static volatile double TURN_D   = 0.0013;

    public static volatile double YAW_ERROR_TOLERANCE     = 0.5;
    public static volatile double BEARING_ERROR_TOLERANCE = 0.5;
    public static volatile double RANGE_ERROR_TOLERANCE   = 0.5;

    public static volatile double DESIRED_DISTANCE_FROM_APRIL_TAG_IN = 15.5;

    public static volatile double MAX_DRIVE_SPEED  = 0.5;
    public static volatile double MAX_TURN_SPEED   = 0.5;
    public static volatile double MAX_STRAFE_SPEED = 0.5;

    // ----------- AUTO WAIT TIMES------------ //

    public static volatile int INITIAL_SLEEP_MS = 4000;
    public static volatile int STEP_1_SLEEP_MS  = 1000;
    public static volatile int STEP_2_SLEEP_MS  = 800;
    public static volatile int STEP_3_SLEEP_MS  = 2000;
}
