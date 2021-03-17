package org.firstinspires.ftc.robot_utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Vals {
    public static double flywheel_kp = 0.06; //40
    public static double flywheel_ki = 0.3;
    public static double flywheel_kd = 0.007;
    public static double flywheel_ks = 0;
    public static double flywheel_kv = 0.03;
    public static double flywheel_ff = 0.00827;
    public static double flywheel_tolerance = 1;
    public static double flywheel_speed = 1200;
    public static double flywheel_direction = -1;
    public static int flywheel_ready_ticks = 5;
    public static int flywheel_ready_min_speed = 1110;
    public static int flywheel_ready_max_speed = 1200;
    public static double flywheel_filtered_speed = 0;
    public static int flywheel_max_achievable_ticks = 1360;

    public static double rotate_kp = .015;
    public static double rotate_ki = .11;
    public static double rotate_kd = .0003;
    public static double rotate_tolerance = 1;
    public static double rotate_target = 0;

    public static final double TICKS_PER_INCH_MOVEMENT = 40.58;

    public static double hitter_start = 0.4;
    public static double hitter_end = .7;

    public static double wobble_arm_speed = 0.8;
    public static double wobble_hand_open = 1.0;
    public static double wobble_hand_close = 0;

}
