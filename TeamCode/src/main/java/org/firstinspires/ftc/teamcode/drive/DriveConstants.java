package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    //public static final double TICKS_PER_REV = 1;
    //public static final double MAX_RPM = 1;

    //TestRobot Calibration Parameters
    //public static final double TICKS_PER_REV = 537.6;
    //public static final double MAX_RPM = 312;

    //Main Robot Calibration Parameters
    public static final double TICKS_PER_REV = 383.6 ;
    public static final double MAX_RPM = 435 ;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    // public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    //TestRobot Calibration Parameters
    //public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(11, 0, 0.5, 13.134);
    //public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(11, 0, 0, 0);

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    //public static double WHEEL_RADIUS = 2; // in
    //public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    //public static double TRACK_WIDTH = 1; // in

    //TestRobot Calibration Parameters
    //public static double WHEEL_RADIUS = 1.9685; // in
    //EDIT THIS FOR STRAIGHT TEST
    //public static double GEAR_RATIO = 0.5 *55.5/60.0; // output (wheel) speed / input (motor) speed
    //EDIT THIS FOR TRACK WIDTH TUNER TEST
    //public static double TRACK_WIDTH = 8.9;//9.4; // in

    //Main Robot Calibration Parameters
    public static double WHEEL_RADIUS = 1.8898; // in
    //EDIT THIS FOR STRAIGHT TEST
    public static double GEAR_RATIO = 1.0; // Test Robot : *55.5/60.0; // output (wheel) speed / input (motor) speed
    //EDIT THIS FOR TRACK WIDTH TUNER TEST
    public static double TRACK_WIDTH = 16.1;// Test Robot : 10.75;//9.4; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.0119; //0.01360; //0.013102;//1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0.0031; //0.00041;
    public static double kStatic = 0.10183; //0.10739

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */

    /*
     * Note from LearnRoadRunner.com:
     * The velocity and acceleration constraints were calculated based on the following equation:
     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
     * Test Robot : Resulting in 27.334275112574463 in/s.
     * Game Robot: Resulting in 73.17330064499293 in/s.

     * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.
     * This is capped at 85% because there are a number of variables that will prevent your bot from actually
     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiences, etc.
     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
     * max velocity. The theoreticaly maximum velocity is 32.15797072067584 in/s.
     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
     * affected if it is aiming for a velocity not actually possible.
     *
     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
     * to degrade. As of now, it simply mirrors the velocity results in 27.334275112574463 in/s/s
     *
     * Maximum Angular Velocity is calculated as: maximum velocity / (trackWidth / 2) * (180 / Math.PI)

     */

    /*public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );*/
    //TestRobot Calibration Parameters
    /*public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            27.334275112574463, 27.334275112574463, 0.0,
            Math.toRadians(333.22097872340424), Math.toRadians(333.22097872340424), 0.0
    );*/
    //85% max velocity
    /*public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            73.17330064499293, 73.17330064499293, 0.0,
            Math.toRadians(360), Math.toRadians(360), 0.0
    );*/

    //75% max velocity
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            64.5646770396996549, 64.5646770396996549, 0.0,
            Math.toRadians(360), Math.toRadians(360), 0.0
    );

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
