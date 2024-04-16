package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.poseHeadOffset;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.RFHolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.RFTrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizerLeft;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizerRight;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadrunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Trajectory-cancelable version of the simple mecanum drive hardware implementation for REV hardware.
 * Ensure that this is copied into your project.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static final PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0.0, 0.5);
    public static final PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0.1, 0.5);

    public static final double LATERAL_MULTIPLIER = 1.3;

    public static final double VX_WEIGHT = 1;
    public static final double VY_WEIGHT = 1;
    public static final double OMEGA_WEIGHT = 1;
    private TrajectorySequenceRunner trajectorySequenceRunner;
    private TrajectorySequence trajectorySeq;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private RFTrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;
    private Pose2d endPose = new Pose2d(0, 0, 0);

    private final double [] OFFSETS = {0.005,-0.005,0.01,0.02 };

    private final double BUTTERED_POSITION = 0.6;
    private final double INIT_POSITION = 1.0;

    private final double FIELD_CENTRIC_DOWNSCALE = 0.3;
    public static double INITIAL1 = 0.005, INITIAL2 = 0.005,INITIAL3 = .01, INITIAL4 = 0.9;
    public static double FINAL1 = 1.00, FINAL2 = 0.605,FINAL3 = .601, FINAL4 = 0.20;


    private boolean isButtered = false, isFieldCentric = false;

    private ArrayList<Servo> servos;

    public org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive drive;

    public SampleMecanumDrive(HardwareMap hardwareMap, Tracker.TrackType trackType) {
        super(kV,
                kA,
                kStatic,
                TRACK_WIDTH,
                TRACK_WIDTH,
                LATERAL_MULTIPLIER);
        follower = new RFHolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();



        // TODO: adjust the names of the following hardware devices to match your configuration
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.

        leftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorRightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        DcMotorEx backEncoder = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        DcMotorEx leftEncoder = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        DcMotorEx rightEncoder = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }



        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));


        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        if (trackType == Tracker.TrackType.ROADRUN_ODOMETRY) {
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        } else if (trackType == Tracker.TrackType.ROADRUN_IMU_LEFT) {
            setLocalizer(new TwoWheelTrackingLocalizerLeft(hardwareMap, this));
        } else if (trackType == Tracker.TrackType.ROADRUN_IMU_RIGHT) {
            setLocalizer(new TwoWheelTrackingLocalizerRight(hardwareMap, this));
        } else {
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        }
        BasicRobot.dashboard = FtcDashboard.getInstance();
        BasicRobot.dashboard.setTelemetryTransmissionInterval(25);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
        trajectorySeq = trajectorySequenceBuilder(getPoseEstimate()).lineTo(new Vector2d(10, 0)).build();
        servos = new ArrayList<>();
        servos.add(hardwareMap.servo.get("servoLeftFront"));
        servos.add(hardwareMap.servo.get("servoLeftBack"));
        servos.add(hardwareMap.servo.get("servoRightFront"));
        servos.add(hardwareMap.servo.get("servoRightBack"));
        isButtered=true;
        toggleButtered();
        Motor frontLeft = new Motor(hardwareMap, "motorLeftFront"),
        frontRight = new Motor(hardwareMap, "motorRightFront"),
        backLeft = new Motor(hardwareMap, "motorLeftBack"),
        backRight = new Motor(hardwareMap, "motorRightBack");

    drive =
        new org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive(
            frontLeft, frontRight, backLeft, backRight);
    currentPose = new Pose2d(0,0,0);
    poseHeadOffset = 0;

    update();

//
    }

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        this(hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY);
    }

    public StandardTrackingWheelLocalizer getTracker(){
        return (StandardTrackingWheelLocalizer)this.getLocalizer();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }
    public void driveRobotCentric(double strafeSpeed, double verticalSpeed, double turnSpeed){
        double[] buh = drive.driveFieldCentric(strafeSpeed, verticalSpeed, turnSpeed ,0.0);
        powerSpeed(buh[0], buh[1], buh[2], buh[3]);
    }

    public void powerSpeed(double frontLeftSpeed, double frontRightSpeed,
        double backLeftSpeed, double backRightSpeed) {
        double maxOutput = 1;
        leftFront.setPower(frontLeftSpeed * maxOutput);
        rightFront.setPower(frontRightSpeed * maxOutput);
        leftRear.setPower(backLeftSpeed * maxOutput);
    rightRear.setPower(backRightSpeed * maxOutput);
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );

    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("going from: " +trajectorySequence.start() +" to: " + trajectorySequence.end());
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
        trajectorySeq = trajectorySequence;
        endPose = trajectorySequence.end();
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
        trajectorySeq = trajectorySequence;

    }

    public void changeTrajectorySequence(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.changeTrajectorySequence(trajectorySequence);
        trajectorySeq = trajectorySequence;
        endPose = trajectorySequence.end();
        Trajectory tragedy = ((TrajectorySegment) trajectorySequence.get(0)).getTrajectory();
        follower.changeTrajectory(tragedy);
    }

    public void breakFollowing() {
        trajectorySequenceRunner.breakFollowing();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
//        setPoseEstimate(currentPose);
        logger.log("/RobotLogs/GeneralRobot", "curPose"+currentPose);
        updatePoseEstimate();

        logger.log("/RobotLogs/GeneralRobot", "curPose2"+currentPose);

        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public TrajectorySequence getCurrentTraj() {
        return trajectorySequenceRunner.getTrajectorySequence();
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;
        if(isButtered){
            drivePower = new Pose2d(vel.getX(),0,vel.getHeading());
            vel = new Pose2d(vel.getX(),0,vel.getHeading());

        }
        if(isFieldCentric){
            vel = vel.times(FIELD_CENTRIC_DOWNSCALE);
            drivePower = new Pose2d(vel.vec().rotated(-currentPose.getHeading()), vel.getHeading());
            vel = drivePower;
        }
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }
    public void toggleFieldCentric(){
        isFieldCentric= !isFieldCentric;
        if(isFieldCentric&&isButtered){
            toggleButtered();
        }
        LOGGER.log(RFLogger.Severity.INFO, "fieldCentric: "+isFieldCentric);
    }
    public void toggleButtered(){
        isButtered=!isButtered;
        if(isButtered&&isFieldCentric){
            toggleFieldCentric();
        }
        toggleServos();
        LOGGER.log(RFLogger.Severity.INFO, "isButtered: "+isButtered);
    }

    public boolean isButtered(){
        return isButtered;
    }


    public void toggleServos(){
            if(isButtered){
                servos.get(0).setPosition(INITIAL1);
                servos.get(1).setPosition(INITIAL2);
                servos.get(2).setPosition(INITIAL3);
                servos.get(3).setPosition(INITIAL4);
//            isButtered = false;
            }
            else{
                servos.get(0).setPosition(FINAL1);
                servos.get(1).setPosition(FINAL2);
                servos.get(2).setPosition(FINAL3);
                servos.get(3).setPosition(FINAL4);
//                isButtered = true;
            }

    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    public Pose2d getEndPose() {
        return endPose;
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+poseHeadOffset;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}