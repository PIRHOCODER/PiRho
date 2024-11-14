package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Blue Observation", group="Robot")
public class AutoBlueObservation extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.0;
    public static final double TURN_SPEED = 0.5;
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final static double VIPER_POWER = 0.75;
    private final static double PIVOT_POWER = 0.35;
    public final boolean isFieldCentric = false;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
        waitForStart();
        intake.close();
        intake.unwhack();
        pivot.setTargetPosition(-1170);
        robotController.distanceDrive(25, -12.528807709151522, DRIVE_SPEED);
        // Pivot: -1170, viper: 1280
        viperSlide.setTargetPosition(1280);
        robotController.sleep(1);
        pivot.setTargetPosition(-1000);
        robotController.sleep(1);
        viperSlide.setTargetPosition(800);
        robotController.sleep(1);
        intake.open();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(1);
        intake.close();

        robotController.distanceDrive(27.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(24.0, -0.0, DRIVE_SPEED);
        robotController.distanceDrive(9.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(45.0, 180.0, DRIVE_SPEED);
        robotController.distanceDrive(45.0, -0.0, DRIVE_SPEED);
        robotController.distanceDrive(12.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(45.0, 180.0, DRIVE_SPEED);
        robotController.distanceDrive(45.0, -0.0, DRIVE_SPEED);
        robotController.distanceDrive(9.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(45.0, 180.0, DRIVE_SPEED);
    }

    public void initialize() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
//        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
//        claw.setDirection(Servo.Direction.REVERSE);
//        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");

        IMU gyro = hardwareMap.get(IMU.class, "imu2");

        SparkFunOTOS photoSensor = hardwareMap.get(SparkFunOTOS.class, "PHOTOSENSOR");

        viperSlide = new ViperSlide(
                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT")
        );
        pivot = new Pivot(
                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT")
        );
        intake = new Intake(hardwareMap);
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, photoSensor,this);

        telemetry.addData("Status", "Initialized");
    }
}