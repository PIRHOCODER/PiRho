package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.io.File;
import java.util.List;
import java.util.Locale;

@Autonomous(name="Encoder Auto", group="4100")
public class Encoder_Auto extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor Arm = null;
    private Servo Hand = null;
    private DcMotor Intake = null;
    private DcMotor Shooter = null;
    private Servo Spanker = null;
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    BNO055IMU imu;
    final double HAND_CLOSE_POSITION = 0.6;
    final double HAND_OPEN_POSITION = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables.
        Hand = hardwareMap.get(Servo.class, "hand");
        Arm = hardwareMap.get(DcMotor.class, "arm");

        Intake = hardwareMap.get(DcMotor.class, "intake");
        Spanker = hardwareMap.get(Servo.class, "spanker");
        Shooter = hardwareMap.get(DcMotor.class, "shooter");

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        Arm.setDirection(DcMotor.Direction.REVERSE);
        Hand.setPosition(HAND_CLOSE_POSITION);
        //armMotion(true, 0.8, 400);

        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Spanker.setPosition(0.85);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        telemetry.addData("Gyro Calibration Status", imu.getCalibrationStatus().toString());

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //forward - Speed Method
            driveStraight_Encoder(700,0.5);
            //backward - Speed Method
            //driveStraight_Encoder(-500,0.3);

            stopMotion(3000);

            //forward - Distance Method
            //driveStraight_Encoder_Alternative(700,0.3);
            //backward - Distance Method
            //driveStraight_Encoder_Alternative(-600,0.3);
        }
    }

    void driveStraight_Encoder(double distance, double power) throws InterruptedException {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int straightFactor = -1;
        if(distance * power > 0) {
            straightFactor = 1;
        }
        double LFEnc = LF.getCurrentPosition();
        double RFEnc = RF.getCurrentPosition();
        double LBEnc = LB.getCurrentPosition();
        double RBEnc = RB.getCurrentPosition();
        double avgEnc = (LFEnc + LBEnc + RFEnc + RBEnc) / 4.0;
        double target = avgEnc + distance;
        final double currentAngle = aquireHeading();
        double targetAngle = currentAngle;

        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;

        if(straightFactor > 0) {
            while (avgEnc <= target - 50) {
                LFEnc = LF.getCurrentPosition();
                RFEnc = RF.getCurrentPosition();
                LBEnc = LB.getCurrentPosition();
                RBEnc = RB.getCurrentPosition();
                avgEnc = (LFEnc + LBEnc + RFEnc + RBEnc) / 4.0;
                double tempAngle = aquireHeading();
                LF_power = straightFactor * Math.abs(power);
                LB_power = straightFactor * Math.abs(power);
                RF_power = straightFactor * Math.abs(power);
                RB_power = straightFactor * Math.abs(power);
                if (tempAngle < normalizeAngle(targetAngle - 1 * 1.0)) {
                    RF_power += 0.1;
                    RB_power += 0.1;
                    LF_power -= 0.1;
                    LB_power -= 0.1;
                } else if (tempAngle > normalizeAngle(targetAngle + (1.0))) {
                    RF_power -= 0.1;
                    RB_power -= 0.1;
                    LF_power += 0.1;
                    LB_power += 0.1;
                }
                RF_power = Range.clip(RF_power, -1, 1);
                RB_power = Range.clip(RB_power, -1, 1);
                LF_power = Range.clip(LF_power, -1, 1);
                LB_power = Range.clip(LB_power, -1, 1);
                LF.setPower(LF_power);
                RF.setPower(RF_power);
                LB.setPower(LB_power);
                RB.setPower(RB_power);
                telemetry.addLine("Encoder Speed Method");
                telemetry.addData("Encoders Average:", avgEnc);
                telemetry.addData("Distance Left", target - avgEnc - 50);
                telemetry.addData("RF_power", RF_power);
                telemetry.addData("RB_power", RB_power);
                telemetry.addData("LF_power", LF_power);
                telemetry.addData("LB_power", LB_power);

                telemetry.update();
            }
        }
        else{
            while (avgEnc >= target + 50) {
                LFEnc = LF.getCurrentPosition();
                RFEnc = RF.getCurrentPosition();
                LBEnc = LB.getCurrentPosition();
                RBEnc = RB.getCurrentPosition();
                avgEnc = (LFEnc + LBEnc + RFEnc + RBEnc) / 4.0;
                double tempAngle = aquireHeading();
                LF_power = straightFactor * Math.abs(power);
                LB_power = straightFactor * Math.abs(power);
                RF_power = straightFactor * Math.abs(power);
                RB_power = straightFactor * Math.abs(power);
                if (tempAngle < normalizeAngle(targetAngle - 1 * 1.0)) {
                    RF_power += 0.1;
                    RB_power += 0.1;
                    LF_power -= 0.1;
                    LB_power -= 0.1;
                } else if (tempAngle > normalizeAngle(targetAngle + (1.0))) {
                    RF_power -= 0.1;
                    RB_power -= 0.1;
                    LF_power += 0.1;
                    LB_power += 0.1;
                }
                RF_power = Range.clip(RF_power, -1, 1);
                RB_power = Range.clip(RB_power, -1, 1);
                LF_power = Range.clip(LF_power, -1, 1);
                LB_power = Range.clip(LB_power, -1, 1);
                LF.setPower(LF_power);
                RF.setPower(RF_power);
                LB.setPower(LB_power);
                RB.setPower(RB_power);
                telemetry.addLine("Encoder Speed Method");
                telemetry.addData("Encoders Average:", avgEnc);
                telemetry.addData("Distance Left", target - avgEnc + 50);
                telemetry.addData("RF_power", RF_power);
                telemetry.addData("RB_power", RB_power);
                telemetry.addData("LF_power", LF_power);
                telemetry.addData("LB_power", LB_power);
                telemetry.update();
            }
        }
        stopMotion();

    }

    void driveStraight_Encoder_Alternative(double distance, double power) throws InterruptedException{
        int loop = 0;
        final double currentAngle = aquireHeading();
        double targetAngle = currentAngle;
        int straightFactor = -1;
        if(distance * power > 0) {
            straightFactor = 1;
        }
        double LFtarget = LF.getCurrentPosition() + straightFactor * Math.abs(distance);
        double RFtarget = RF.getCurrentPosition() + straightFactor * Math.abs(distance);
        double LBtarget = LB.getCurrentPosition() + straightFactor * Math.abs(distance);
        double RBtarget = RB.getCurrentPosition() + straightFactor * Math.abs(distance);
        LF.setTargetPosition((int) (LFtarget));
        RF.setTargetPosition((int) (RFtarget));
        LB.setTargetPosition((int) (LBtarget));
        RB.setTargetPosition((int) (RBtarget));
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (LF.isBusy()&&RF.isBusy()&&LB.isBusy()&&RB.isBusy()) {
            double tempAngle = aquireHeading();
            LF_power = straightFactor * Math.abs(power);
            LB_power = straightFactor * Math.abs(power);
            RF_power = straightFactor * Math.abs(power);
            RB_power = straightFactor * Math.abs(power);
            if (tempAngle < normalizeAngle(targetAngle - 1 * 1.0)) {
                RF_power += 0.1;
                RB_power += 0.1;
                LF_power -= 0.1;
                LB_power -= 0.1;
            } else if (tempAngle > normalizeAngle(targetAngle + (1.0))) {
                RF_power -= 0.1;
                RB_power -= 0.1;
                LF_power += 0.1;
                LB_power += 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addLine("Encoder Distance Method");
            loop++;
            if(loop == 10) {
                telemetry.addData("RF Distance Left", RFtarget - RF.getCurrentPosition());
                telemetry.addData("RB Distance Left", RBtarget - RB.getCurrentPosition());
                telemetry.addData("LF Distance Left", LFtarget - LF.getCurrentPosition());
                telemetry.addData("LB Distance Left", LBtarget - LB.getCurrentPosition());
                loop = 0;
            }
            telemetry.update();
        }
            stopMotion();
    }

    void stopMotion() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }

    void stopMotion(long timeInterval) {
        stopMotion();
        sleep(timeInterval);
    }

    double normalizeAngle(double angle) {
        double tempDeg = angle % 360;
        if (tempDeg >= 180) {
            tempDeg -= 360;
        } else if (tempDeg < -180) {
            tempDeg += 360;
        }
        return tempDeg;
    }

    double aquireHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempHead = normalizeAngle(heading);
        telemetry.addData("Heading", tempHead);
        telemetry.update();
        sleep(20);
        return tempHead;
    }

    void driveStraight(boolean isForward, double margin, double power, double timeInterval,boolean stop) throws InterruptedException {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;
            if (tempAngle < normalizeAngle(targetAngle - 1 * margin)) {
                RF_power += 0.1;
                RB_power += 0.1;
                LF_power -= 0.1;
                LB_power -= 0.1;
            } else if (tempAngle > normalizeAngle(targetAngle + (margin))) {
                RF_power -= 0.1;
                RB_power -= 0.1;
                LF_power += 0.1;
                LB_power += 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        if(stop){
            stopMotion();
        }
    }

    void drivePerpendicularly(boolean isLeft, double margin, double power, double timeInterval) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int perpendicularFactor = -1;
        if (isLeft) {
            perpendicularFactor = 1;
        }
        double targetAngle = normalizeAngle(currentAngle + 90 * perpendicularFactor);
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = -1 * perpendicularFactor * power;
            LB_power = perpendicularFactor * power;
            RF_power = perpendicularFactor * power;
            RB_power = -1 * perpendicularFactor * power;
            if (tempAngle < normalizeAngle(targetAngle - 1 * margin)) {
                RF_power += perpendicularFactor * 0.1;
                RB_power -= perpendicularFactor * 0.1;
                LF_power += perpendicularFactor * 0.1;
                LB_power -= perpendicularFactor * 0.1;
            } else if (tempAngle > normalizeAngle(targetAngle + (margin))) {
                RF_power -= perpendicularFactor * 0.1;
                RB_power += perpendicularFactor * 0.1;
                LF_power -= perpendicularFactor * 0.1;
                LB_power += perpendicularFactor * 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        stopMotion();
    }


    //make a turn that based on the current heading in a certain direction and angle
    void rotateAtAngle(boolean isClockwise, double degree, double margin, double power) {
        int angleFactor = -1;
        if (!isClockwise) {
            angleFactor = 1;
        }
        final double currentAngle = aquireHeading();
        double targetAngle = normalizeAngle(currentAngle + degree * angleFactor);
        rotateToAngle(targetAngle, margin, power);
        stopMotion();
    }

    //make a turn TO a certain angle
    void rotateToAngle(double targetAngle, double margin, double power) {
        int angleFactor = 0;
        final double currentAngle = aquireHeading();
        if (currentAngle - targetAngle > 0) {
            if (currentAngle - targetAngle < 180) {
                //cw
                angleFactor = -1;
            } else {
                //ccw
                angleFactor = 1;
            }
        } else {
            if (targetAngle - currentAngle < 180) {
                //ccw
                angleFactor = 1;
            } else {
                //cw
                angleFactor = -1;
            }
        }
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        double tempAngle = currentAngle;
        while (!((tempAngle < targetAngle + margin) && (tempAngle > targetAngle - margin))) {
            tempAngle = aquireHeading();
            RF_power = angleFactor * power;
            RB_power = angleFactor * power;
            LF_power = -1 * angleFactor * power;
            LB_power = -1 * angleFactor * power;
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        stopMotion();
    }

    boolean armMotion(boolean moveUp, double power, double timeInterval) {
        ElapsedTime armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        int upFactor = -1;
        if (!moveUp) {
            upFactor = 1;
        }
        while (armTime.milliseconds() <= timeInterval) {
            if (armTime.milliseconds() <= timeInterval - 300) {
                power -= upFactor * 0.02;
            }
            Arm.setPower(Math.abs(power) * (upFactor));
            sleep(20);
        }
        Arm.setPower(0);
        return true;
    }

    boolean handMotion(boolean close) {
        if (close) {
            Hand.setPosition(HAND_CLOSE_POSITION);
        } else {
            Hand.setPosition(HAND_OPEN_POSITION);
        }
        telemetry.addData("Hand Position:", Hand.getPosition());
        telemetry.update();
        return true;
    }
}
