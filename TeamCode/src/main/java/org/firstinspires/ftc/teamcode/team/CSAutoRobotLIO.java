package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.firstinspires.ftc.teamcode.team.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team.subsystems.ExpansionHubs;
import org.firstinspires.ftc.teamcode.team.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.RobotStateEstimator;


/**
 * Motor naming convention:
 *     Drivetrain
 *         Front Left Wheel -> LF
 *         Back Left Wheel   -> LR
 *         Front Right Wheel -> RF
 *         Back Right Wheel  -> RR
 *      Elevator
 *         Left Motor -> Elev Left
 *         Right Motor -> Elev Right
 *     Arm
 *         Arm Servo  -> Arm
 *      Claw
 *          Gripper -> Claw
 * Misc. sensors naming convention:

 */
public class CSAutoRobotLIO {
    //private  RevBlinkinLedDriver lights;
    private TimeProfiler matchRuntime;
    private ExpansionHubs expansionHubs;
    private RobotStateEstimator robotStateEstimator;
    private Drive drive;
    private LiftSubsystem liftSubsystem;
    private OuttakeSubsystem outtakeSubsystem;
    private DroneSubsystem droneSubsystem;
    private RevMotor[] motors;
    private RevServo[] servos;


    public void init(HardwareMap hardwareMap) {

//        setExpansionHubs(new ExpansionHubs(this,
//                hardwareMap.get(ExpansionHubEx.class, "Control Hub"),
//                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
//        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Elev Left")), false, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 0.7402879093),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Elev Right")), false, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 0.7402879093)
        });

        setServos(new RevServo[] {
                new RevServo((ExpansionHubServo)(hardwareMap.get("Arm"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("Claw")))
        });

        setLiftSubsystem(new LiftSubsystem(getMotors()[0], getMotors()[1]));
        setOuttakeSubsystem(new OuttakeSubsystem(getServos()[0]));
        setDroneSubsystem(new DroneSubsystem(getServos()[1]));
        setMatchRuntime(new TimeProfiler(false));
    }
    public RevMotor[] getMotors() {
        return motors;
    }

    public void setMotors(RevMotor[] motors) {
        this.motors = motors;
    }

    public RevServo[] getServos() {
        return servos;
    }

    public void setServos(RevServo[] servos) {
        this.servos = servos;
    }

    public ExpansionHubs getExpansionHubs() {
        return expansionHubs;
    }

    public void setExpansionHubs(ExpansionHubs expansionHubs) {
        this.expansionHubs = expansionHubs;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    public LiftSubsystem getLiftSubsystem() {
        return liftSubsystem;
    }

    public void setLiftSubsystem(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;
    }

    public OuttakeSubsystem getArmSubsystem() {
        return outtakeSubsystem;
    }

    public void setOuttakeSubsystem(OuttakeSubsystem outtakeSubsystem){
        this.outtakeSubsystem = outtakeSubsystem;
    }

    public DroneSubsystem getDroneSubsystem() {
        return droneSubsystem;
    }

    public void setDroneSubsystem(DroneSubsystem droneSubsystem){
        this.droneSubsystem = droneSubsystem;
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

    public Pose2d getRobotPose() {
        return getRobotStateEstimator().getPose();
    }

    public double getRobotSpeed() {
        return getRobotStateEstimator().getVelocityPose().getTranslation().norm() +
                Math.abs(getRobotStateEstimator().getVelocityPose().getRotation().getRadians());
    }
}