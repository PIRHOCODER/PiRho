package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.CenterstagePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class BaseAuto extends LinearOpMode {
    // the webcam
    OpenCvWebcam webcam;
    // the pipeline
    CenterstagePipeline pipeline = new CenterstagePipeline(telemetry);

    protected SampleMecanumDrive drive;
    protected ScoringMechanism mechanism = new ScoringMechanism();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        mechanism.init(hardwareMap, true);

        initCameraStuff();
        CenterstagePipeline.detectionStates state = CenterstagePipeline.detectionStates.ONE;
        while (!opModeIsActive() && !isStopRequested()) {
            mechanism.closeRightClaw();
            mechanism.closeLeftClaw();
            mechanism.lowerPlaneLauncher();

            state = pipeline.getState();
            telemetry.addData("Detected State", state);
            telemetry.update();
        }

        // simple conditional for setting the starting pose, because I think its necessary???

        Pose2d startPose = startPoseSetter();
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = trajectorySequenceBuilder(state);
        waitForStart();
        drive.followTrajectorySequenceAsync(sequence);
        while (opModeIsActive()) {
            drive.update();
        }
    }

    public abstract TrajectorySequence trajectorySequenceBuilder(CenterstagePipeline.detectionStates detectionState);
    public abstract Pose2d startPoseSetter();

    // Initializes the camera stuff.
    public void initCameraStuff() {
        // Instantiate an OpenCvCamera object for the camera we'll be using.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Specify the pipeline we want to use.
        webcam.setPipeline(pipeline);

        // Open the connection to the camera device.
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Tell the webcam to start streaming images to us.
                webcam.startStreaming(640, 480);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened.
            }
        });
    }

    // Used to store custom Pose2D that uses degrees instead of radians.
    // This makes it easy to edit positions from the FTC Dashboard.
    public static class customPose2D {
        public double x;
        public double y;
        public double h;
        public double tan;

        public customPose2D(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public customPose2D(double x, double y, double h, double tan) {
            this.x = x;
            this.y = y;
            this.h = h;
            this.tan = tan;
        }
    }
}
