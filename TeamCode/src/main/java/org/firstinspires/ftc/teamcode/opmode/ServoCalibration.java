package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;

@TeleOp(name="ServoCalibration", group="Linear Opmode")
public class ServoCalibration extends LinearOpMode {

    // declaring the only servo needed for this program
    private Servo servo = null;

    // declare controller variable, which will be used to control setting the servo to
    // different positions
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();

    // decimal format to make telemetry easier to read
    DecimalFormat df = new DecimalFormat("#.##");



    // main method
    public void runOpMode() {

        // initialize the servo called "servo"
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();


        while (opModeIsActive()) { //---------------PRESSES PLAY---------------

            // method to update gamepads once per cycle
            try {
                previousGamepad.copy(currentGamepad);
                currentGamepad.copy(gamepad1);
            } catch (Exception e) {
            }

            // simple ifs for buttons and basic positions
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up)
                servo.setPosition(1);
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down)
                servo.setPosition(0);
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left)
                servo.setPosition(Math.max(servo.getPosition() - 0.1, 0));
            if (currentGamepad.dpad_right && !previousGamepad.dpad_right)
                servo.setPosition(Math.min(servo.getPosition() + 0.1, 1));
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper)
                servo.setPosition(Math.max(servo.getPosition() - 0.01, 0));
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper)
                servo.setPosition(Math.min(servo.getPosition() + 0.01, 1));

            // telemetry just to see the position
            telemetry.addData("Servo Position: ", df.format(servo.getPosition()));
            telemetry.update();

        }
    }
}
