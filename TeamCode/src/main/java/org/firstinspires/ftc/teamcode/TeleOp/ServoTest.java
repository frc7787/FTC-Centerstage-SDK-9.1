package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo - Test", group = "Test")
public class ServoTest extends OpMode {
    private static ServoImplEx testServo;

    private static double SERVO_POS = 0.0d;

    private static boolean leftBumperWasPressed, rightBumperWasPressed;

    @Override public void init() {
        leftBumperWasPressed  = false;
        rightBumperWasPressed = false;

        testServo = hardwareMap.get(ServoImplEx.class, "PixelPlacerServoRight");
    }

    @Override public void init_loop() {
        telemetry.addLine("Press Left Bumper to increment start position");
        telemetry.addLine("Press Right Bumper to decrement start position");

        telemetry.addLine("Press options (The one on the left) the reset the start position.");

        if (gamepad1.options) SERVO_POS = 0.0d;

        if (gamepad1.left_bumper) { // Increment Position
            if (!leftBumperWasPressed) {
                SERVO_POS += 0.01d;

                leftBumperWasPressed = true;
            }
        } else {
            leftBumperWasPressed = false;
        }

        if (gamepad1.right_bumper) { // Decrement Position
            if (!rightBumperWasPressed) {
                SERVO_POS -= 0.01d;

                rightBumperWasPressed = true;
            }
        } else {
            rightBumperWasPressed = false;
        }

        SERVO_POS = Range.clip(SERVO_POS, 0.0d, 1.0d);

        telemetry.addData("Servo Start Pos", SERVO_POS);
        telemetry.update();
    }

    @Override public void start() {
        testServo.setPosition(SERVO_POS);
    }

    @Override public void loop() {
        telemetry.addLine("Press Left Bumper to increment position");
        telemetry.addLine("Press Right Bumper to decrement position");

        telemetry.addLine("Press options (The one on the left) the reset the servo to it's zero position");

        if (gamepad1.options) SERVO_POS = 0.0d;

        if (gamepad1.left_bumper) { // Increment Position
           if (!leftBumperWasPressed) {
               SERVO_POS += 0.01d;

               leftBumperWasPressed = true;
           }
        } else {
            leftBumperWasPressed = false;
        }

        if (gamepad1.right_bumper) { // Decrement Position
            if (!rightBumperWasPressed) {
                SERVO_POS -= 0.01d;

                rightBumperWasPressed = true;
            }
        } else {
            rightBumperWasPressed = false;
        }

        testServo.setPosition(Range.clip(SERVO_POS, 0.0, 1.0));

        telemetry.addData("Servo Position", testServo.getPosition());
        telemetry.update();
    }

    @Override public void stop() {
        testServo.setPwmDisable();
    }
}
