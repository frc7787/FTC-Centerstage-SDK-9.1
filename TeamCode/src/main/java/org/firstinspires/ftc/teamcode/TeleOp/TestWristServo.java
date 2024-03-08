package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test - Wrist Servo", group = "Test")
public class TestWristServo extends OpMode {
    ServoImplEx wristServo;

    double SERVO_POS = 0.0;

    boolean leftBumperWasPressed  = false;
    boolean rightBumperWasPressed = false;

    @Override public void init() {
        // Expansion Hub Port 0
        wristServo = hardwareMap.get(ServoImplEx.class, "PixelPlacerServoRight");

        wristServo.setPosition(0.0);
    }

    @Override public void loop() {
        telemetry.addLine("Press Left Bumper to increment position");
        telemetry.addLine("Press Right Bumper to increment position");

        telemetry.addLine("Press options (The one on the left) the reset the servo to zero");

        if (gamepad1.options) SERVO_POS = 0.0;

        if (gamepad1.left_bumper) { // Increment Position
           if (!leftBumperWasPressed) {
               SERVO_POS += 0.01;

               leftBumperWasPressed = true;
           }
        } else {
            leftBumperWasPressed = false;
        }

        if (gamepad1.right_bumper) { // Decrement Position
            if (!rightBumperWasPressed) {
                SERVO_POS -= 0.01;

                rightBumperWasPressed = true;
            }
        } else {
            rightBumperWasPressed = false;
        }

        wristServo.setPosition(Range.clip(SERVO_POS, 0.0, 1.0));

        telemetry.addData("Servo Commanded Position", wristServo.getPosition());
        telemetry.update();
    }
}
