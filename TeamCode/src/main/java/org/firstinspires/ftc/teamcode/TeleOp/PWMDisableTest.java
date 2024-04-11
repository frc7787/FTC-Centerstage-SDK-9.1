package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "PWM - Disable Test")
public class PWMDisableTest extends OpMode {

    ServoImplEx testServo;

    @Override public void init() {
        testServo = hardwareMap.get(ServoImplEx.class, "PixelFixerLeft");

        testServo.setDirection(Servo.Direction.REVERSE);

        testServo.setPosition(0.5);
    }

    @Override public void loop() {
        if (gamepad1.left_bumper) {
            testServo.setPwmDisable();
        } else if (gamepad1.dpad_down) {
            testServo.setPwmEnable();
            testServo.setPosition(0);
        }
    }
}