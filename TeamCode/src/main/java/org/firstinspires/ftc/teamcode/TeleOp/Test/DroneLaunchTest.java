package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotPropertyParser;
import org.firstinspires.ftc.teamcode.Subsytems.*;

import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "Test - Drone Launcher", group = "Testing")
public class DroneLaunchTest extends OpMode {
    private int LAUNCH_POSITION = 2000;

    @Override public void init() {
        RobotPropertyParser.populateConstantsClass();
        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        telemetry.addLine("Arm will home on init!");
        telemetry.addLine("Arm will move to launch position, on start.");
        telemetry.update();
    }

    @Override public void loop() {
        Arm.setTargetPos(0, LAUNCH_POSITION);

        telemetry.addLine("Press left trigger to release the launcher.");
        telemetry.addLine("Press right trigger to reset it.");
        telemetry.addLine("Press dpad up and dpad down to adjust the launch angle");

        Arm.update(false);

        if (gamepad1.left_trigger > 0.9 || gamepad2.left_trigger > 0.9) {
            Auxiliaries.releaseLauncher();
        }

        if (gamepad1.right_trigger > 0.9 || gamepad2.right_trigger > 0.9) {
            Auxiliaries.resetLauncher();
        }

        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            LAUNCH_POSITION += 10;
        }

        if (gamepad1.dpad_down || gamepad2.dpad_down) {
            LAUNCH_POSITION -= 10;
        }

        telemetry.addData("Launch Position", LAUNCH_POSITION);

        Arm.update(false);
        telemetry.update();
    }
}