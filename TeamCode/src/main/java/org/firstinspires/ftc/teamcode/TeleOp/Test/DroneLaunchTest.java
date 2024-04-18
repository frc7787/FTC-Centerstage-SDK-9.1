package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotPropertyParser;
import org.firstinspires.ftc.teamcode.Subsytems.*;

import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "Test - Drone Launcher", group = "Testing")
@Disabled
public class DroneLaunchTest extends OpMode {

    @Override public void init() {
        RobotPropertyParser.populateConstantsClass();
        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        telemetry.addLine("Arm will home on init!");
        telemetry.addLine("Arm will move to launch position, on start.");
        telemetry.update();
    }


    @Override public void start() {
        Arm.setTargetPos(0, ENDGAME_POSITION);
        Arm.update(false);
    }

    @Override public void loop() {
        telemetry.addLine("Press left trigger to release the launcher.");
        telemetry.addLine("Press right trigger to reset it.");

        Arm.update(false);

        if (gamepad2.left_trigger > 0.9) Auxiliaries.releaseLauncher();

        if (gamepad2.right_trigger > 0.9) Auxiliaries.resetLauncher();

        telemetry.update();
    }
}