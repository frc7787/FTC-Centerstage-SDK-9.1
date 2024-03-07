package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.*;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "Test - Drone Launcher", group = "Testing")
public class TeleOpDroneLaunch extends OpMode {

    @Override public void init() { Arm.init(hardwareMap); }

    @Override public void loop() {
        telemetry.addLine("Press left bumper to the endgame position");
        telemetry.addLine("Press left trigger to release the launcher, and right trigger to reset it");

        Arm.update(false);

        if (gamepad2.left_bumper) Arm.setTargetPos(0, ENDGAME_POSITION);

        if (gamepad2.left_trigger > 0.9) Auxiliaries.releaseLauncher();

        if (gamepad2.right_trigger > 0.9) Auxiliaries.resetLauncher();

        telemetry.update();
    }
}