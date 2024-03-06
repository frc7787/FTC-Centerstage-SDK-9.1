package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.*;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "TeleOp drone testing", group = "Testing")
public class TeleOpDroneLaunch extends OpMode {

    @Override public void init() {
        Arm.init(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(AUTO);
        }
    }

    @Override public void loop() {
        Arm.update(false);

        if (gamepad2.left_bumper) {
            Arm.setTargetPos(0, ENDGAME_POSITION);
        }

        if (gamepad2.left_trigger > 0.9) {
            Auxiliaries.releaseLauncher();
        }
        if (gamepad2.right_trigger > 0.9) {
            Auxiliaries.resetLauncher();
        }

        telemetry.update();
    }
}