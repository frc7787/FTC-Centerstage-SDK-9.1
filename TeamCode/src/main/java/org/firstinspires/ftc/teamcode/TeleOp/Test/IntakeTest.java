package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Intake", group = "Test")
@Disabled
public class IntakeTest extends OpMode {

    private double intakePower = 1.0;

    @Override public void init() {
        Intake.init(hardwareMap);
    }

    @Override public void loop() {
        telemetry.addLine("Press left trigger to intake ; Right trigger to outtake.");
        telemetry.addLine("Press dpad down to decrease the intake power and dpad up to increase it.");
        telemetry.addData("Current Power", intakePower);

        Intake.debug(telemetry, AMPS);

        if (gamepad1.left_trigger > 0.9 || gamepad2.left_trigger > 0.9) {
            Intake.intake();
        } else if (gamepad1.right_trigger > 0.9 || gamepad2.right_trigger > 0.9) {
            Intake.outtake();
        } else {
            Intake.stop();
        }

        if ((gamepad1.dpad_up || gamepad2.dpad_up) && intakePower < 1.0) intakePower += 0.1;

        if ((gamepad1.dpad_down || gamepad2.dpad_down) && intakePower < 1.0) intakePower -= 0.1;

        telemetry.update();
    }
}
