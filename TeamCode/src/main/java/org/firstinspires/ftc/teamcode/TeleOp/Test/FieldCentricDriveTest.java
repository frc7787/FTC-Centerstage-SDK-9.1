package org.firstinspires.ftc.teamcode.TeleOp.Test;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;

@TeleOp(name = "Test - Field Centric", group = "Test")
public class FieldCentricDriveTest extends OpMode {

    @Override public void init() {
        DriveBase.init(hardwareMap);
    }

    @Override public void loop() {
        double drive  = gamepad1.left_stick_y * -1.0;
        double strafe = gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

        DriveBase.driveManualFieldCentric(drive, strafe, turn);

        DriveBase.debug(telemetry, AMPS);
    }
}
