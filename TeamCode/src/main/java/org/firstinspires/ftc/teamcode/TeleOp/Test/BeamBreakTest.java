package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Auxiliaries;

@TeleOp(name = "Beam Break Test", group = "Test")
@Disabled
public class BeamBreakTest extends OpMode {

    LED LeftLEDRedChannel, LeftLEDGreenChannel, RightLEDRedChannel, RightLEDGreenChannel;

    boolean outtakeTrayIsFull, outtakeTrayWasFull, shouldRumble;

    @Override public void init() {
        Auxiliaries.init(hardwareMap);
        Arm.init(hardwareMap);

        LeftLEDRedChannel    = hardwareMap.get(LED.class, "LeftLEDRedChannel");
        LeftLEDGreenChannel  = hardwareMap.get(LED.class, "LeftLEDGreenChannel");
        RightLEDRedChannel   = hardwareMap.get(LED.class, "RightLEDRedChannel");
        RightLEDGreenChannel = hardwareMap.get(LED.class, "RightLEDGreenChannel");

        LeftLEDRedChannel.enable(false);
        LeftLEDGreenChannel.enable(false);
        RightLEDRedChannel.enable(false);
        RightLEDGreenChannel.enable(false);

        outtakeTrayIsFull  = false;
        outtakeTrayWasFull = false;
    }

    @Override public void loop() {
        telemetry.addLine("Pass things in front of the beam break to see if it breaks.");
        telemetry.addData("Front Beam Break Is Broken", Auxiliaries.frontBeamBreakIsBroken());
        telemetry.addData("Back Beam Break Is Broken", Auxiliaries.backBeamBreakIsBroken());

        normalPeriodLEDController();
        normalPeriodRumbleController();
    }

    void normalPeriodLEDController() {
        if (Auxiliaries.backBeamBreakIsBroken()) {
            LeftLEDGreenChannel.enable(true);
            LeftLEDRedChannel.enable(true);
        } else {
            LeftLEDRedChannel.enable(false);
            LeftLEDGreenChannel.enable(false);
        }

        if (Auxiliaries.frontBeamBreakIsBroken()) {
            RightLEDGreenChannel.enable(true);
            RightLEDRedChannel.enable(true);
        } else {
            RightLEDGreenChannel.enable(false);
            RightLEDRedChannel.enable(false);
        }
    }

    void normalPeriodRumbleController() {
        outtakeTrayWasFull = outtakeTrayIsFull;
        outtakeTrayIsFull  = Auxiliaries.frontBeamBreakIsBroken() && Auxiliaries.backBeamBreakIsBroken();

        if (outtakeTrayIsFull && !outtakeTrayWasFull) {
            gamepad1.rumbleBlips(2);
            gamepad2.rumbleBlips(2);
        }
    }
}
