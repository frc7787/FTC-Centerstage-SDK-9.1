package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;


import org.firstinspires.ftc.teamcode.RobotPropertyParser;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.*;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.Properties.*;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpMain.GamePeriod.*;

@TeleOp(name = "TeleOp - Use This One", group = "Production")
public class TeleOpMain extends OpMode {
    GamePeriod gamePeriod;

    enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    LED LEDOne, LEDTwo;
    MecanumDriveBase driveBase ;
    
    @Override public void init() {
        RobotPropertyParser.populateConstantsClass();
        Intake.init(hardwareMap);
        Auxiliaries.init(hardwareMap);
        driveBase = new MecanumDriveBase(hardwareMap);
        driveBase.init();
        Arm.init(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(AUTO);
        }

        LEDOne = hardwareMap.get(LED.class, "LEDOne");
        LEDTwo = hardwareMap.get(LED.class, "LEDTwo");

        LEDOne.enable(false);
        LEDTwo.enable(false);

        gamePeriod = NORMAL;
    }

    @Override public void loop() {
        Intake.update(); // This HAS to come before Arm.update()

        Arm.update(Intake.isActive());
        //Arm.debug(telemetry);

        double drive  = gamepad1.left_stick_y * -1.0; // Left stick y is inverted
        double strafe = gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.update();

        MecanumDriveBase.driveManualFF(drive, strafe, turn, 0.02);

        switch (gamePeriod) {
           case NORMAL:
               gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
               gamepad2.setLedColor(0,0,255, Gamepad.LED_DURATION_CONTINUOUS);

               LEDOne.enable(false);
               LEDTwo.enable(false);

               normalPeriodLoop();

               if (gamepad2.share) {
                   gamePeriod = ENDGAME;
                   gamepad1.rumble(1, 1, 1000);
                   gamepad2.rumble(1, 1, 1000);
               }

               break;
           case ENDGAME:
               if (gamepad2.options) {
                   gamePeriod = NORMAL;
                   gamepad1.rumble(1,1, 1000);
                   gamepad2.rumble(1,1,1000);
               }

               gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
               gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

               LEDOne.enable(true);
               LEDTwo.enable(true);

               endgameLoop();
               break;
       }
    }

    private void endgameLoop() {
        if (gamepad2.left_bumper) Arm.setTargetPos(0, ENDGAME_POSITION);

        if (Arm.wormTargetPos() == ENDGAME_POSITION) {
            Auxiliaries.releaseHanger(); // Automatically release hanging hook when we go to hang

            if (gamepad2.left_trigger > 0.9) Auxiliaries.releaseLauncher();

            if (gamepad2.dpad_down) Arm.setTargetPos(0,0);
        }
    }

    private void normalPeriodLoop() {
        if (gamepad1.dpad_up) { // Mosaic Fixing Left Control
            Auxiliaries.movePixelPlacerToMosaicFixingPositionLeft();
        } else if (gamepad1.dpad_down) {
            Auxiliaries.retractPixelPlacerLeft();
        }

        if (gamepad1.triangle) { // Mosaic Fixing Right Control
            Auxiliaries.movePixelPlacerToMosaicFixingPositionRight();
        } else if (gamepad1.cross) {
            Auxiliaries.retractPixelPlacerRight();
        }

        if (Arm.wormPos() < Arm.WORM_SAFETY_LIMIT) { // Intake and delivery tray logic
            if (gamepad2.left_trigger > 0.9) {
                Intake.intake();
                Arm.setDoorPos(TRAY_DOOR_OPEN_POS);
            } else if (gamepad2.left_trigger < 0.9 && gamepad2.left_trigger > 0.5) {
                Intake.intake();
                Arm.setDoorPos(TRAY_DOOR_CLOSED_POS);
            } else if (gamepad2.right_trigger > 0.9) {
                Intake.outtake();
            } else {
                Intake.stop();
                Arm.setDoorPos(TRAY_DOOR_CLOSED_POS);
            }
        } else {
            if (gamepad2.right_trigger > 0.5) {
                Intake.outtake();
            } else {
                Intake.stop();
            }

            if (gamepad2.left_bumper) {
                Arm.openDeliveryTrayDoorLeft(TRAY_DOOR_OPEN_POS);
            } else {
                Arm.openDeliveryTrayDoorLeft(0.0);
            }

            if (gamepad2.right_bumper) {
                Arm.openDeliveryTrayDoorRight(TRAY_DOOR_OPEN_POS);
            } else {
                Arm.openDeliveryTrayDoorRight(0.0);
            }
        }

        if (gamepad2.dpad_down) { // Arm Logic
            Arm.setTargetPos(0,0);
        } else if (gamepad2.dpad_left) {
            Arm.setTargetPos(LOW_EXT_POS + ANGLE_OFFSET, LOW_ROT_POS);
        } else if (gamepad2.dpad_right) {
            Arm.setTargetPos(MED_EXT_POS + ANGLE_OFFSET, MED_ROT_POS);
        } else if (gamepad2.dpad_up) {
            Arm.setTargetPos(HIGH_EXT_POS + ANGLE_OFFSET, HIGH_ROT_POS);
        } else if (gamepad2.cross) {
            Arm.setTargetPos(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (gamepad2.square) {
            Arm.setTargetPos(LOW_EXT_POS, LOW_ROT_POS);
        } else if (gamepad2.circle) {
            Arm.setTargetPos(MED_EXT_POS, MED_ROT_POS);
        } else if (gamepad2.triangle) {
            Arm.setTargetPos(HIGH_EXT_POS, HIGH_ROT_POS);
        }
    }
}