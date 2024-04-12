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

    LED leftLED, rightLED;
    MecanumDriveBase driveBase;

    boolean outtakeTrayWasFull, outtakeTrayIsFull;
    
    @Override public void init() {
        outtakeTrayIsFull  = false;
        outtakeTrayWasFull = false;

        RobotPropertyParser.populateConstantsClass();
        Intake.init(hardwareMap);
        Auxiliaries.init(hardwareMap);
        driveBase = new MecanumDriveBase(hardwareMap);
        driveBase.init();
        Arm.init(hardwareMap);

        Auxiliaries.retractPixelPlacerServo();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(AUTO);
        }

        Auxiliaries.retractPixelFixerLeft();
        Auxiliaries.retractFixerRight();

        leftLED  = hardwareMap.get(LED.class, "LeftLED");
        rightLED = hardwareMap.get(LED.class, "RightLED");

        leftLED.enable(false);
        rightLED.enable(false);

        gamePeriod = NORMAL;
    }

    @Override public void loop() {
        Intake.update(); // This HAS to come before Arm.update()

        Arm.update(Intake.isActive());

        MecanumDriveBase.driveManualFF(
                gamepad1.left_stick_y * -1.0,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                0.02
        );

        ledController();

        gamepadLedController();

        if (gamepad1.left_bumper) {
            Auxiliaries.disableLeftPixelPlacer();
        }

        if (gamepad1.right_bumper) {
            Auxiliaries.disableRightPixelPlacer();
        }

        if (gamepad1.square) {
            Auxiliaries.moveToFixingPositionLevelOneRight();
        } else if (gamepad1.triangle) {
            Auxiliaries.moveToFixingPositionLevelTwoRight();
        } else if (gamepad1.cross) {
            Auxiliaries.enableRightPixelPlacer();
            Auxiliaries.retractFixerRight();
        }

        if (gamepad1.dpad_left) {
            Auxiliaries.moveToFixingPositionLevelOneLeft();
        } else if (gamepad1.dpad_up) {
            Auxiliaries.moveToFixingPositionLevelTwoLeft();
        } else if (gamepad1.share) {
            Auxiliaries.moveToFixingPositionLevelThreeLeft();
        } else if (gamepad1.dpad_down) {
            Auxiliaries.enableLeftPixelPlacer();
            Auxiliaries.retractPixelFixerLeft();
        }

        switch (gamePeriod) {
           case NORMAL:
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

               endgameLoop();
               break;
       }

       telemetry.update();
    }

    void gamepadLedController() {
        switch (gamePeriod) {
            case NORMAL:
                gamepad1.setLedColor(0,0,255, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(0,0,255, Gamepad.LED_DURATION_CONTINUOUS);
            case ENDGAME:
                gamepad1.setLedColor(255,0,0, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    void ledController() {
        if (Auxiliaries.frontBeamBreakIsBroken()) {
            leftLED.enable(true);
        } else {
            leftLED.enable(false);
        }

        if (Auxiliaries.backBeamBreakIsBroken()) {
            rightLED.enable(true);
        } else {
            rightLED.enable(false);
        }
    }


    void endgameLoop() {
        if (gamepad2.left_bumper) Arm.setTargetPos(0, ENDGAME_POSITION);

        if (gamepad2.right_trigger > 0.8 || gamepad2.left_trigger > 0.8) Auxiliaries.releaseLauncher();

        if (gamepad2.right_bumper) {
            Arm.setTargetPos(0, 2500);
            Auxiliaries.releaseHanger();
        }

        if (gamepad2.dpad_down) Arm.setTargetPos(0,-280);
    }

    void normalPeriodLoop() {
        if (Arm.wormPos() < Arm.WORM_SAFETY_LIMIT) { // Intake and delivery tray logic
            if (gamepad2.left_trigger > 0.9) {
                Intake.intake();
                Arm.openDeliveryTrayDoor(TRAY_DOOR_OPEN_POS);
            } else if (gamepad2.left_trigger < 0.9 && gamepad2.left_trigger > 0.5) {
                Intake.intake();
                Arm.openDeliveryTrayDoor(TRAY_DOOR_CLOSED_POS);
            } else if (gamepad2.right_trigger > 0.9) {
                Intake.outtake();
            } else {
                Intake.stop();
                Arm.openDeliveryTrayDoor(TRAY_DOOR_CLOSED_POS);
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
        } else if (gamepad2.cross) {
            Arm.setTargetPos(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (gamepad2.square) {
            Arm.setTargetPos(LOW_EXT_POS - 10, LOW_ROT_POS - 10);
        } else if (gamepad2.circle) {
            Arm.setTargetPos(MED_EXT_POS - 180, MED_ROT_POS - 250);
        } else if (gamepad2.triangle) {
            Arm.setTargetPos(HIGH_EXT_POS, HIGH_ROT_POS);
        }
    }
}