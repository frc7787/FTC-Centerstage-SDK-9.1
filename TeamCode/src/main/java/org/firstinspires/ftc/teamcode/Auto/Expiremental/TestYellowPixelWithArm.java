package org.firstinspires.ftc.teamcode.Auto.Expiremental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState;

@Autonomous(name = "Test - Place Yellow Pixel With Arm")
@Disabled
public class TestYellowPixelWithArm extends LinearOpMode {

    enum PlacingState {
        START,
        MOVING_TO_POS,
        PLACING,
        PLACED
    }

    int wormTargetPos     = 810;
    int elevatorTargetPos = 2400;

    enum POSITION {
        LEFT,
        RIGHT
    }

    PlacingState placingState = PlacingState.START;
    POSITION position = POSITION.LEFT;

    @Override public void runOpMode() {
        Arm.init(hardwareMap);

        while (opModeInInit()) {

            telemetry.addLine("Press left bumper for left placement, and right for right.");
            telemetry.addData("Current Placement", position);

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                position = POSITION.LEFT;
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                position = POSITION.RIGHT;
            }

            telemetry.update();
        }

        waitForStart();

        placePixelOnBackdrop();
    }

    private void placePixelOnBackdrop() {
        while (placingState != PlacingState.PLACED) {
            if (isStopRequested() || !opModeIsActive()) return;

            Arm.update(false);

            telemetry.addData("Placing State", placingState);
            telemetry.update();

            switch (placingState) {
                case START:
                    Arm.setTargetPos(elevatorTargetPos, wormTargetPos);

                    placingState = PlacingState.MOVING_TO_POS;
                    break;
                case MOVING_TO_POS:
                    if (Arm.state() == NormalPeriodArmState.AT_POS) {
                        placingState = PlacingState.PLACING;
                    }
                    break;
                case PLACING:
                    switch (position) {
                        case LEFT:
                            Arm.openDeliveryTrayDoorLeft(0.4);
                        case RIGHT:
                            Arm.openDeliveryTrayDoorLeft(0.4);
                    }

                    sleep(1000);

                    switch (position) {
                        case LEFT:
                            Arm.openDeliveryTrayDoorRight(0.0);
                        case RIGHT:
                            Arm.openDeliveryTrayDoorRight(0.0);
                    }

                    placingState = PlacingState.PLACED;
                    break;
                case PLACED:
                    break;
            }
        }
    }
}
