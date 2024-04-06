package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.teamcode.Properties.*;
import static org.firstinspires.ftc.teamcode.Subsytems.Utility.HomingState.*;
import static org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState.*;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Properties;
import org.firstinspires.ftc.teamcode.Subsytems.Utility.HomingState;
import org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState;

public class Arm {
    public static final int WORM_SAFETY_LIMIT = 700;

    private static final double ELEVATOR_HOMING_POWER = -Math.abs(Properties.ELEVATOR_HOMING_POWER);
    private static final double WORM_HOMING_POWER     = -Math.abs(Properties.WORM_HOMING_POWER);
    private static final double WORM_BACKLASH_REMOVING_POWER = 0.25;

    private static final double SAFETY_VOLTAGE = 0.8;

    public static DcMotorImplEx wormMotor, elevatorMotor;
    private static ServoImplEx leftDoor, rightDoor;
    private static RevTouchSensor wormLimitSwitch, elevatorLimitSwitch;
    private static DigitalChannel leftOuttakeLimitSwitch, rightOuttakeLimitSwitch;

    private static AnalogInput wormPotentiometer;

    private static int elevatorTargetPos, wormTargetPos;
    private static double elevatorPower, wormPower;

    private static NormalPeriodArmState normalPeriodArmState;
    private static HomingState homingState;

    /**
     * Initializes all of the hardware for the Arm, and resets the arm state
     * @param hardwareMap The hardware map you are using to get the hardware likely "hardwareMap"
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        // Control Hub Digital Port 0
        wormLimitSwitch  = hardwareMap.get(RevTouchSensor.class, "WormLimitSwitch");
        // Expansion Hub Digital Port 0
        elevatorLimitSwitch = hardwareMap.get(RevTouchSensor.class, "ExtensionLimitSwitch");

        leftOuttakeLimitSwitch  = hardwareMap.get(DigitalChannel.class, "LeftOuttakeLimitSwitch");
        rightOuttakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "RightOuttakeLimitSwitch");

        // Expansion Hub Motor Port 0
        wormMotor  = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        // Control Hub Motor Port 3
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");

        // Expansion Hub Port 1
        leftDoor = hardwareMap.get(ServoImplEx.class, "LeftDoorServo");
        // Expansion Hub Port 2
        rightDoor = hardwareMap.get(ServoImplEx.class, "RightDoorServo");

        wormPotentiometer = hardwareMap.analogInput.get("WormPotentiometer");

        elevatorMotor.setDirection(REVERSE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
        wormMotor.setMode(STOP_AND_RESET_ENCODER);

        leftOuttakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightOuttakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        elevatorTargetPos = 0;
        wormTargetPos     = 0;

        elevatorPower = DEFAULT_ELEVATOR_POWER;
        wormPower     = DEFAULT_WORM_POWER;

        moveDeliveryTrayDoor(TRAY_DOOR_CLOSED_POS);

        normalPeriodArmState = UNKNOWN;
        homingState          = START;
    }

    public static void powerElevator(double power) {
        elevatorMotor.setPower(power);
    }

    /**
     * Function to update the state of the arm every loop, moves the arm to the target pos
     * and checks if the arm should be homing.
     */
    public static void update(boolean intaking) {
        if (wormMotor.getCurrentPosition() < WORM_SAFETY_LIMIT && !intaking) { // Delivery Tray Door Override
            moveDeliveryTrayDoor(TRAY_DOOR_CLOSED_POS);
        }

        switch (normalPeriodArmState) {
            case AT_POS:
                // If the target position is less than 5 we know we are home and should control the
                // elevator motor based on whether or not we are intaking
                if (wormMotor.getTargetPosition() <= 5) {

                    // If we are intaking we want to apply a slight negative power to the elevator
                    // to make sure that we intake pixels properly
                    if (intaking) {
                        elevatorMotor.setPower(-0.05);
                    } else { // If not we want to make sure that the elevator power is set to 0.0
                        elevatorMotor.setPower(0.0);
                    }
                }
                break;
            case TO_POS:

                // If the worm limit switch is pressed and the worm target position is 0
                // then we know we are trying to, and have reached home but somewhere along the way
                // the encoder has drifted. This means that we should stop and reset the encoder,
                // then set the target position to 0. That way drift doesn't accumulate over time.
                if (wormLimitSwitch.isPressed() && wormTargetPos == 0) {
                    wormMotor.setMode(STOP_AND_RESET_ENCODER);
                    wormMotor.setMode(RUN_USING_ENCODER);
                    rotateWorm(0);
                }

                // If the elevator limit switch is pressed and the elevator target position is 0
                // then we know we are trying, and have, reached home but somewhere along the way
                // the encoder drifted. This means that we should stop and reset the encoder, then
                // set the target position to 0. That way drift doesn't accumulate over time.
                if (elevatorLimitSwitch.isPressed() && elevatorTargetPos == 0) {
                    elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
                    elevatorMotor.setMode(RUN_USING_ENCODER);
                    extendElevator(0);
                }

                // If the target position is 0 and the limit switches are not pressed then we know
                // that we are trying to reach home but have not yet reached it. It also tells us
                // that we have to determine a safe way to return home.
                if (elevatorTargetPos == 0 && wormTargetPos == 0) {

                    // If the worm motor is greater than or equal to  the safety limit and the
                    // position of the elevator motor is greater than 30 we know that we can safely
                    // retract the elevator, and that we should so that it retracts fully and we
                    // can safely rotate the worm down.
                    if (wormMotor.getCurrentPosition() > WORM_SAFETY_LIMIT && elevatorMotor.getCurrentPosition() > 30) {
                        extendElevator(0, elevatorPower);

                    // If the position of the worm motor is less than the safety limit and the
                    // position of the elevator motor is greater than 900 we need to
                    // rotate the worm up to the safety limit so that we can retract the elevator.
                    } else if (wormMotor.getCurrentPosition() < WORM_SAFETY_LIMIT && elevatorMotor.getCurrentPosition() > 900) {
                        rotateWorm(WORM_SAFETY_LIMIT + 20, wormPower);
                        extendElevator(1000);

                    // I'm not really sure what this check is trying to accomplish. I am honestly
                    // at a loss for what this really does.
                    } else if (90 < elevatorMotor.getCurrentPosition() && elevatorMotor.getCurrentPosition() < 900) {
                        extendElevator(0, elevatorPower);
                    } else {
                        // Confused about this. Maybe this is the problem. If we get to this state
                        // we know that the worm is in a safe position, and we know that the
                        // elevator motor is greater than 90 and less than 900. So why do we need to
                        // do this check? This seems like it might cause problems
                        if (elevatorMotor.getCurrentPosition() <= 90) {
                            rotateWorm(0, wormPower);
                        }
                    }
                // If the elevator position is greater than 0 and the worm position is less than
                // the safety limit we need to rotate up to the max of the current position and the
                // safety limit so that we can extend our safely.
                } else if (elevatorTargetPos > 0 && wormMotor.getCurrentPosition() < WORM_SAFETY_LIMIT) {
                    wormTargetPos = Math.max(wormTargetPos, (WORM_SAFETY_LIMIT + 20));
                    rotateWorm(wormTargetPos, wormPower);
                // If the elevator position is NOT 0 and the worm is greater than the safety limit
                // we want to rotate the worm to whatever the target position is. Then if it is safe
                // we want to extend the elevator to whatever the target position is.
                } else {
                    rotateWorm(wormTargetPos, wormPower);

                    // If the current position of the worm motor is greater than the safety
                    // we extend the elevator to whatever the target position of the motor is
                    if (wormMotor.getCurrentPosition() > WORM_SAFETY_LIMIT) {
                        extendElevator(elevatorTargetPos, elevatorPower);
                    }

                    // I think that the following check might be a problem. This check is only
                    // trigger when the target position of the worm and elevator motors is not zero.
                    // I think that this means the state will not move to AT_POS if the target
                    // position of the elevator and the worm are both 0 (When we hit Dpad-Down) to
                    // go to 0,0.

                    // If neither the elevator motor or worm motor is busy we are at position and
                    // should change into the appropriate state
                    if (!elevatorMotor.isBusy() && !wormMotor.isBusy()) {
                        normalPeriodArmState = AT_POS;
                    }
                }
                break;
            case UNKNOWN: // If we don't know what the current state of the robot is (For example when we start TeleOp) we want to starting homing
                setHoming();
                break;
            case HOMING: // Homing sequence should only be called when the arm state is homing
                home();
                break;
        }
    }

    public static boolean bothOuttakeLimitSwitchesArePressed() {
        return !leftOuttakeLimitSwitch.getState() && !rightOuttakeLimitSwitch.getState();
    }

    /**
     * Command the arm to start the homing sequence
     */
    public static void setHoming() {
        homingState          = START;
        normalPeriodArmState = HOMING;
    }

    /**
     * Sets the target position of the arm
     * @param wormTargetPos The target position of the worm
     * @param elevatorTargetPos The target position of the elevator
     */
    public static void setTargetPos(int elevatorTargetPos,  int wormTargetPos) {
        normalPeriodArmState = TO_POS;

        Arm.elevatorTargetPos = elevatorTargetPos;
        Arm.wormTargetPos     = wormTargetPos;
    }

    public static void setElevatorPower(double power) {
        Arm.elevatorPower = power;
    }

    public static void setWormPower(double power) {
        Arm.wormPower = power;
    }

    /**
     * Sets the position for the door to move to. Note, if the worm limit switch is pressed this
     * function will be overwritten
     * @param doorPos The position for the door to go to
     */
    public static void openDeliveryTrayDoor(double doorPos) {
        moveDeliveryTrayDoor(doorPos);
    }

    /**
     * Homes the arm and worm. First, it homes the elevator, then the worm after it is finished
     */
    private static void home() {
       switch (homingState) {
            case START:
                openDeliveryTrayDoor(TRAY_DOOR_CLOSED_POS); // Make sure the tray is closed so we don't break it

                wormMotor.setMode(RUN_USING_ENCODER);
                elevatorMotor.setMode(RUN_USING_ENCODER);

                // If the worm power is greater than the safety voltage, start homing the elevator
                if (wormPotentiometer.getVoltage() >= SAFETY_VOLTAGE) {
                    wormMotor.setPower(0.0);
                    homingState = HOMING_ELEVATOR;
                } else { // If the voltage is less than the safety limit, drive the worm up,
                    wormMotor.setPower(1.0);
                    elevatorMotor.setPower(-0.05);

                    // If the limit switch is pressed, we start homing the elevator
                    if (elevatorLimitSwitchPressed()) {
                        wormMotor.setPower(0.0);
                        homingState = HOMING_ELEVATOR;
                    }
                }

                normalPeriodArmState = HOMING;
                break;
            case HOMING_ELEVATOR:
                elevatorMotor.setPower(ELEVATOR_HOMING_POWER);

                // Stop homing once the limit switch is pressed
                if (elevatorLimitSwitch.isPressed()) {
                    homingState = HOMING_WORM;
                    elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
                    wormMotor.setPower(0.0);
                }
                break;
           case HOMING_WORM:
               wormMotor.setPower(WORM_HOMING_POWER);

               // If the worm limit switch is pressed, start removing backlash
               if (wormLimitSwitch.isPressed()) {
                   homingState = REMOVING_WORM_BACKLASH;
                   wormMotor.setPower(0);
               }
               break;
           case REMOVING_WORM_BACKLASH:
               wormMotor.setPower(WORM_BACKLASH_REMOVING_POWER);

               // Remove the backlash of the worm.
               if (!wormLimitSwitch.isPressed()) {
                   homingState = COMPLETE;
                   wormMotor.setMode(STOP_AND_RESET_ENCODER);
                   rotateWorm(-10,0.0);
               }
               break;
            case COMPLETE:
                wormMotor.setMode(STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
                wormMotor.setMode(RUN_USING_ENCODER);
                elevatorMotor.setMode(RUN_USING_ENCODER);

                normalPeriodArmState = AT_POS;
                homingState = IDLE;

                elevatorTargetPos = 0;
                wormTargetPos     = 0;
                break;
            case IDLE:
                break;
       }
    }

    public static void relevantTelemetry(@NonNull Telemetry telemetry) {
        telemetry.addData("Potentiometer Voltage: ", wormPotentiometer.getVoltage());
        telemetry.addData("Arm State", normalPeriodArmState);
        telemetry.addData("Left Limit Switch", leftOuttakeLimitSwitch.getState());
        telemetry.addData("Right Limit Switch", rightOuttakeLimitSwitch.getState());
        telemetry.addData("Left Limit Switch Connection", leftOuttakeLimitSwitch.getConnectionInfo());
        telemetry.addData("Right Limit Switch Connection", rightOuttakeLimitSwitch.getConnectionInfo());
        telemetry.addData("Extension Target Position", elevatorMotor.getTargetPosition());
        telemetry.addData("Worm Target Position", wormMotor.getTargetPosition());
    }

    /**
     * Extends the elevator to the provided position at the provided power
     * @param targetPos The position to move the elevator to
     * @param power The power to move to the target position at
     */
    private static void extendElevator(int targetPos, double power) {
        elevatorMotor.setTargetPosition(targetPos);
        elevatorMotor.setMode(RUN_TO_POSITION);
        elevatorMotor.setPower(power);
    }

    /**
     * Extends the elevator to the provided position at the power defined by DEFAULT_ELEVATOR_POWER
     * @param targetPos The position to move the elevator to
     */
    private static void extendElevator(int targetPos) {
        extendElevator(targetPos, DEFAULT_ELEVATOR_POWER);
    }

    /**
     * Rotates the worm to the provided position at the provided power
     * @param pos The position to move the worm to
     * @param power The power to move to the target position at
     */
    public static void rotateWorm(int pos, double power) {
        wormMotor.setTargetPosition(pos);
        wormMotor.setMode(RUN_TO_POSITION);
        wormMotor.setPower(power);
    }

    /**
     * Rotates the worm to the the provided position at the power defined by DEFAULT_WORM_POWER
     * @param targetPos The position to move the worm to
     */
    public static void rotateWorm(int targetPos) {
        rotateWorm(targetPos, DEFAULT_WORM_POWER);
    }

    /**
     * Moves the door to the provided positions/
     * @param leftPos The position to move the left servo door
     * @param rightPos The position to move the right servo door
     */
    private static void moveDeliveryTrayDoor(double leftPos, double rightPos) {
        leftDoor.setPosition(leftPos);
        rightDoor.setPosition(rightPos);
    }

    public static void openDeliveryTrayDoorLeft(double pos) {
        leftDoor.setPosition(pos);
    }

    public static void openDeliveryTrayDoorRight(double pos) {
        rightDoor.setPosition(pos);
    }

    /**
     * Moves the door to the provided position. Note that this moves BOTH door servos
     * @param pos
     */
    private static void moveDeliveryTrayDoor(double pos) {
        moveDeliveryTrayDoor(pos, pos);
    }

    public static void debugWorm(@NonNull Telemetry telemetry) {
        telemetry.addLine("Worm Debug");

        telemetry.addData("Worm Limit Switch Is Pressed", wormLimitSwitch.isPressed());
        telemetry.addData("Worm Motor Direction", wormMotor.getDirection());
        telemetry.addData("Worm Motor Power", wormMotor.getPower());
        telemetry.addData("Worm Motor Current (Amps)", wormMotor.getCurrent(AMPS));
        telemetry.addData("Worm Current Pos", wormMotor.getCurrentPosition());
        telemetry.addData("Worm Target Pos", wormMotor.getTargetPosition());
        telemetry.addData("Elevator Motor LOCAL Target Pos", wormTargetPos);
        telemetry.addData("Worm Run Mode", wormMotor.getMode());
    }

    public static void debugElevator(@NonNull Telemetry telemetry) {
        telemetry.addLine("Elevator Debug");

        telemetry.addData("Elevator Limit Switch Is Pressed", elevatorLimitSwitch.isPressed());
        telemetry.addData("Elevator Motor Direction", elevatorMotor.getDirection());
        telemetry.addData("Elevator Motor Power", elevatorMotor.getPower());
        telemetry.addData("Elevator Motor Current (AMPS)", elevatorMotor.getCurrent(AMPS));
        telemetry.addData("Elevator Motor Current Pos", elevatorMotor.getCurrentPosition());
        telemetry.addData("Elevator Motor Target Pos", elevatorMotor.getTargetPosition());
        telemetry.addData("Elevator Motor LOCAL Target Pos", elevatorTargetPos);
        telemetry.addData("Elevator Motor Run Mode", elevatorMotor.getMode());
    }

    /**
     * Displays debug information about the delivery tray.
     * @param telemetry The telemetry to display the information on
     */
    public static void debugDeliveryTray(@NonNull Telemetry telemetry) {
        telemetry.addLine("Delivery Tray Debug");

        // NOTE: You might think that adding the direction of the servos would be useful
        // however, since these are the higher quality servos we just change it in firmware
        // so what the SDK thinks the direction is and the actual direction are different

        telemetry.addData("Left Door Servo Commanded Position", leftDoor.getPosition());
        telemetry.addData("Right Door Servo Commanded Position", rightDoor.getPosition());
    }

    /**
     * Displays debug information about the arm
     * @param telemetry The telemetry to display the information on
     */
    public static void debugArm(@NonNull Telemetry telemetry) {
        telemetry.addLine("Arm Debug");

        telemetry.addData("Arm State - Normal Period", normalPeriodArmState);
        telemetry.addData("Homing State", homingState);
        telemetry.addData("Total Arm Current (AMPS) ", elevatorMotor.getCurrent(AMPS) + wormMotor.getCurrent(AMPS));
    }

    /**
     * Displays debug information about the arm
     * @param telemetry The telemetry to display the information on
     */
    public static void debug(@NonNull Telemetry telemetry) {
      debugWorm(telemetry);
      debugElevator(telemetry);
      debugDeliveryTray(telemetry);
      debugArm(telemetry);
    }

    /**
     * @return The current position of the elevator
     */
    public static int elevatorPos() {
        return elevatorMotor.getCurrentPosition();
    }

    /**
     * @return The current position of the worm drive
     */
    public static int wormPos() {
        return wormMotor.getCurrentPosition();
    }

    /**
     * @return The target position of the worm drive
     */
    public static int wormTargetPos() {
        return wormMotor.getTargetPosition();
    }


    /**
     * @return The state of the arm in the normal period
     */
    public static NormalPeriodArmState armState() {
        return normalPeriodArmState;
    }

    /**
     * @return If the elevator limit switch is pressed
     */
    public static boolean elevatorLimitSwitchPressed() {
        return elevatorLimitSwitch.isPressed();
    }
}
