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
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
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

    private static DcMotorImplEx wormMotor, elevatorMotor;
    private static ServoImplEx leftDoor, rightDoor;
    private static RevTouchSensor wormLimitSwitch, elevatorLimitSwitch;
    private static AnalogInput wormPotentiometer;

    private static int elevatorTargetPos, wormTargetPos;

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

        elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
        wormMotor.setMode(STOP_AND_RESET_ENCODER);

        elevatorTargetPos = 0;
        wormTargetPos     = 0;

        moveDeliveryTrayDoor(TRAY_DOOR_CLOSED_POS);

        normalPeriodArmState = UNKNOWN;
        homingState          = START;
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
                if (elevatorMotor.getTargetPosition() == 0 && wormMotor.getTargetPosition() == 0 && !intaking) {
                    elevatorMotor.setPower(0.0);
                    wormMotor.setPower(0.0);

                } else if (elevatorMotor.getTargetPosition() == 0 && wormMotor.getTargetPosition() == 0) {
                    elevatorMotor.setPower(0.1);
                    wormMotor.setPower(0.0);
                }
                else {
                    elevatorMotor.setPower(0.2);
                }
            case TO_POS:
                // Make sure we follow a safe sequence back
                if (elevatorTargetPos == 0 && wormTargetPos == 0){
                    if (wormMotor.getCurrentPosition() > WORM_SAFETY_LIMIT && elevatorMotor.getCurrentPosition() > 10) {
                        extendElevator(0);
                    } else if (wormMotor.getCurrentPosition() < WORM_SAFETY_LIMIT && elevatorMotor.getCurrentPosition() > 900){
                        rotateWorm(WORM_SAFETY_LIMIT + 20);
                        extendElevator(1000);
                    } else if (10 < elevatorMotor.getCurrentPosition() && elevatorMotor.getCurrentPosition() < 900) {
                        extendElevator(0);
                    } else {
                        if (elevatorMotor.getCurrentPosition() < 10) {
                            extendElevator(0);
                            rotateWorm(0);
                        }
                    }
                } else if (elevatorTargetPos > 0 && wormMotor.getCurrentPosition() < WORM_SAFETY_LIMIT) {
                    // If the target worm pos is greater than the safety limit we go there, if not we go to the safety limit.
                    wormTargetPos = Math.max(wormTargetPos, (WORM_SAFETY_LIMIT + 20)); // We want to overshoot a little bit to improve consistency
                    rotateWorm(wormTargetPos);
                } else {
                    rotateWorm(wormTargetPos);
                    if (wormMotor.getCurrentPosition() > WORM_SAFETY_LIMIT){
                        extendElevator(elevatorTargetPos);
                    }
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

    /**
     * Sets the position for the door to move to. Note, if the worm limit switch is pressed this
     * function will be overwritten
     * @param doorPos The position for the door to go to
     */
    public static void setDoorPos(double doorPos) {
        moveDeliveryTrayDoor(doorPos);
    }

    /**
     * Homes the arm and worm. First, it homes the elevator, then the worm after it is finished
     */
    private static void home() {
       switch (homingState) {
            case START:
                setDoorPos(TRAY_DOOR_CLOSED_POS); // Make sure the tray is closed so we don't break it

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
                    extendElevator(0,0.0);
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
                normalPeriodArmState = AT_POS;
                homingState = IDLE;
                break;
            case IDLE:
                break;
       }
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
