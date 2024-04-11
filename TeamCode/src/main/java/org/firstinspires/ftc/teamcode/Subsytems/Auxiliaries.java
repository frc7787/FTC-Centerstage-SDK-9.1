package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.HANGER_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCHER_SERVO_LAUNCH_POS;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCHER_SERVO_ZERO_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auxiliaries {
    public static double PIXEL_PLACER_SERVO_SPIKE_STRIP_POS = 0.38;
    public static double RETRACT_PURPLE_PIXEL_PLACER_SERVO  = 0.5;

    private static ServoImplEx hangerServo, launcherServo, purplePixelPlacer, mosaicFixingServoLeft, mosaicFixingServoRight;

    private static DigitalChannel frontBeamBreak, backBeamBreak;

    /**
     * Initializes all of the Auxiliary systems including the plane launcher, hanger, and pixel placer
     * servos
     * @param hardwareMap The hardware map you are using, likely "hardwareMap"
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        hangerServo            = hardwareMap.get(ServoImplEx.class, "HangerServo");
        launcherServo          = hardwareMap.get(ServoImplEx.class, "LauncherServo");
        purplePixelPlacer      = hardwareMap.get(ServoImplEx.class, "PurplePixelPlacer");
        mosaicFixingServoLeft  = hardwareMap.get(ServoImplEx.class, "PixelFixerLeft");
        mosaicFixingServoRight = hardwareMap.get(ServoImplEx.class, "PixelFixerRight");

        frontBeamBreak = hardwareMap.get(DigitalChannel.class, "FrontBeamBreak");
        backBeamBreak  = hardwareMap.get(DigitalChannel.class, "BackBeamBreak");

        frontBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        backBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        purplePixelPlacer.setDirection(Servo.Direction.REVERSE);
        mosaicFixingServoLeft.setDirection(Servo.Direction.REVERSE);

        launcherServo.setDirection(Servo.Direction.REVERSE);
        launcherServo.setPosition(LAUNCHER_SERVO_ZERO_POS);

        hangerServo.setDirection(Servo.Direction.REVERSE);
        hangerServo.setPosition(0.0);
    }

    public static void disableLeftPixelPlacer() {
        mosaicFixingServoLeft.setPwmDisable();
    }

    public static void disableRightPixelPlacer() {
        mosaicFixingServoRight.setPwmDisable();
    }

    public static void enableLeftPixelPlacer() {
        mosaicFixingServoLeft.setPwmEnable();
    }

    public static void enableRightPixelPlacer() {
        mosaicFixingServoRight.setPwmEnable();
    }

    public static boolean frontBeamBreakIsBroken() {
        return !frontBeamBreak.getState();
    }

    public static boolean backBeamBreakIsBroken() {
        return !backBeamBreak.getState();
    }


    /**
     * Releases the plane launcher
     */
    public static void releaseLauncher() {
        launcherServo.setPosition(LAUNCHER_SERVO_LAUNCH_POS);
    }

    /**
     * Resets the plane launcher
     */
    public static void resetLauncher() {
        launcherServo.setPosition(LAUNCHER_SERVO_ZERO_POS);
    }

    /**
     * Releases the hanger mechanism
     */
    public static void releaseHanger() {
        hangerServo.setPosition(HANGER_SERVO_POSITION);
    }

    /**
     * Places the left pixel placer (From the back of the robot) on the spike strip
     */
    public static void placePixelOnSpikeStrip() {
        purplePixelPlacer.setPosition(PIXEL_PLACER_SERVO_SPIKE_STRIP_POS);
    }

    public static void retractPixelPlacerServo() {
        purplePixelPlacer.setPosition(RETRACT_PURPLE_PIXEL_PLACER_SERVO);
    }

    public static void moveToFixingPositionLevelOneLeft() {
        mosaicFixingServoLeft.setPosition(0.62);
    }

    public static void moveToFixingPositionLevelTwoLeft() {
        mosaicFixingServoLeft.setPosition(0.59);
    }

    public static void moveToFixingPositionLevelThreeLeft() {
        mosaicFixingServoLeft.setPosition(0.54);
    }

    public static void moveToFixingPositionLevelOneRight() {
        mosaicFixingServoRight.setPosition(0.61);
    }

    public static void moveToFixingPositionLevelTwoRight() {
        mosaicFixingServoRight.setPosition(0.55);
    }

    public static void retractPixelFixerLeft() {
        mosaicFixingServoLeft.setPosition(0.0);
    }

    public static void retractFixerRight() {
        mosaicFixingServoRight.setPosition(0.0);
    }

    public static void debugHanger(@NonNull Telemetry telemetry) {
        telemetry.addLine("Hanger Debug");

        telemetry.addData("Hanger Servo Position", hangerServo.getPosition());
        telemetry.addData("Hanger Servo PWM Range", hangerServo.getPwmRange().toString());
        telemetry.addData("Hanger Servo Direction", hangerServo.getDirection());
    }

    public static void debugLauncher(@NonNull Telemetry telemetry) {
        telemetry.addLine("Launcher Debug");

        telemetry.addData("Launcher Servo Position", launcherServo.getPosition());
        telemetry.addData("Launcher Servo PWM Range", launcherServo.getPwmRange().toString());
        telemetry.addData("Launcher Servo Direction", launcherServo.getDirection());
    }

    public static void debugPixelPlacers(@NonNull Telemetry telemetry) {
        telemetry.addLine("Purple Pixel Placer Servo Debug");

        telemetry.addData("Left Pixel Placer Servo Target Position", purplePixelPlacer.getPosition());
        telemetry.addData("Right Pixel Placer Servo Target Position", mosaicFixingServoLeft.getPosition());
    }

    public static void debugAll(@NonNull Telemetry telemetry) {
        debugHanger(telemetry);
        debugLauncher(telemetry);
        debugPixelPlacers(telemetry);
    }
}
