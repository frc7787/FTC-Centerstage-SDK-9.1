package org.firstinspires.ftc.teamcode.Auto.Test;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.*;
import org.firstinspires.ftc.teamcode.Auto.Utility.PIDController;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Test - Center On April Tag", group = "Testing")
@Disabled
public class CenterOnAprilTagTest extends LinearOpMode {
    AprilTagProcessor aprilTagProcessor;
    AprilTagDetection desiredTag;
    VisionPortal visionPortal;

    final int DESIRED_TAG_ID = 5;

    final Size resolution = new Size(640, 480);

    final double yawErrTolerance     = 0.5;
    final double bearingErrTolerance = 0.8;
    final double rangeErrTolerance   = 0.5;

    int maxAprilTagDetections = 25;

    int myExposureMS   = 2;
    int myGain         = 0;
    int myWhiteBalance = 4000;

    final double DESIRED_DISTANCE = 17.5;

    final double MAX_AUTO_SPEED  = 0.5;
    final double MAX_AUTO_STRAFE = 0.5;
    final double MAX_AUTO_TURN   = 0.5;

    enum PlacingState {
        START,
        MOVING_TO_POS,
        PLACING,
        PLACED
    }

    int wormTargetPos     = 830;
    int elevatorTargetPos = 2430;

    enum POSITION {
        LEFT,
        RIGHT
    }

    PlacingState placingState = PlacingState.START;
    POSITION position = POSITION.LEFT;

    // "P" Value for drive, strafe, and turn
    final double DRIVE_GAIN  = 0.025;
    final double STRAFE_GAIN = 0.07;
    final double TURN_GAIN   = 0.05;

    // "D" Value for drive, strafe, turn
    final double DRIVE_D  = 0.0025;
    final double STRAFE_D = 0.00002;
    final double TURN_D   = 0.0008;

    double drive, strafe, turn;
    double rangeErr, yawErr, bearingErr;
    double prevRangeErr, prevYawErr, prevBearingErr;

    MecanumDriveBase driveBase;

    PIDController turnPID   = new PIDController(TURN_GAIN, 0.0, TURN_D);
    PIDController strafePID = new PIDController(STRAFE_GAIN, 0.0, STRAFE_D);
    PIDController drivePID  = new PIDController(DRIVE_GAIN, 0.0, DRIVE_D);

    @Override public void runOpMode() {
        driveBase = new MecanumDriveBase(hardwareMap);

        driveBase.init();

        Arm.init(hardwareMap);

        initVisionProcessing();

        waitForStart();

        if (isStopRequested()) return;

        centerOnAprilTag();

        placePixelOnBackdrop();
    }

    /**
     * Initializes the April Tag Processing Pipeline
     */
    public void initVisionProcessing() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(resolution)
                .setAutoStopLiveView(true)
                .build();

        while (visionPortal.getCameraState() != STREAMING) {
            if (isStopRequested() || !opModeIsActive()) return;

            telemetry.addLine("Camera Waiting.");
            telemetry.addData("Current Camera State", visionPortal.getCameraState().toString());
            telemetry.update();
        }

        setManualCameraSettings(myExposureMS, myGain, myWhiteBalance);
    }

    /**
     * Sets the exposure, gain, and whiteBalance manually
     * @param exposureMS The exposure in milliseconds
     * @param gain The gain to set
     * @param whiteBalance The color temperature to set
     */
    private void setManualCameraSettings(int exposureMS, int gain, int whiteBalance) {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(whiteBalance);
    }

    /**
     * Note, this function is blocking
     */
    private boolean detectAprilTags() {
        int count = 0;

        boolean targetDetected = false;

        while (!targetDetected) { // Wait until we detect the desired April Tag
            if (isStopRequested() || !opModeIsActive()) return false;

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            if (count > maxAprilTagDetections) {
                telemetry.addLine("No Tags Detected");
                break;
            }

            for (AprilTagDetection detection : currentDetections) {
                if (isStopRequested() || !opModeIsActive()) return false;

                if (detection.metadata == null) continue;

                if (detection.id == DESIRED_TAG_ID) { // If the tag is the one we want, stop looking
                    telemetry.addLine("Detected Desired Tag");

                    desiredTag = detection;

                    targetDetected = true;

                    break;
                }
            }

            count ++;
        }

        return targetDetected;
    }

    /**
     * Tries to center the robot on the april tag with the id specified by DESIRED_TAG_ID
     */
    @SuppressLint("DefaultLocale")
    public void centerOnAprilTag() {
        boolean isAtTarget = false;

        String errValues, driveValues;

        long start = System.currentTimeMillis();

        while (!isAtTarget) { // Wait for the robot to center on the April Tag
            if (isStopRequested() || !opModeIsActive()) return;

            // Quit loop if it takes more than 5 seconds to center
            if (System.currentTimeMillis() - start > 5000) {
                MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
                break;
            }

            if (detectAprilTags()) {
                telemetry.addLine("Centering on April Tag");

                prevRangeErr = rangeErr;
                rangeErr     = (desiredTag.ftcPose.range - DESIRED_DISTANCE);

                prevYawErr = yawErr;
                yawErr     = desiredTag.ftcPose.yaw;

                prevBearingErr = bearingErr;
                bearingErr     = desiredTag.ftcPose.bearing;

                rangeErr   = 0.9 * rangeErr   + 0.1 * prevRangeErr;
                yawErr     = 0.9 * yawErr     + 0.1 * prevYawErr;
                bearingErr = 0.9 * bearingErr + 0.1 * prevBearingErr;

                if (Math.abs(rangeErr) < rangeErrTolerance)     rangeErr   = 0.0;
                if (Math.abs(yawErr) < yawErrTolerance)         yawErr     = 0.0;
                if (Math.abs(bearingErr) < bearingErrTolerance) bearingErr = 0.0;

                drive  = Range.clip(drivePID.calculate(desiredTag.ftcPose.range, DESIRED_DISTANCE), -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(strafePID.calculate(-desiredTag.ftcPose.yaw, 0.0), -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                turn   = Range.clip(turnPID.calculate(bearingErr, 0.0), -MAX_AUTO_TURN, MAX_AUTO_TURN);

                drive *= -1.0;

                errValues   = String.format("RangeErr %f, BearingErr %f, YawErr %f", rangeErr, bearingErr, yawErr);
                driveValues = String.format("Drive %f, Strafe %f, Turn %f", drive, strafe, turn);

                telemetry.addLine(driveValues);
                telemetry.addLine(errValues);

                if (isWithinTolerance()) {
                    isAtTarget = true;
                    MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
                } else {
                    MecanumDriveBase.driveManualFF(drive, strafe, turn, 0.03);
                }
            } else {
                MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
            }

            telemetry.update();
        }
    }

    public boolean isWithinTolerance() {
        return (Math.abs(rangeErr)      < rangeErrTolerance
                && Math.abs(yawErr)     < yawErrTolerance
                && Math.abs(bearingErr) < bearingErrTolerance);
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
                    Arm.openDeliveryTrayDoor(0.3);

                    sleep(700);

                    Arm.openDeliveryTrayDoor(0.0);

                    placingState = PlacingState.PLACED;
                    break;
                case PLACED:
                    break;
            }
        }
    }
}