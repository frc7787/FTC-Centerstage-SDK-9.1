package org.firstinspires.ftc.teamcode.Auto.Test;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.*;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Test - Center On April Tag", group = "Testing")
public class CenterOnAprilTagTest extends LinearOpMode {
    AprilTagProcessor aprilTagProcessor;
    AprilTagDetection desiredTag;
    VisionPortal visionPortal;

    final int DESIRED_TAG_ID = 5;

    final double yawErrTolerance     = 0.1;
    final double bearingErrTolerance = 0.4;
    final double rangeErrTolerance   = 1.0;

    int myExposure     = 3;
    int myGain         = 255;
    int myWhiteBalance = 4800;

    final double DESIRED_DISTANCE = 24.0;

    final double SPEED_GAIN  =  0.025;
    final double STRAFE_GAIN =  0.02;
    final double TURN_GAIN   =  0.006;

    final double MAX_AUTO_SPEED  = 1.0;
    final double MAX_AUTO_STRAFE = 1.0;
    final double MAX_AUTO_TURN   = 1.0;

    double drive, strafe, turn;
    double rangeErr, yawErr, bearingErr;

    boolean targetFound;

    MecanumDriveBase driveBase;

    @Override
    public void runOpMode() {
        driveBase = new MecanumDriveBase(hardwareMap);

        driveBase.init();

        initVisionProcessing();

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            centerOnAprilTag();
        }
    }

    /**
     * Initializes the April Tag Processing Pipeline
     */
    public void initVisionProcessing() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                .build();

        aprilTagProcessor.setDecimation(4);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setAutoStopLiveView(true)
                .build();

        while (visionPortal.getCameraState() != STREAMING) {
            telemetry.addLine("Camera Waiting.");
            telemetry.addData("Current Camera State", visionPortal.getCameraState().toString());
            telemetry.update();
        }

        setManualCameraSettings(myExposure, myGain, myWhiteBalance);
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
     * Tries to center the robot on the april tag with the id specified by DESIRED_TAG_ID
     */
    @SuppressLint("DefaultLocale")
    public void centerOnAprilTag() {
        boolean targetReached = false;

        String errValues, driveValues;

        while (!targetReached) { // Wait for the robot to center on the April Tag
            detectAprilTags(); // Get all of the April Tags; Blocking

            if (isStopRequested() || !opModeIsActive()) return;

            telemetry.addLine("Attempting To Center On April Tag");
            telemetry.update();

            rangeErr  = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            bearingErr = desiredTag.ftcPose.bearing;
            yawErr     = desiredTag.ftcPose.yaw;

            drive  = Range.clip(rangeErr * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED) * -1.0;
            strafe = Range.clip(-yawErr * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(-bearingErr * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            driveValues = String.format("RangeErr %f, BearingErr %f, YawErr %f", rangeErr, bearingErr, yawErr);
            errValues   = String.format("Drive %f, Strafe %f, Turn %f", drive, strafe, turn);

            telemetry.addLine(driveValues);
            telemetry.addLine(errValues);

            MecanumDriveBase.driveManualFF(drive, strafe, turn, 0.01);

            if (isWithinTargetTolerance()) {
                MecanumDriveBase.driveManualFF(0,0,0, 0.0);

                targetReached = true;
            }

            telemetry.update();
        }
    }

    /**
     * Note, this function is blocking
     */
    private void detectAprilTags() {
        int checkCount = 0;

        while (!targetFound) { // Wait until we detect the desired April Tag
            if (isStopRequested() || !opModeIsActive()) return;

            // Make sure that we don't waste to much time checking
            if (checkCount > 1000) MecanumDriveBase.driveManualFF(0.0,0.0,0.0, 0.0);

            telemetry.addLine("Searching For Target");

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            if (currentDetections.isEmpty()) telemetry.addLine("No Tags Detected");

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata == null) continue;

                if (detection.id == DESIRED_TAG_ID) { // If the tag is the one we want, stop looking
                    telemetry.addLine("Detected Desired Tag");
                    telemetry.update();

                    desiredTag = detection;

                    targetFound = true;

                    return;
                }
                telemetry.update();
            }

            checkCount += 1;
        }

        targetFound = false;
    }

    /**
     * Checks whether we are within the tolerances defined by rangeErrTolerance, bearingErrTolerance,
     * and yawErrTolerance
     * @return True if we are within tolerance, false if we are not
     */
    private boolean isWithinTargetTolerance() {
       return rangeErr < rangeErrTolerance && bearingErr < bearingErrTolerance && yawErr < yawErrTolerance;
    }
}