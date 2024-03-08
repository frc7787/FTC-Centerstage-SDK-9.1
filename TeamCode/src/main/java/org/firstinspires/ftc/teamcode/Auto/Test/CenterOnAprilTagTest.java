package org.firstinspires.ftc.teamcode.Auto.Test;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.*;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Test - Center On April Tag", group = "Testing")
public class CenterOnAprilTagTest extends LinearOpMode {
    AprilTagProcessor aprilTagProcessor;
    AprilTagDetection desiredTag;

    final int DESIRED_TAG_ID = 5;

    final double yawErrorToleranceInches   = 0.1d;
    final double bearingErrorToleranceDeg  = 1.0d;
    final double rangeErrorToleranceInches = 0.5d;

    VisionPortal visionPortal;

    int myExposure     = 3;
    int myGain         = 255;
    int myWhiteBalance = 4800;

    final double DESIRED_DISTANCE = 4.0d;

    final double SPEED_GAIN  =  0.008d;
    final double STRAFE_GAIN =  0.02d;
    final double TURN_GAIN   =  0.01d;

    final double MAX_AUTO_SPEED  = 1.0d;
    final double MAX_AUTO_STRAFE = 1.0d;
    final double MAX_AUTO_TURN   = 1.0d;


    @Override public void runOpMode() {
        DriveBase.init(hardwareMap);

        initVisionProcessing();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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
        double drive, strafe, turn;

        boolean targetFound   = false;
        boolean targetReached = false;

        while (!targetFound) { // Wait until we detect the desired April Tag
            if (isStopRequested() || !opModeIsActive()) return;

            DriveBase.driveManualRobotCentric(0,0,0);

            telemetry.addLine("Searching For Target");

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            if (currentDetections.isEmpty()) telemetry.addLine("No Tags Detected");

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata == null) continue;

                if (detection.id == DESIRED_TAG_ID) { // If the tag is the one we want, stop looking
                    telemetry.addLine("Detected Desired Tag");

                    desiredTag = detection;

                    targetFound = true;

                    break;
                }
            }

            telemetry.update();
        }

        while (!targetReached) { // Wait for the robot to center on the April Tag
            if (isStopRequested() || !opModeIsActive()) return;

            telemetry.addLine("Attempting To Center On April Tag");
            telemetry.update();

            double rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double bearingError = desiredTag.ftcPose.bearing;
            double yawError     = desiredTag.ftcPose.yaw;

            drive  = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(-bearingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            telemetry.addLine(String.format("RangeError %f, BearingError %f, YawError %f", rangeError, bearingError, yawError));
            telemetry.addLine(String.format("Drive %f, Strafe %f, Turn %f", drive, strafe, turn));

            DriveBase.driveManualRobotCentric(drive, strafe, turn);

            if (rangeError < rangeErrorToleranceInches && bearingError < bearingErrorToleranceDeg && yawError < yawErrorToleranceInches) {
                DriveBase.driveManualRobotCentric(0,0,0);
                targetReached = true;
            }
        }
    }
}