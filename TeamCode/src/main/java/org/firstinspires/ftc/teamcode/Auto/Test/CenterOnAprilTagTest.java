package org.firstinspires.ftc.teamcode.Auto.Test;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Test - Center On April Tag", group = "Linear OpMode")
public class CenterOnAprilTagTest extends LinearOpMode {
    AprilTagProcessor aprilTagProcessor;
    AprilTagDetection desiredTag;

    final int DESIRED_TAG_ID = 5;

    VisionPortal visionPortal;

    int myExposure     = 3;
    int myGain         = 255;
    int myWhiteBalance = 4800;

    final double DESIRED_DISTANCE = 4.0;

    final double SPEED_GAIN  =  0.008;
    final double STRAFE_GAIN =  0.02;
    final double TURN_GAIN   =  0.01;

    final double MAX_AUTO_SPEED  = 0.3;
    final double MAX_AUTO_STRAFE = 0.3;
    final double MAX_AUTO_TURN   = 0.3;

    AprilTagCenteringState aprilTagCenteringState = AprilTagCenteringState.START;

    enum AprilTagCenteringState {
        START,
        CENTERING_YAW,
        CENTERING_HEADING,
        MOVING_TO_RANGE,
        COMPLETE
    }

    @Override
    public void runOpMode() {
        DriveBase.init(hardwareMap);

        initVideo(); // Initializes the Vision Processing

        waitForStart();

        while (opModeIsActive()) {
            centerOnAprilTag(DESIRED_TAG_ID);
        }
    }

    public void initVideo() {
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

        setManualExposure(myExposure, myGain, myWhiteBalance);
    }

    private void setManualExposure(int exposureMS, int gain, int whiteBalance) {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(whiteBalance);
    }

    public void centerOnAprilTag(int desiredTagId) {
        boolean targetFound   = false;
        boolean targetReached = false;

        double drive, strafe, turn;

        while (!targetFound) {
            telemetry.addLine("Searching For Target");

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata == null) continue; // Skip tags that we don't have information on

                if (detection.id == desiredTagId) {
                    telemetry.addData("April Tag found. Id", detection.id);
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            telemetry.update();
        }

        while (!targetReached) {
            double rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError     = desiredTag.ftcPose.yaw;

            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED) * -1.0;
            strafe = Range.clip(headingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            switch (aprilTagCenteringState) {
                case START:
                    DriveBase.driveManualRobotCentric(0,0,0);

                    aprilTagCenteringState = AprilTagCenteringState.CENTERING_YAW;
                    break;
                case CENTERING_YAW:
                    DriveBase.driveManualRobotCentric(0, strafe, 0);

                    if (yawError < 0.5) aprilTagCenteringState = AprilTagCenteringState.CENTERING_HEADING;

                    break;
                case CENTERING_HEADING:
                    DriveBase.driveManualRobotCentric(0,0, turn);

                    if (yawError > 0.6) {
                        aprilTagCenteringState = AprilTagCenteringState.CENTERING_HEADING;
                        break;
                    }

                    if (headingError < 1.0) aprilTagCenteringState = AprilTagCenteringState.MOVING_TO_RANGE;

                    break;
                case MOVING_TO_RANGE:
                    DriveBase.driveManualRobotCentric(drive, 0, 0);

                    if (yawError > 0.6) {
                        aprilTagCenteringState = AprilTagCenteringState.CENTERING_YAW;
                        break;
                    }

                    if (headingError > 1.0) {
                        aprilTagCenteringState = AprilTagCenteringState.CENTERING_HEADING;
                    }
                case COMPLETE:
                    break;
            }

            if (rangeError < 0.2 && headingError < 1 && yawError < 0.5) targetReached = true;
        }

        DriveBase.driveManualRobotCentric(0,0,0);
    }
}