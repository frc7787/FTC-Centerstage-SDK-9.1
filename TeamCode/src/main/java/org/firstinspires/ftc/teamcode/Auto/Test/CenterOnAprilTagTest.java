package org.firstinspires.ftc.teamcode.Auto.Test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name = "Test - Center On April Tag", group = "Test")
public class CenterOnAprilTagTest extends OpMode {
    AprilTagProcessor aprilTagProcessor;
    AprilTagDetection desiredTag = null;
    int DESIRED_TAG_ID = 5;

    VisionPortal visionPortal;

    int myExposure     = 3;
    int myGain         = 255;
    int myWhiteBalance = 4800;

    final double DESIRED_DISTANCE = 4.0;

    // Error Correction Values
    final double SPEED_GAIN  = 0.015;
    final double STRAFE_GAIN = 0.01;
    final double TURN_GAIN   = 0.001;

    // Speed clip
    final double MAX_AUTO_SPEED  = 0.25;
    final double MAX_AUTO_STRAFE = 0.3;
    final double MAX_AUTO_TURN   = 0.15;

    @Override public void init() {
        DriveBase.init(hardwareMap);
        initVideo();
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

        setManualExposure(myExposure, myGain, myWhiteBalance);
    }

    @Override public void loop() {
        aprilTagBackdrop();
    }

    @Override public void stop() {
        DriveBase.driveManualFieldCentric(0,0,0);
    }

    private void setManualExposure(int exposureMS, int gain, int white) {
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) { // Wait for camera to start streaming
            telemetry.addLine("Waiting for camera");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(white);
    }

    public void aprilTagBackdrop() {
        List<AprilTagDetection> currentDetections;

        boolean targetFound;
        boolean targetReached = false;

        double drive, strafe, turn;

        desiredTag  = null;

        while (!targetReached) {
            targetFound = false;

            while (true) {
                currentDetections = aprilTagProcessor.getDetections();

                if (!currentDetections.isEmpty()) {
                    break;
                }

                telemetry.addLine("No AprilTags Detected. Waiting.");
                telemetry.update();
            }

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata == null) continue;

                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    telemetry.addData("April Tag found", detection.id);
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if (targetFound) {
                // Determines the range, heading, and yaw error
                double rangeError   = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                double headingError = desiredTag.ftcPose.bearing;
                double yawError     = desiredTag.ftcPose.yaw;

                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = -1.0 * Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                turn   = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Drive", drive);
                telemetry.addData("Strafe", strafe);
                telemetry.addData("Turn", turn);

                telemetry.addData("Range", desiredTag.ftcPose.range);

                telemetry.addData("Range Error", rangeError);
                telemetry.addData("Heading Error", headingError);
                telemetry.addData("Yaw Error", yawError);

                if (rangeError <= 0.1 && headingError < 2) {
                    targetReached = true;
                    continue;
                }

                DriveBase.driveManualRobotCentric(drive, strafe, turn);

                telemetry.update();
            }
        }

        DriveBase.driveManualRobotCentric(0,0,0);
    }


}