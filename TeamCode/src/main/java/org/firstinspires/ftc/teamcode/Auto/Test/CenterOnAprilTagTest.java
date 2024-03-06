package org.firstinspires.ftc.teamcode.Auto.Test;

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

@Autonomous (name = "Test - Center On April Tag", group = "Linear OpMode")
public class CenterOnAprilTagTest extends LinearOpMode {
    AprilTagProcessor aprilTagProcessor;  // Used for managing the AprilTag detection process.
    AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    final int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.

    VisionPortal visionPortal;

    int myExposure     = 3;
    int myGain         = 255;
    int myWhiteBalance = 4800;

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 4.0; //  this is how close the camera should get to the target (inches)

    // CHANGED test values
    final double SPEED_GAIN  =  0.008;
    final double STRAFE_GAIN =  0.02;
    final double TURN_GAIN   =  0.01;

    final double MAX_AUTO_SPEED = 0.3;
    final double MAX_AUTO_STRAFE= 0.3;
    final double MAX_AUTO_TURN  = 0.3;

    @Override
    public void runOpMode() {
        DriveBase.init(hardwareMap);

        initVideo();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            aprilTagBackdrop();

            telemetry.update();
            sleep(30000);
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

        setManualExposure(myExposure, myGain, myWhiteBalance);

    }

    private void setManualExposure(int exposureMS, int gain, int whiteBalance) {
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Camera Waiting");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(whiteBalance);
    }

    public void aprilTagBackdrop() {
        boolean targetFound;
        boolean targetReached = false;
        double  drive;
        double  strafe;
        double  turn;

        desiredTag = null;

        while (opModeIsActive() && !targetReached) {
            targetFound = false;

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata == null) {
                    continue;
                }

                if (detection.id == DESIRED_TAG_ID) {
                    telemetry.addData("April Tag found. Id", detection.id);
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError     = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED) * -1.0;
                strafe = Range.clip(headingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                turn   = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Drive", drive);
                telemetry.addData("Strafe", strafe);
                telemetry.addData("Turn", turn);

                telemetry.addData("Range error", rangeError);
                telemetry.addData("Heading error", headingError);
                telemetry.addData("Yaw Error", yawError);

                // Apply desired axes motions to the drivetrain.
                if (rangeError <= 0.1) {
                    targetReached = true;
                } else {
                    DriveBase.driveManualRobotCentric(drive, strafe, turn);
                }
                telemetry.update();
            } else {
                DriveBase.driveManualRobotCentric(0,0,0);
            }
        }

        DriveBase.driveManualRobotCentric(0,0,0);
    }
}