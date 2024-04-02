package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Auto.Core.PropColor;
import org.firstinspires.ftc.teamcode.Auto.Core.PropLocation;
import org.firstinspires.ftc.teamcode.Auto.Utility.PIDController;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Auxiliaries;
import org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red - Backdrop", group = "Red")
public class AutoRedBackdrop extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;
    OpenCvCamera camera;

    AprilTagProcessor aprilTagProcessor;
    AprilTagDetection desiredTag;
    VisionPortal visionPortal;

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
        CLEARING_PIXELS,
        RETRACTING,
        PLACED
    }

    int wormTargetPos     = 890;
    int elevatorTargetPos = 2430;

    PlacingState placingState = PlacingState.START;

    // "P" Value for drive, strafe, and turn
    final double DRIVE_GAIN  = 0.025;
    final double STRAFE_GAIN = 0.07;
    final double TURN_GAIN   = 0.05;

    // "D" Value for drive, strafe, turn
    final double DRIVE_D  = 0.0025;
    final double STRAFE_D = 0.00002;
    final double TURN_D   = 0.0012;

    double drive, strafe, turn;
    double rangeErr, yawErr, bearingErr;
    double prevRangeErr, prevYawErr, prevBearingErr;

    MecanumDriveBase mecanumDriveBase;

    PIDController turnPID   = new PIDController(TURN_GAIN, 0.0, TURN_D);
    PIDController strafePID = new PIDController(STRAFE_GAIN, 0.0, STRAFE_D);
    PIDController drivePID  = new PIDController(DRIVE_GAIN, 0.0, DRIVE_D);

    @Override
    public void runOpMode() throws InterruptedException {
        propDetector     = new PropDetector(PropColor.RED);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);

        mecanumDriveBase.init();

        Arm.init(hardwareMap);

        Pose2d startPose = new Pose2d(11, -63, Math.toRadians(270));

        mecanumDriveBase.setPoseEstimate(startPose);

        initAprilTagVisionProcessing();

        TrajectorySequence toSpikeLeft = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12, -36))
                .lineToLinearHeading(new Pose2d(-4, -32, Math.toRadians(-180)))
                .build();

        TrajectorySequence toSpikeCenter = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(11, -20, Math.toRadians(0)))
                .build();

        TrajectorySequence toSpikeRight = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(25, -20, Math.toRadians(0)))
                .build();

        TrajectorySequence toBackdropLeft = mecanumDriveBase.trajectorySequenceBuilder(toSpikeLeft.end())
                .strafeTo(new Vector2d(-4, -38))
                .lineToConstantHeading(new Vector2d(38, -36))
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence toBackdropCenter = mecanumDriveBase.trajectorySequenceBuilder(toSpikeCenter.end())
                .strafeTo(new Vector2d(11, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -36))
                .build();

        TrajectorySequence toBackdropRight = mecanumDriveBase.trajectorySequenceBuilder(toSpikeRight.end())
                .strafeTo(new Vector2d(25, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -45))
                .build();

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(propDetector);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Failed to open camera due to error code", errorCode);
                telemetry.update();
            }
        });

        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        Arm.update(false);

        Arm.rotateWorm(1400);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        location = propDetector.getPropLocation();


        int leftCount = 0;
        int rightCount = 0;
        int centerCount = 0;
        int noneCount = 0;

        for (int i = 0; i <= 20; i++) {
            switch (propDetector.getPropLocation()) {
                case LEFT:
                    leftCount += 1;
                    break;
                case RIGHT:
                    rightCount += 1;
                    break;
                case CENTER:
                    centerCount += 1;
                    break;
                case NONE:
                    noneCount += 1;
                    break;
            }
        }

        if (leftCount >= rightCount && leftCount >= noneCount && leftCount >= centerCount) {
            location = PropLocation.LEFT;
        } else if (rightCount >= leftCount && rightCount >= noneCount && rightCount >= centerCount) {
            location = PropLocation.RIGHT;
        } else if (centerCount >= noneCount) {
            location = PropLocation.CENTER;
        } else {
            location = PropLocation.NONE;
        }

        telemetry.addData("PROP LOCATION: ", location);
        telemetry.update();

        Arm.rotateWorm(0);

        sleep(0);

        switch (location) {
            case LEFT:
                mecanumDriveBase.followTrajectorySequence(toSpikeLeft);
                Auxiliaries.placePixelOnSpikeStripRight();
                mecanumDriveBase.followTrajectorySequence(toBackdropLeft);

                centerOnAprilTag(4);

                placePixelOnBackdrop();
                break;
            case RIGHT:
                mecanumDriveBase.followTrajectorySequence(toSpikeRight);
                Auxiliaries.placePixelOnSpikeStripRight();
                mecanumDriveBase.followTrajectorySequence(toBackdropRight);

                centerOnAprilTag(6);

                placePixelOnBackdrop();
                break;
            case CENTER:
            case NONE:
                mecanumDriveBase.followTrajectorySequence(toSpikeCenter);
                Auxiliaries.placePixelOnSpikeStripRight();
                mecanumDriveBase.followTrajectorySequence(toBackdropCenter);

                centerOnAprilTag(5);

                placePixelOnBackdrop();
                break;
        }

        TrajectorySequence toPark = mecanumDriveBase.trajectorySequenceBuilder(mecanumDriveBase.getPoseEstimate())
                .strafeTo(new Vector2d(45, -64))
                .lineTo(new Vector2d(60, -64))
                .build();

        mecanumDriveBase.followTrajectorySequence(toPark);


        sleep(20000);
    }

    /**
     * Initializes the April Tag Processing Pipeline
     */
    public void initAprilTagVisionProcessing() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .enableLiveView(false)
                .setCameraResolution(resolution)
                .setAutoStopLiveView(true)
                .build();

        while (visionPortal.getCameraState() != STREAMING) {
            if (isStopRequested() || !opModeIsActive()) return;
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
    private boolean detectAprilTags(int desiredTagId) {
        int count = 0;

        boolean targetDetected = false;

        while (!targetDetected) { // Wait until we detect the desired April Tag
            if (isStopRequested() || !opModeIsActive()) return false;

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            if (count > maxAprilTagDetections) {
                break;
            }

            for (AprilTagDetection detection : currentDetections) {
                if (isStopRequested() || !opModeIsActive()) return false;

                if (detection.metadata == null) continue;

                if (detection.id == desiredTagId) { // If the tag is the one we want, stop looking

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
    public void centerOnAprilTag(int desiredTagId) {
        boolean isAtTarget = false;

        mecanumDriveBase.updatePoseEstimate();

        while (!isAtTarget) { // Wait for the robot to center on the April Tag
            if (isStopRequested() || !opModeIsActive()) return;

            if (detectAprilTags(desiredTagId)) {
                long start = System.currentTimeMillis();

                // Quit loop if it takes more than 5 seconds to center
                if (System.currentTimeMillis() - start > 5000) {
                    MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
                    break;
                }

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

                if (isWithinTolerance()) {
                    isAtTarget = true;
                    MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
                } else {
                    MecanumDriveBase.driveManualFF(drive, strafe, turn, 0.03);
                }
            } else {
                MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
            }
        }

        mecanumDriveBase.updatePoseEstimate();
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

            switch (placingState) {
                case START:
                    Arm.setElevatorPower(0.75);

                    Arm.setTargetPos(elevatorTargetPos, wormTargetPos);

                    placingState = PlacingState.MOVING_TO_POS;
                    break;
                case MOVING_TO_POS:
                    if (Arm.armState() == NormalPeriodArmState.AT_POS) {
                        placingState = PlacingState.PLACING;
                    }
                    break;
                case PLACING:
                    Arm.openDeliveryTrayDoor(0.3);

                    sleep(700);

                    Arm.openDeliveryTrayDoor(0.0);

                    placingState = PlacingState.CLEARING_PIXELS;
                    break;
                case CLEARING_PIXELS:
                    Arm.rotateWorm(1200);

                    if (Arm.armState() == NormalPeriodArmState.AT_POS) {
                        placingState = PlacingState.RETRACTING;
                    }
                case RETRACTING:
                    Arm.setElevatorPower(1.0);

                    Arm.setTargetPos(0, 0);
                    Arm.update(false);

                    if (Arm.wormPos() < 5 && Arm.elevatorPos() < 5) {
                        placingState = PlacingState.PLACED;
                    }
                    break;
                case PLACED:
                    break;
            }
        }
    }
}
