package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Properties.BEARING_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Properties.CAMERA_RESOLUTION;
import static org.firstinspires.ftc.teamcode.Properties.DESIRED_DISTANCE_FROM_APRIL_TAG_IN;
import static org.firstinspires.ftc.teamcode.Properties.DRIVE_D;
import static org.firstinspires.ftc.teamcode.Properties.DRIVE_GAIN;
import static org.firstinspires.ftc.teamcode.Properties.ELEVATOR_EXTENSION_SPEED_AUTO;
import static org.firstinspires.ftc.teamcode.Properties.ELEVATOR_RETRACTION_SPEED_AUTO;
import static org.firstinspires.ftc.teamcode.Properties.EXPOSURE_MS;
import static org.firstinspires.ftc.teamcode.Properties.GAIN;
import static org.firstinspires.ftc.teamcode.Properties.AUTO_INITIAL_WORM_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.MAX_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.Properties.MAX_STRAFE_SPEED;
import static org.firstinspires.ftc.teamcode.Properties.MAX_TURN_SPEED;
import static org.firstinspires.ftc.teamcode.Properties.RANGE_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Properties.STRAFE_D;
import static org.firstinspires.ftc.teamcode.Properties.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.Properties.TURN_D;
import static org.firstinspires.ftc.teamcode.Properties.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.Properties.WHITE_BALANCE;
import static org.firstinspires.ftc.teamcode.Properties.YAW_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Properties.YELLOW_PIXEL_CLEARING_WORM_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.YELLOW_PIXEL_ELEVATOR_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.YELLOW_PIXEL_WORM_POSITION;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
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

@Autonomous(name = "Blue - Backdrop", group = "Blue")
@Config
public class AutoBlueBackdrop extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;
    OpenCvCamera camera;

    MecanumDriveBase mecanumDriveBase;

    AprilTagProcessor aprilTagProcessor;
    AprilTagDetection desiredTag;
    VisionPortal visionPortal;

    TrajectorySequence toSpikeLeft,
                       toSpikeCenter,
                       toSpikeRight,
                       toBackdropLeft,
                       toBackdropCenter,
                       toBackdropRight,
                       toPark;

    int maxAprilTagDetections = 25;

    enum PlacingState {
        START,
        MOVING_TO_POS,
        PLACING,
        CLEARING_PIXELS,
        RETRACTING,
        PLACED
    }

    PlacingState placingState = PlacingState.START;

    double drive, strafe, turn;
    double rangeErr, yawErr, bearingErr;
    double prevRangeErr, prevYawErr, prevBearingErr;

    PIDController turnPID   = new PIDController(TURN_GAIN, 0.0, TURN_D);
    PIDController strafePID = new PIDController(STRAFE_GAIN, 0.0, STRAFE_D);
    PIDController drivePID  = new PIDController(DRIVE_GAIN, 0.0, DRIVE_D);

    @Override public void runOpMode() throws InterruptedException {
        propDetector     = new PropDetector(PropColor.BLUE);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);

        mecanumDriveBase.init();

        Pose2d startPose = new Pose2d(11, 63, Math.toRadians(90));

        mecanumDriveBase.setPoseEstimate(startPose);

        toSpikeLeft = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(23, 34, Math.toRadians(0)))
                .build();

        toSpikeCenter = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(0)))
                .build();

        toSpikeRight = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, 39, Math.toRadians(22)))
                .lineToConstantHeading(new Vector2d(-2, 36))
                .build();

        toBackdropLeft = mecanumDriveBase.trajectorySequenceBuilder(toSpikeLeft.end())
                .strafeTo(new Vector2d(23, 38))
                .lineToLinearHeading(new Pose2d(38, 42, Math.toRadians(0)))
                .build();

        toBackdropCenter = mecanumDriveBase.trajectorySequenceBuilder(toSpikeCenter.end())
                .strafeTo(new Vector2d(10, 38))
                .lineToConstantHeading(new Vector2d(38, 36))
                .build();

        toBackdropRight = mecanumDriveBase.trajectorySequenceBuilder(toSpikeRight.end())
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(38, 30))
                .build();

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(propDetector);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Failed to open camera due to error code", errorCode);
                telemetry.update();
            }
        });

        initAprilTagVisionProcessing();

        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        Arm.update(false);

        Arm.rotateWorm(AUTO_INITIAL_WORM_POSITION);

        waitForStart();

        if (isStopRequested()) return;

        location = propDetector.getPropLocation();

        int leftCount   = 0;
        int rightCount  = 0;
        int noneCount   = 0;
        int centerCount = 0;

        for (int i = 0; i <= 20;  i++) {
            switch (propDetector.getPropLocation()) {
                case LEFT:
                    leftCount += 1;
                    break;
                case RIGHT:
                    rightCount += 1;
                    break;
                case CENTER:
                    centerCount += 1;
                case NONE:
                    noneCount += 1;
                    break;
            }
        }

        if (leftCount >= rightCount && leftCount >= noneCount && leftCount >= centerCount) {
            location = PropLocation.LEFT;
        } else if (rightCount >= leftCount && rightCount >= noneCount && rightCount >= centerCount) {
            location = PropLocation.RIGHT;
        } else if (centerCount >= noneCount){
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

                centerOnAprilTag(1);

                placePixelOnBackdrop();
                break;
            case CENTER:
            case NONE:
                mecanumDriveBase.followTrajectorySequence(toSpikeCenter);
                Auxiliaries.placePixelOnSpikeStripRight();
                mecanumDriveBase.followTrajectorySequence(toBackdropCenter);

                centerOnAprilTag(2);

                placePixelOnBackdrop();
                break;
            case RIGHT:
                mecanumDriveBase.followTrajectorySequence(toSpikeRight);
                Auxiliaries.placePixelOnSpikeStripRight();
                mecanumDriveBase.followTrajectorySequence(toBackdropRight);

                centerOnAprilTag(3);

                placePixelOnBackdrop();
                break;
        }

        toPark = mecanumDriveBase.trajectorySequenceBuilder(mecanumDriveBase.getPoseEstimate())
                        .strafeTo(new Vector2d(45, 64))
                        .lineTo(new Vector2d(60, 64))
                        .build();

        mecanumDriveBase.followTrajectorySequence(toPark);

        sleep(2000);
    }

    /**
     * Initializes the April Tag Processing Pipeline
     */
    void initAprilTagVisionProcessing() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .enableLiveView(false)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(true)
                .build();

        while (visionPortal.getCameraState() != STREAMING) {
            if (isStopRequested() || !opModeIsActive()) return;
        }

        setManualCameraSettings(EXPOSURE_MS, GAIN, WHITE_BALANCE);
    }

    /**
     * Sets the exposure, gain, and whiteBalance manually
     * @param exposureMS The exposure in milliseconds
     * @param gain The gain to set
     * @param whiteBalance The color temperature to set
     */
    void setManualCameraSettings(int exposureMS, int gain, int whiteBalance) {
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
    boolean detectAprilTags(int desiredTagId) {
        int count = 0;

        boolean targetDetected = false;

        while (!targetDetected) { // Wait until we detect the desired April Tag
            if (isStopRequested() || !opModeIsActive()) return false;

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            if (count > maxAprilTagDetections) break;

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

    @SuppressLint("DefaultLocale")
    void centerOnAprilTag(int desiredTagId) {
        boolean isAtTarget = false;

        long start = System.currentTimeMillis();

        mecanumDriveBase.updatePoseEstimate();

        if (isWithinTolerance()) return;

        while (!isAtTarget) { // Wait for the robot to center on the April Tag
            if (isStopRequested() || !opModeIsActive()) return;

            // Quit loop if it takes more than 4 seconds to center
            if (System.currentTimeMillis() - start > 4000) {
                MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
                break;
            }

            if (detectAprilTags(desiredTagId)) {
                prevRangeErr = rangeErr;
                rangeErr     = (desiredTag.ftcPose.range - DESIRED_DISTANCE_FROM_APRIL_TAG_IN);

                prevYawErr = yawErr;
                yawErr     = desiredTag.ftcPose.yaw;

                prevBearingErr = bearingErr;
                bearingErr     = desiredTag.ftcPose.bearing;

                rangeErr   = 0.9 * rangeErr   + 0.1 * prevRangeErr;
                yawErr     = 0.9 * yawErr     + 0.1 * prevYawErr;
                bearingErr = 0.9 * bearingErr + 0.1 * prevBearingErr;

                if (Math.abs(rangeErr) < RANGE_ERROR_TOLERANCE)     rangeErr   = 0.0;
                if (Math.abs(yawErr) < YAW_ERROR_TOLERANCE)         yawErr     = 0.0;
                if (Math.abs(bearingErr) < BEARING_ERROR_TOLERANCE) bearingErr = 0.0;

                drive  = Range.clip(drivePID.calculate(desiredTag.ftcPose.range, DESIRED_DISTANCE_FROM_APRIL_TAG_IN), -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
                strafe = Range.clip(strafePID.calculate(-desiredTag.ftcPose.yaw, 0.0), -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
                turn   = Range.clip(turnPID.calculate(bearingErr, 0.0), -MAX_TURN_SPEED, MAX_TURN_SPEED);

                drive *= -1.0;

                if (isWithinTolerance()) {
                    isAtTarget = true;
                    MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
                } else {
                    MecanumDriveBase.driveManualFF(drive, strafe, turn, 0.03);
                }
            } else {
                MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);

                if (isWithinTolerance()) isAtTarget = true;
            }
        }

        mecanumDriveBase.updatePoseEstimate();
    }

    boolean isWithinTolerance() {
        return (Math.abs(rangeErr)      < RANGE_ERROR_TOLERANCE
                && Math.abs(yawErr)     < YAW_ERROR_TOLERANCE
                && Math.abs(bearingErr) < BEARING_ERROR_TOLERANCE);
    }

    void placePixelOnBackdrop() {
        while (placingState != PlacingState.PLACED) {
            if (isStopRequested() || !opModeIsActive()) return;

            Arm.update(false);

            switch (placingState) {
                case START:
                    Arm.setElevatorPower(ELEVATOR_EXTENSION_SPEED_AUTO);

                    Arm.setTargetPos(YELLOW_PIXEL_ELEVATOR_POSITION, YELLOW_PIXEL_WORM_POSITION);

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
                    Arm.setTargetPos(Arm.elevatorPos(), YELLOW_PIXEL_CLEARING_WORM_POSITION);

                    Arm.update(false);

                    if (Arm.armState() == NormalPeriodArmState.AT_POS) {
                        placingState = PlacingState.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    Arm.setElevatorPower(ELEVATOR_RETRACTION_SPEED_AUTO);

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