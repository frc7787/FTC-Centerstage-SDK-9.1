package org.firstinspires.ftc.teamcode.Auto.Expiremental;

import static org.firstinspires.ftc.teamcode.Properties.*;

import static org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState.AT_POS;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Auto.Core.PlacingState;
import org.firstinspires.ftc.teamcode.Auto.Core.PropColor;
import org.firstinspires.ftc.teamcode.Auto.Core.PropLocation;
import org.firstinspires.ftc.teamcode.Auto.PropDetector;
import org.firstinspires.ftc.teamcode.Auto.Utility.PIDController;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Auxiliaries;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.Auto.Core.PlacingState.*;

@Autonomous(name = "Blue - Audience - Clean", group = "Blue")
@Config
@Disabled
public class AutoBlueAudienceClean extends LinearOpMode {
    private final int MAX_APRIL_TAG_DETECTIONS = 25;

    private final long INITIAL_SLEEP                      = 0;
    private final long AFTER_PURPLE_PIXEL_PLACEMENT_SLEEP = 0;
    private final long BEFORE_PARK_SLEEP                  = 0;

    private PropDetector propDetector;
    private PropLocation location;
    private OpenCvCamera camera;

    private MecanumDriveBase mecanumDriveBase;

    private AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection desiredTag;
    private VisionPortal visionPortal;

    private TrajectorySequence toSpikeLeft,
                               toSpikeCenter,
                               toSpikeRight,
                               toBackdropLeft,
                               toBackdropCenter,
                               toBackdropRight,
                               toPark;

    private PlacingState placingState = START;

    private double drive, strafe, turn;
    private double rangeErr, yawErr, bearingErr;
    private double prevRangeErr, prevYawErr, prevBearingErr;

    private final PIDController TURN_PID   = new PIDController(TURN_GAIN, 0.0, TURN_D);
    private final PIDController STRAFE_PID = new PIDController(STRAFE_GAIN, 0.0, STRAFE_D);
    private final PIDController DRIVE_PID  = new PIDController(DRIVE_GAIN, 0.0, DRIVE_D);

    @Override public void runOpMode() throws InterruptedException {
        initHardware();
        initAprilTagVisionProcessing();
        initPropDetection();
        initRoadrunner();

        Arm.setWormTargetPos(AUTO_INITIAL_WORM_POSITION);
        Arm.update(false);

        waitForStart();

        if (isStopRequested()) return;

        Arm.setWormTargetPos(0);

        detectProp();

        sleep(INITIAL_SLEEP);

        switch (location) {
            case LEFT:
                mecanumDriveBase.followTrajectorySequence(toSpikeLeft);
                Auxiliaries.retractPixelPlacerServo();

                sleep(AFTER_PURPLE_PIXEL_PLACEMENT_SLEEP);

                mecanumDriveBase.followTrajectorySequence(toBackdropLeft);

                centerOnAprilTag(1);

                placePixelOnBackdrop(
                        YELLOW_PIXEL_WORM_POSITION_AUDIENCE,
                        YELLOW_PIXEL_ELEVATOR_POSITION_AUDIENCE,
                        1000
                );

                toPark = mecanumDriveBase.trajectorySequenceBuilder(toBackdropLeft.end())
                        .strafeTo(new Vector2d(45, 64))
                        .lineTo(new Vector2d(60, 64))
                        .build();
                break;
            case CENTER:
            case NONE:
                mecanumDriveBase.followTrajectorySequence(toSpikeCenter);
                Auxiliaries.retractPixelPlacerServo();

                sleep(AFTER_PURPLE_PIXEL_PLACEMENT_SLEEP);

                mecanumDriveBase.followTrajectorySequence(toBackdropCenter);

                centerOnAprilTag(2);

                placePixelOnBackdrop(
                        YELLOW_PIXEL_WORM_POSITION_AUDIENCE,
                        YELLOW_PIXEL_ELEVATOR_POSITION_AUDIENCE,
                        1000
                );

                toPark = mecanumDriveBase.trajectorySequenceBuilder(toBackdropCenter.end())
                        .strafeTo(new Vector2d(45, 64))
                        .lineTo(new Vector2d(60, 64))
                        .build();
                break;
            case RIGHT:
                mecanumDriveBase.followTrajectorySequence(toSpikeRight);
                Auxiliaries.retractPixelPlacerServo();

                sleep(AFTER_PURPLE_PIXEL_PLACEMENT_SLEEP);

                mecanumDriveBase.followTrajectorySequence(toBackdropRight);

                centerOnAprilTag(3);

                placePixelOnBackdrop(
                        YELLOW_PIXEL_WORM_POSITION_AUDIENCE,
                        YELLOW_PIXEL_ELEVATOR_POSITION_AUDIENCE,
                        1000
                );

                toPark = mecanumDriveBase.trajectorySequenceBuilder(toBackdropRight.end())
                        .strafeTo(new Vector2d(45, 57))
                        .lineTo(new Vector2d(60, 57))
                        .build();
                break;
        }

        sleep(BEFORE_PARK_SLEEP);

        mecanumDriveBase.followTrajectorySequence(toPark);

        sleep(20000);
    }

    private void initHardware() {
        Auxiliaries.init(hardwareMap);
        Arm.init(hardwareMap);

        mecanumDriveBase = new MecanumDriveBase(hardwareMap);

        mecanumDriveBase.init();
    }

    private void initPropDetection() {
        propDetector = new PropDetector(PropColor.BLUE);

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
    }

    /**
     * Initializes the April Tag Processing Pipeline
     */
    private void initAprilTagVisionProcessing() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681)
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

    private void initRoadrunner() {
        Pose2d startPos = new Pose2d(-35, 63, Math.toRadians(90));

        mecanumDriveBase.setPoseEstimate(startPos);

        toSpikeLeft = mecanumDriveBase.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(-36, 36))
                .lineToLinearHeading(new Pose2d(-21, 32, Math.toRadians(0)))
                .build();

        toSpikeCenter = mecanumDriveBase.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-35, 20, Math.toRadians(180)))
                .build();

        toSpikeRight = mecanumDriveBase.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-49, 20, Math.toRadians(180)))
                .build();

        toBackdropLeft = mecanumDriveBase.trajectorySequenceBuilder(toSpikeLeft.end())
                .strafeTo(new Vector2d(-20, 38))
                .lineToConstantHeading(new Vector2d(-36, 42))
                .strafeTo(new Vector2d(-36, 12))
                .lineToConstantHeading(new Vector2d(41.5, 13))
                .strafeTo(new Vector2d(41, 45))
                .build();

        toBackdropCenter = mecanumDriveBase.trajectorySequenceBuilder(toSpikeCenter.end())
                .strafeTo(new Vector2d(-35, 12))
                .lineToConstantHeading(new Vector2d(43.5, 9))
                .strafeTo(new Vector2d(41.5, 29))
                .turn(Math.toRadians(180))
                .build();

        toBackdropRight = mecanumDriveBase.trajectorySequenceBuilder(toSpikeRight.end())
                .strafeTo(new Vector2d(-49, 11.5))
                .lineToConstantHeading(new Vector2d(43, 11.5))
                .strafeTo(new Vector2d(40.5, 22))
                .turn(Math.toRadians(180))
                .build();
    }

    private void detectProp() {
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
    }

    private void placePixelOnBackdrop(int wormPos, int elevatorPos, int outtakeSleep) {

        while (true) {
            Arm.update(false);

            if (isStopRequested() || !opModeIsActive()) return;

            switch (placingState) {
                case START:
                    Arm.setTargetPos(wormPos, elevatorPos);

                    placingState = ROTATING_TO_PLACE_YELLOW_PIXEL;
                    break;

                case PLACING_YELLOW_PIXEL:
                    Arm.openDeliveryTrayDoor(TRAY_DOOR_OPEN_POS, TRAY_DOOR_OPEN_POS);

                    sleep(outtakeSleep);

                    Arm.openDeliveryTrayDoor(0.0, 0.0);

                    placingState = CLEARING_YELLOW_PIXEL;
                    break;
                case CLEARING_YELLOW_PIXEL:
                    Arm.setWormTargetPos(wormPos + 200);

                    if (Arm.state() == AT_POS) placingState = RETRACTING_ARM;
                    break;
                case RETRACTING_ARM:
                    Arm.setTargetPos(0, 0);

                    if (Arm.state() == AT_POS) placingState = PLACED;
                    break;
                case PLACED:
                    break;
            }

            Arm.update(false);

            if (placingState == PlacingState.PLACED) {
                return;
            }
        }
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
    private boolean detectAprilTags(int desiredTagId) {
        int count = 0;

        boolean targetDetected = false;

        while (!targetDetected) { // Wait until we detect the desired April Tag
            if (isStopRequested() || !opModeIsActive()) return false;

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            if (count > MAX_APRIL_TAG_DETECTIONS) break;

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

        while (!isAtTarget) { // Wait for the robot to center on the April Tag
            if (isStopRequested() || !opModeIsActive()) return;

            // Quit loop if it takes more than 5 seconds to center
            if (System.currentTimeMillis() - start > 5000) {
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

                drive  = Range.clip(DRIVE_PID.calculate(desiredTag.ftcPose.range, DESIRED_DISTANCE_FROM_APRIL_TAG_IN), -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
                strafe = Range.clip(STRAFE_PID.calculate(-desiredTag.ftcPose.yaw, 0.0), -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
                turn   = Range.clip(TURN_PID.calculate(bearingErr, 0.0), -MAX_TURN_SPEED, MAX_TURN_SPEED);

                drive *= -1.0;

                if (isWithinTolerance()) {
                    isAtTarget = true;
                    MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);
                } else {
                    MecanumDriveBase.driveManualFF(drive, strafe, turn, 0.03);
                }
            } else {
                MecanumDriveBase.driveManualFF(0.0, 0.0, 0.0, 0.0);

                if (isWithinTolerance()) break;
            }
        }

        mecanumDriveBase.updatePoseEstimate();
    }

    private boolean isWithinTolerance() {
        return (Math.abs(rangeErr)      <= RANGE_ERROR_TOLERANCE
                && Math.abs(yawErr)     <= YAW_ERROR_TOLERANCE
                && Math.abs(bearingErr) <= BEARING_ERROR_TOLERANCE);
    }
}
