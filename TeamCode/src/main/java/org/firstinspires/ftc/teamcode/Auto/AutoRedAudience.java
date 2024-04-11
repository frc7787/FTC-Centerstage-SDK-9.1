package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Properties.BEARING_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Properties.CAMERA_RESOLUTION;
import static org.firstinspires.ftc.teamcode.Properties.DESIRED_DISTANCE_FROM_APRIL_TAG_IN;
import static org.firstinspires.ftc.teamcode.Properties.DRIVE_D;
import static org.firstinspires.ftc.teamcode.Properties.DRIVE_GAIN;
import static org.firstinspires.ftc.teamcode.Properties.ELEVATOR_EXTENSION_SPEED_AUTO;
import static org.firstinspires.ftc.teamcode.Properties.ELEVATOR_RETRACTION_SPEED_AUTO;
import static org.firstinspires.ftc.teamcode.Properties.EXPOSURE_MS;
import static org.firstinspires.ftc.teamcode.Properties.GAIN;
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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

@Autonomous(name = "Red - Audience", group = "Red")
public class AutoRedAudience extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;
    OpenCvCamera camera;

    DcMotorImplEx wormMotor, elevatorMotor;

    ServoImplEx leftDoor, rightDoor;

    MecanumDriveBase mecanumDriveBase;

    AprilTagProcessor aprilTagProcessor;
    AprilTagDetection desiredTag;
    VisionPortal visionPortal;

    int maxAprilTagDetections = 25;

    enum PlacingState {
        START,
        ROTATING_TO_PLACE_YELLOW_PIXEL,
        EXTENDING_TO_PLACE_YELLOW_PIXEL,
        PLACING_YELLOW_PIXEL,
        RETRACTING_ELEVATOR,
        RETRACTING_WORM,
        PLACED
    }

    PlacingState placingState = PlacingState.START;

    TrajectorySequence toSpikeLeft,
                       toSpikeCenter,
                       toSpikeRight,
                       toBackdropLeft,
                       toBackdropCenter,
                       toBackdropRight,
                       toPark;

    double drive, strafe, turn;
    double rangeErr, yawErr, bearingErr;
    double prevRangeErr, prevYawErr, prevBearingErr;

    PIDController turnPID   = new PIDController(TURN_GAIN, 0.0, TURN_D);
    PIDController strafePID = new PIDController(STRAFE_GAIN, 0.0, STRAFE_D);
    PIDController drivePID  = new PIDController(DRIVE_GAIN, 0.0, DRIVE_D);

    @Override
    public void runOpMode() throws InterruptedException {
        wormMotor     = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");

        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
        wormMotor.setMode(STOP_AND_RESET_ENCODER);

        elevatorMotor.setMode(RUN_USING_ENCODER);
        wormMotor.setMode(RUN_USING_ENCODER);

        // Expansion Hub Port 1
        leftDoor = hardwareMap.get(ServoImplEx.class, "LeftDoorServo");
        // Expansion Hub Port 2
        rightDoor = hardwareMap.get(ServoImplEx.class, "RightDoorServo");

        propDetector     = new PropDetector(PropColor.RED);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);

        mecanumDriveBase.init();

        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(270.0));

        mecanumDriveBase.setPoseEstimate(startPose);

        toSpikeRight = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, -39, Math.toRadians(-158)))
                .lineToConstantHeading(new Vector2d(-22, -36))
                .build();

        toSpikeCenter = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, -20, Math.toRadians(0)))
                .build();

        toSpikeLeft = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42, -21, Math.toRadians(0)))
                .build();

        toBackdropLeft = mecanumDriveBase.trajectorySequenceBuilder(toSpikeLeft.end())
                .strafeTo(new Vector2d(-42, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -30))
                .build();

        toBackdropCenter = mecanumDriveBase.trajectorySequenceBuilder(toSpikeCenter.end())
                .strafeTo(new Vector2d(-36, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -36))
                .build();

        toBackdropRight = mecanumDriveBase.trajectorySequenceBuilder(toSpikeRight.end())
                .lineToLinearHeading(new Pose2d(-24, -38, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-40, -36))
                .strafeTo(new Vector2d(-40, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -42))
                .turn(Math.toRadians(180))
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

        initAprilTagVisionProcessing();

        Auxiliaries.init(hardwareMap);

        rotateWorm(1400, 1.0);

        waitForStart();

        if (isStopRequested()) { return; }

        location = propDetector.getPropLocation();

        int leftCount   = 0;
        int rightCount  = 0;
        int centerCount = 0;
        int noneCount   = 0;

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
                    break;
                case NONE:
                    noneCount += 1;
                    break;
            }
        }

        if (leftCount >= rightCount && leftCount >= centerCount && leftCount >= noneCount) {
            location = PropLocation.LEFT;
        } else if (rightCount >= leftCount && rightCount >= noneCount && rightCount >= centerCount) {
            location = PropLocation.RIGHT;
        } else if (centerCount >= noneCount) {
            location = PropLocation.CENTER;
        } else {
            location = PropLocation.NONE;
        }

        rotateWorm(0, 1.0);

        sleep(0);

        telemetry.addData("PROP LOCATION: ", location);
        telemetry.update();

        switch (location) {
            case LEFT:
                mecanumDriveBase.followTrajectorySequence(toSpikeLeft);
                Auxiliaries.retractPixelPlacerServo();
                mecanumDriveBase.followTrajectorySequence(toBackdropLeft);

                centerOnAprilTag(4);

                placePixelOnBackdrop();
                break;
            case CENTER:
            case NONE: // This case should copy center
                mecanumDriveBase.followTrajectorySequence(toSpikeCenter);
                Auxiliaries.retractPixelPlacerServo();
                mecanumDriveBase.followTrajectorySequence(toBackdropCenter);

                centerOnAprilTag(5);

                placePixelOnBackdrop();
                break;
            case RIGHT:
                mecanumDriveBase.followTrajectorySequence(toSpikeRight);
                Auxiliaries.retractPixelPlacerServo();
                mecanumDriveBase.followTrajectorySequence(toBackdropRight);

                centerOnAprilTag(6);

                placePixelOnBackdrop();
                break;
        }

        toPark = mecanumDriveBase.trajectorySequenceBuilder(mecanumDriveBase.getPoseEstimate())
                        .strafeTo(new Vector2d(45, -64))
                        .lineTo(new Vector2d(60, -64))
                        .build();

        mecanumDriveBase.followTrajectorySequence(toPark);

        sleep(20000);
    }

    /**
     * Extends the elevator to the provided position at the provided power
     * @param targetPos The position to move the elevator to
     * @param power The power to move to the target position at
     */
    private void extendElevator(int targetPos, double power) {
        telemetry.addLine("Got to Extend elevator");
        telemetry.update();
        elevatorMotor.setTargetPosition(targetPos);
        elevatorMotor.setMode(RUN_TO_POSITION);
        elevatorMotor.setPower(power);
    }

    /**
     * Rotates the worm to the provided position at the provided power
     * @param pos The position to move the worm to
     * @param power The power to move to the target position at
     */
    public void rotateWorm(int pos, double power) {
        wormMotor.setTargetPosition(pos);
        wormMotor.setMode(RUN_TO_POSITION);
        wormMotor.setPower(power);
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
     * Opens the delivery tray door to the positions provided in the argument
     * @param leftPos The position to move the left door to
     * @param rightPos The position to move the right door to
     */
    public void openDeliveryTrayDoor(double leftPos, double rightPos) {
        leftDoor.setPosition(leftPos);
        rightDoor.setPosition(rightPos);
    }

    /**
     * Tries to center the robot on the april tag with the id specified by DESIRED_TAG_ID
     */
    @SuppressLint("DefaultLocale")
    public void centerOnAprilTag(int desiredTagId) {
        boolean isAtTarget = false;

        long start = System.currentTimeMillis();

        mecanumDriveBase.updatePoseEstimate();

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

    public boolean isWithinTolerance() {
        return (Math.abs(rangeErr)      < RANGE_ERROR_TOLERANCE
                && Math.abs(yawErr)     < YAW_ERROR_TOLERANCE
                && Math.abs(bearingErr) < BEARING_ERROR_TOLERANCE);
    }

    private void placePixelOnBackdrop() {

        while (true) {
            if (isStopRequested() || !opModeIsActive()) return;

            switch (placingState) {
                case START:
                    rotateWorm(YELLOW_PIXEL_WORM_POSITION, 1.0);

                    placingState = PlacingState.ROTATING_TO_PLACE_YELLOW_PIXEL;
                    break;
                case ROTATING_TO_PLACE_YELLOW_PIXEL:
                    if (wormMotor.getCurrentPosition() >= 750) {
                        placingState = PlacingState.EXTENDING_TO_PLACE_YELLOW_PIXEL;
                    }

                    break;
                case EXTENDING_TO_PLACE_YELLOW_PIXEL:
                    extendElevator(YELLOW_PIXEL_ELEVATOR_POSITION, 1.0);

                    if (elevatorMotor.getCurrentPosition() >= YELLOW_PIXEL_ELEVATOR_POSITION - 5) {
                        placingState = PlacingState.PLACING_YELLOW_PIXEL;
                    }

                    break;
                case PLACING_YELLOW_PIXEL:
                    openDeliveryTrayDoor(0.1, 0.1);

                    sleep(1000);

                    openDeliveryTrayDoor(0.0, 0.0);

                    placingState = PlacingState.RETRACTING_ELEVATOR;
                    break;
                case RETRACTING_ELEVATOR:
                    extendElevator(0, 0.8);

                    if (elevatorMotor.getCurrentPosition() < 100) {
                        placingState = PlacingState.RETRACTING_WORM;
                    }
                    break;
                case RETRACTING_WORM:
                    rotateWorm(0, 0.8);

                    if (wormMotor.getCurrentPosition() < 30) {
                        placingState = PlacingState.PLACED;
                    }
                    break;
                case PLACED:
                    break;
            }

            if (placingState == PlacingState.PLACED) {
                return;
            }
        }
    }
}
