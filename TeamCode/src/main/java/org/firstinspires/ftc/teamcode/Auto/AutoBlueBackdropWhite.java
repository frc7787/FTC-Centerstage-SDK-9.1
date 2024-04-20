 package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Properties.AUTO_INITIAL_WORM_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.BEARING_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Properties.CAMERA_RESOLUTION;
import static org.firstinspires.ftc.teamcode.Properties.DESIRED_DISTANCE_FROM_APRIL_TAG_IN;
import static org.firstinspires.ftc.teamcode.Properties.DRIVE_D;
import static org.firstinspires.ftc.teamcode.Properties.DRIVE_GAIN;
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
import static org.firstinspires.ftc.teamcode.Properties.WHITE_PIXEL_ELEVATOR_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.WHITE_PIXEL_WORM_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.YAW_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Properties.YELLOW_PIXEL_ELEVATOR_POSITION_BACKDROP;
import static org.firstinspires.ftc.teamcode.Properties.YELLOW_PIXEL_WORM_POSITION_BACKDROP;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue - Backdrop - White", group = "Blue")
@Config
public class AutoBlueBackdropWhite extends LinearOpMode {
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
                       toPixelStack,
                       getPixels,
                       toBackdropAgain,
                       toPixelStackRight,
                       getPixelsRight,
                       toBackdropAgainRight,
                       toPixelStackLeft,
                       getPixelsLeft,
                       toBackdropAgainLeft,
                       toPark;

    DcMotorImplEx wormMotor, elevatorMotor;

    Servo leftDoor, rightDoor;

    int maxAprilTagDetections = 25;

    enum PlacingState {
        START,
        ROTATING_TO_PLACE_YELLOW_PIXEL,
        EXTENDING_TO_PLACE_YELLOW_PIXEL,
        PLACING_YELLOW_PIXEL,
        CLEARING_PIXELS,
        RETRACTING_ELEVATOR,
        RETRACTING_WORM,
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

        wormMotor     = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");

        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
        wormMotor.setMode(STOP_AND_RESET_ENCODER);

        leftDoor  = hardwareMap.get(ServoImplEx.class, "LeftDoorServo");
        rightDoor = hardwareMap.get(ServoImplEx.class, "RightDoorServo");

        mecanumDriveBase.init();

        Pose2d startPose = new Pose2d(11, 63, Math.toRadians(90));

        mecanumDriveBase.setPoseEstimate(startPose);

        Intake.init(hardwareMap);

        toSpikeLeft = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(23, 34, Math.toRadians(0)))
                .build();

        toSpikeCenter = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(0)))
                .build();

        toSpikeRight = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, 39, Math.toRadians(22)))
                .lineToConstantHeading(new Vector2d(-1.5, 36))
                .build();

        toBackdropLeft = mecanumDriveBase.trajectorySequenceBuilder(toSpikeLeft.end())
                .strafeTo(new Vector2d(23, 38))
                .lineToLinearHeading(new Pose2d(38, 44, Math.toRadians(0)))
                .build();

        toBackdropCenter = mecanumDriveBase.trajectorySequenceBuilder(toSpikeCenter.end())
                .strafeTo(new Vector2d(12, 38))
                .lineToConstantHeading(new Vector2d(38, 36.5))
                .build();

        toBackdropRight = mecanumDriveBase.trajectorySequenceBuilder(toSpikeRight.end())
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(38, 31))
                .build();

        toPixelStackLeft = mecanumDriveBase.trajectorySequenceBuilder(toBackdropLeft.end())
                .strafeTo(new Vector2d(38, 62))
                .lineToConstantHeading(new Vector2d(-36, 64))
                .build();
        getPixelsLeft = mecanumDriveBase.trajectorySequenceBuilder(toPixelStackLeft.end())
                .lineToConstantHeading(new Vector2d(-62, 40))
                .build();

        toBackdropAgainLeft = mecanumDriveBase.trajectorySequenceBuilder(getPixelsLeft.end())
                .lineToConstantHeading(new Vector2d(-36, 64))
                .lineToConstantHeading(new Vector2d(38, 64))
                .strafeTo(new Vector2d(38, 47))
                .build();

        toPixelStack = mecanumDriveBase.trajectorySequenceBuilder(toBackdropCenter.end())
                .strafeTo(new Vector2d(38, 64))
                .lineToConstantHeading(new Vector2d(-36, 64))
                .build();
        getPixels = mecanumDriveBase.trajectorySequenceBuilder(toPixelStack.end())
                .lineToConstantHeading(new Vector2d(-62, 40))
                .build();

        toBackdropAgain = mecanumDriveBase.trajectorySequenceBuilder(getPixels.end())
                .lineToConstantHeading(new Vector2d(-36, 64))
                .lineToConstantHeading(new Vector2d(38, 64))
                .strafeTo(new Vector2d(38, 42))
                .build();

        toPixelStackRight = mecanumDriveBase.trajectorySequenceBuilder(toBackdropRight.end())
                .strafeTo(new Vector2d(38, 64))
                .lineToConstantHeading(new Vector2d(-36, 64))
                .build();
        getPixelsRight = mecanumDriveBase.trajectorySequenceBuilder(toPixelStackRight.end())
                .lineToConstantHeading(new Vector2d(-62, 40))
                .build();

        toBackdropAgainRight = mecanumDriveBase.trajectorySequenceBuilder(getPixelsRight.end())
                .lineToConstantHeading(new Vector2d(-36, 64))
                .lineToConstantHeading(new Vector2d(38, 62))
                .strafeTo(new Vector2d(38, 42))
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

        Auxiliaries.init(hardwareMap);

        rotateWorm(AUTO_INITIAL_WORM_POSITION, 1.0);

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

        rotateWorm(0, 1.0);

        sleep(0);

        switch (location) {
            case LEFT:
                mecanumDriveBase.followTrajectorySequence(toSpikeLeft);
                Auxiliaries.retractPixelPlacerServo();
                mecanumDriveBase.followTrajectorySequence(toBackdropLeft);

                centerOnAprilTag(1);

                placePixelOnBackdrop(
                        YELLOW_PIXEL_WORM_POSITION_BACKDROP,
                        YELLOW_PIXEL_ELEVATOR_POSITION_BACKDROP,
                        1000);
                placingState = placingState.START;
                mecanumDriveBase.followTrajectorySequence(toPixelStackLeft);
                openDeliveryTrayDoor(0.25, 0.25);
                Intake.intake();
                elevatorMotor.setPower(-0.2);
                mecanumDriveBase.followTrajectorySequence(getPixelsLeft);
                sleep(1500);
                Intake.stop();
                sleep(2000);
                mecanumDriveBase.followTrajectorySequence(toBackdropAgainLeft);

                elevatorMotor.setPower(-0.0);
                openDeliveryTrayDoor(0.0, 0.0);
                placePixelOnBackdrop(
                        YELLOW_PIXEL_WORM_POSITION_BACKDROP,
                        YELLOW_PIXEL_ELEVATOR_POSITION_BACKDROP,
                        1000);
                break;
            case CENTER:
            case NONE:
                mecanumDriveBase.followTrajectorySequence(toSpikeCenter);
                Auxiliaries.retractPixelPlacerServo();
                mecanumDriveBase.followTrajectorySequence(toBackdropCenter);

                centerOnAprilTag(2);

                placePixelOnBackdrop(
                        YELLOW_PIXEL_WORM_POSITION_BACKDROP,
                        YELLOW_PIXEL_ELEVATOR_POSITION_BACKDROP,
                        1000);
                placingState = placingState.START;
                mecanumDriveBase.followTrajectorySequence(toPixelStack);
                openDeliveryTrayDoor(0.25, 0.25);
                Intake.intake();
                elevatorMotor.setPower(-0.2);
                mecanumDriveBase.followTrajectorySequence(getPixels);
                sleep(1500);
                Intake.stop();
                sleep(2000);
                mecanumDriveBase.followTrajectorySequence(toBackdropAgain);

                elevatorMotor.setPower(-0.0);
                openDeliveryTrayDoor(0.0, 0.0);
                placePixelOnBackdrop(
                        YELLOW_PIXEL_WORM_POSITION_BACKDROP,
                        YELLOW_PIXEL_ELEVATOR_POSITION_BACKDROP,
                        1000);
                break;
            case RIGHT:
                mecanumDriveBase.followTrajectorySequence(toSpikeRight);
                Auxiliaries.retractPixelPlacerServo();
                mecanumDriveBase.followTrajectorySequence(toBackdropRight);

                centerOnAprilTag(3);

                placePixelOnBackdrop(
                        YELLOW_PIXEL_WORM_POSITION_BACKDROP,
                        YELLOW_PIXEL_ELEVATOR_POSITION_BACKDROP,
                        1000);
                placingState = placingState.START;
                mecanumDriveBase.followTrajectorySequence(toPixelStackRight);
                openDeliveryTrayDoor(0.25, 0.25);
                Intake.intake();
                elevatorMotor.setPower(-0.2);
                mecanumDriveBase.followTrajectorySequence(getPixelsRight);
                sleep(700);
                mecanumDriveBase.followTrajectorySequence(toBackdropAgainRight);
                Intake.stop();
                elevatorMotor.setPower(-0.0);
                openDeliveryTrayDoor(0.0, 0.0);
                placePixelOnBackdrop(
                        WHITE_PIXEL_WORM_POSITION,
                        WHITE_PIXEL_ELEVATOR_POSITION,
                        1000);
                break;
        }

        toPark = mecanumDriveBase.trajectorySequenceBuilder(mecanumDriveBase.getPoseEstimate())
                        .strafeTo(new Vector2d(45, 60))
                        .lineTo(new Vector2d(60, 60))
                        .build();

        mecanumDriveBase.followTrajectorySequence(toPark);

        sleep(2000);
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
     * Extends the elevator to the provided position at the provided power
     * @param targetPos The position to move the elevator to
     * @param power The power to move to the target position at
     */
    private void extendElevator(int targetPos, double power) {
        elevatorMotor.setTargetPosition(targetPos);
        elevatorMotor.setMode(RUN_TO_POSITION);
        elevatorMotor.setPower(power);
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

            if (isWithinTolerance()) return;

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

    private void placePixelOnBackdrop(int wormPos, int elevatorPos, int outtakeSleep) {

        while (true) {
            if (isStopRequested() || !opModeIsActive()) return;

            switch (placingState) {
                case START:
                    rotateWorm(wormPos + 25, 1.0);

                    placingState = PlacingState.ROTATING_TO_PLACE_YELLOW_PIXEL;
                    break;
                case ROTATING_TO_PLACE_YELLOW_PIXEL:
                    if (wormMotor.getCurrentPosition() >= 710) {
                        placingState = PlacingState.EXTENDING_TO_PLACE_YELLOW_PIXEL;
                    }

                    break;
                case EXTENDING_TO_PLACE_YELLOW_PIXEL:
                    extendElevator(elevatorPos + 45, 1.0);

                    if (elevatorMotor.getCurrentPosition() >= elevatorPos - 5) {
                        placingState = PlacingState.PLACING_YELLOW_PIXEL;
                    }

                    break;
                case PLACING_YELLOW_PIXEL:
                    openDeliveryTrayDoor(0.1, 0.1);

                    sleep(outtakeSleep);

                    openDeliveryTrayDoor(0.0, 0.0);

                    placingState = PlacingState.CLEARING_PIXELS;
                    break;
                case CLEARING_PIXELS:
                    rotateWorm(wormPos + 200, 0.8);

                    sleep(400);

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

                    placingState = PlacingState.PLACED;
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