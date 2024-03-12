package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Core.PropColor;
import org.firstinspires.ftc.teamcode.Auto.Core.PropLocation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Auxiliaries;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red - Backdrop", group = "Red")
@Config
public class AutoRedBackdrop extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;

    OpenCvCamera camera;

    MecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetector(PropColor.RED);
        drive = new MecanumDriveBase(hardwareMap);

        drive.init();

        Arm.init(hardwareMap);

        Pose2d startPose = new Pose2d(11, -63, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12, -36))
                .lineToLinearHeading(new Pose2d(-4, -32, Math.toRadians(-180)))
                .build();

        TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(11, -20, Math.toRadians(0)))
                .build();

        TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(25, -20, Math.toRadians(0)))
                .build();

        TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                .strafeTo(new Vector2d(-4, -38))
                .lineToConstantHeading(new Vector2d(38, -36))
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                .strafeTo(new Vector2d(11, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -36))
                .build();

        TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                .strafeTo(new Vector2d(25, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -36))
                .build();

        TrajectorySequence toParkLeft = drive.trajectorySequenceBuilder(toBackdropLeft.end())
                .lineTo(new Vector2d(45, -35))
                .strafeTo(new Vector2d(45, -64))
                .build();

        TrajectorySequence toParkCenter = drive.trajectorySequenceBuilder(toBackdropCenter.end())
                .lineTo(new Vector2d(45, -42))
                .strafeTo(new Vector2d(45, -63))
                .build();

        TrajectorySequence toParkRight = drive.trajectorySequenceBuilder(toBackdropRight.end())
                .lineTo(new Vector2d(45, -50))
                .strafeTo(new Vector2d(45, -63))
                .build();

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

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

        Arm.rotateWorm(25);

        telemetry.addData("PROP LOCATION: ", location);
        telemetry.update();

        switch (location) {
            case LEFT:
                drive.followTrajectorySequence(toSpikeLeft);
                Auxiliaries.placePixelOnSpikeStripRight();
                drive.followTrajectorySequence(toBackdropLeft);
                break;
            case CENTER:
                drive.followTrajectorySequence(toSpikeCenter);
                Auxiliaries.placePixelOnSpikeStripRight();
                drive.followTrajectorySequence(toBackdropCenter);

                break;
            case RIGHT:
                drive.followTrajectorySequence(toSpikeRight);
                Auxiliaries.placePixelOnSpikeStripRight();
                drive.followTrajectorySequence(toBackdropRight);
                break;
            case NONE: // This case should copy center
                drive.followTrajectorySequence(toSpikeCenter);
                sleep(9);
                Auxiliaries.placePixelOnSpikeStripRight();
                drive.followTrajectorySequence(toBackdropCenter);
                break;
        }
        sleep(20000);
    }
}
