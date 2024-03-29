package org.firstinspires.ftc.teamcode.Auto;

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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue - Audience", group = "Blue")
public class AutoBlueAudience extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;
    OpenCvCamera camera;

    MecanumDriveBase drive;

    TrajectorySequence toSpikeLeft, toSpikeCenter, toSpikeRight;
    TrajectorySequence toBackdropLeft, toBackdropCenter, toBackdropRight;

    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetector(PropColor.BLUE);
        drive        = new MecanumDriveBase(hardwareMap);

        drive.init();

        Pose2d startPos = new Pose2d(-35, 63, Math.toRadians(90));

        drive.setPoseEstimate(startPos);

        toSpikeLeft = drive.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(-36, 36))
                .lineToLinearHeading(new Pose2d(-20, 32, Math.toRadians(0)))

                .build();

        toSpikeCenter = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-35, 20, Math.toRadians(180)))
                .build();

        toSpikeRight = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-49, 20, Math.toRadians(180)))
                .build();

        toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                .strafeTo(new Vector2d(-20, 38))
                .lineToConstantHeading(new Vector2d(-36, 36))
                .strafeTo(new Vector2d(-36, 12))
                .lineToConstantHeading(new Vector2d(38, 12))
                .strafeTo(new Vector2d(38, 36))

                .build();

        toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                .strafeTo(new Vector2d(-35, 12))
                .lineToConstantHeading(new Vector2d(38, 12))
                .strafeTo(new Vector2d(38, 36))
                .turn(Math.toRadians(180))
                .build();

        toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                .strafeTo(new Vector2d(-49, 12))
                .lineToConstantHeading(new Vector2d(38, 12))
                .strafeTo(new Vector2d(38, 36))
                .turn(Math.toRadians(180))
                .build();


        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

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

        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        Arm.update(false);

        Arm.rotateWorm(1400);

        waitForStart();

        // Pls do not delete this
        location = propDetector.getPropLocation();

        telemetry.addData("Location", location);

        if (isStopRequested()) { return; }

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

        Arm.rotateWorm(25);

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
