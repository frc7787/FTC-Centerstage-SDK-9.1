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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red - White Pixels", group = "Red")
@Config
public class AutoRedWhitePixels extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(38, -36, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence toWhitePixels = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(38, -12))
                .lineToConstantHeading(new Vector2d(-62, -12))
                .build();

        TrajectorySequence toBackdropWhitePixels = drive.trajectorySequenceBuilder(toWhitePixels.end())
                .lineToConstantHeading(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -36))
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
        Intake.init(hardwareMap);

        Arm.update(false);



        waitForStart();

        if (isStopRequested()) {
            return;
        }

        location = propDetector.getPropLocation();


        // RUN CODE HERE DENNIS YOU BOZO
        Arm.rotateWorm(0);
        Auxiliaries.placePixelOnSpikeStripRight();
        drive.followTrajectorySequence(toWhitePixels);
        Intake.intake();
        sleep(1000);
        Intake.stop();
        drive.followTrajectorySequence(toBackdropWhitePixels);
        Arm.setTargetPos(2401, 1814);
        Arm.update(false);
        Arm.openDeliveryTrayDoorLeft(0.1);
        Arm.openDeliveryTrayDoorRight(0.1);
        sleep(500);
        Arm.openDeliveryTrayDoorLeft(0);
        Arm.openDeliveryTrayDoorRight(0);



        // RUN CODE BEFORE HERE DENNIS YOU BOZO
        sleep(20000);
    }
}
