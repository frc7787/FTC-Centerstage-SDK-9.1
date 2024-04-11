package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "New and improved auto red backdrop", group = "Red")
public class AutoRedBackdropNew extends LinearOpMode {
    MecanumDriveBase mecanumDriveBase;

    TrajectorySequence toSpikeLeft,
                       toSpikeCenter,
                       toSpikeRight,
                       toBackdropLeft,
                       toBackdropCenter,
                       toBackdropRight,
                       toPark;

    void initRoadRunner() {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);

        mecanumDriveBase.init();

        Pose2d startPose = new Pose2d(11, -63, Math.toRadians(270));

        mecanumDriveBase.setPoseEstimate(startPose);

        toSpikeLeft = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12, -36))
                .lineToLinearHeading(new Pose2d(-4, -32, Math.toRadians(-180)))
                .build();

        toSpikeCenter = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(11, -20, Math.toRadians(0)))
                .build();

        toSpikeRight = mecanumDriveBase.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(25, -20, Math.toRadians(0)))
                .build();

        toBackdropLeft = mecanumDriveBase.trajectorySequenceBuilder(toSpikeLeft.end())
                .strafeTo(new Vector2d(-4, -38))
                .lineToConstantHeading(new Vector2d(38, -30))
                .turn(Math.toRadians(180))
                .build();

        toBackdropCenter = mecanumDriveBase.trajectorySequenceBuilder(toSpikeCenter.end())
                .strafeTo(new Vector2d(11, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -36))
                .build();

        toBackdropRight = mecanumDriveBase.trajectorySequenceBuilder(toSpikeRight.end())
                .strafeTo(new Vector2d(25, -12))
                .lineTo(new Vector2d(38, -12))
                .strafeTo(new Vector2d(38, -42))
                .build();
    }

    @Override public void runOpMode() {

    }
}
