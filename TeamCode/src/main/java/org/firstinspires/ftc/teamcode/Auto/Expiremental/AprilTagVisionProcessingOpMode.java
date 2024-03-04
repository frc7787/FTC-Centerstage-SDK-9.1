package org.firstinspires.ftc.teamcode.Auto.Expiremental;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "April Tag Vision Processing OpMode", group = "Utility")
public class AprilTagVisionProcessingOpMode extends OpMode {
    private final FirstVisionProcessor processor = new FirstVisionProcessor();
    private AprilTagProcessor aprilTagProcessor;

    private static final int DESIRED_TAG_ID = -1; // -1 locks on to any tag

    private VisionPortal visionPortal;

    private int minExposure,
                maxExposure,
                myExposure;

    private int minGain,
                myGain,
                maxGain;

    private int minWhiteBalance,
                myWhiteBalance,
                maxWhiteBalance;

    @Override public void init() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 1);

        getDefaultCameraSettings();

        myExposure     = Math.min(5, minExposure + 1);
        myGain         = maxGain;
        myWhiteBalance = (minWhiteBalance + maxWhiteBalance) / 2;

        setCameraProperties(myExposure, myGain, myWhiteBalance);
    }

    @Override public void loop() {
        telemetry.addData("Exposure", myExposure);
        telemetry.addData("Gain", myGain);
        telemetry.addData("White Balance", myWhiteBalance);

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        int numberOfTags = currentDetections.size();

        if (numberOfTags > 0 ) {
            telemetry.addData("Number of Tags Detected", currentDetections.size());
        } else {
            telemetry.addLine("No tags currently detected");
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) {
                telemetry.addData("Size info not available on tag Id", detection.id);
                continue;
            }

            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                telemetry.addData("Desired Tag Detected. Id", detection.id);
                break;
            }

            telemetry.addData("Tag detected with Id", detection.id);
        }

        if (gamepad1.left_bumper)        myExposure += 1;
        if (gamepad1.left_trigger > 0.5) myExposure -= 1;

        if (gamepad1.right_bumper)        myGain += 1;
        if (gamepad1.right_trigger > 0.5) myGain -= 1;

        if (gamepad1.dpad_up)   myWhiteBalance += 1;
        if (gamepad1.dpad_down) myWhiteBalance -= 1;

        myExposure     = Range.clip(myExposure, minExposure, maxExposure);
        myGain         = Range.clip(myGain, minGain, maxGain);
        myWhiteBalance = Range.clip(myWhiteBalance, minWhiteBalance, maxWhiteBalance);

        setCameraProperties(myExposure, myGain, myWhiteBalance);

        telemetry.update();
    }

    private void getDefaultCameraSettings() {
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) { // Wait for camera to start streaming
            telemetry.addLine("Waiting for camera");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        minWhiteBalance = whiteBalanceControl.getMinWhiteBalanceTemperature();
        maxWhiteBalance = whiteBalanceControl.getMaxWhiteBalanceTemperature();
        myWhiteBalance  = whiteBalanceControl.getWhiteBalanceTemperature();
    }

    private void setCameraProperties(int exposureMS, int gain, int white) {
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) { // Wait for camera to start streaming
            telemetry.addLine("Waiting for camera");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(white);
    }
}