package org.firstinspires.ftc.teamcode.Auto.Utility;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
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

@TeleOp(name = "Utility - April Tag Tuning", group = "Utility")
@Disabled
public class AprilTagTuning extends OpMode {
    private AprilTagProcessor aprilTagProcessor;

    private static final int DESIRED_TAG_ID = 5; // -1 locks on to any tag

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

    private Gamepad currentGamepad, prevGamepad;

    @Override public void init() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .setAutoStopLiveView(true)
                .build();

        getDefaultCameraSettings();

        myExposure     = 2;
        myGain         = 0;
        myWhiteBalance = 4000;

        setCameraProperties(myExposure, myGain, myWhiteBalance);

        prevGamepad    = new Gamepad();
        currentGamepad = new Gamepad();
    }

    @Override public void loop() {
        prevGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

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
            if (detection.metadata == null) continue;

            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                telemetry.addData("Detected Tag", detection.id);
                telemetry.addData("Distance From Detected Tag", detection.ftcPose.range);
                telemetry.addData("Yaw From Tag", detection.ftcPose.yaw);
                telemetry.addData("Pitch From Tag", detection.ftcPose.pitch);
                telemetry.addData("Roll From Tag", detection.ftcPose.roll);
            }
        }

        listenForCameraAdjust();

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

    private void listenForCameraAdjust() {
        if (currentGamepad.left_bumper && !prevGamepad.left_bumper) {
            myExposure ++;
        }

        if (currentGamepad.left_trigger > 0.5 && !(prevGamepad.left_trigger > 0.5)) {
            myExposure --;
        }

        if (currentGamepad.right_bumper && !prevGamepad.right_bumper) {
            myGain ++;
        }

        if (currentGamepad.right_trigger > 0.5 && !(prevGamepad.right_trigger > 0.5)) {
            myGain --;
        }

        if (currentGamepad.dpad_up && !prevGamepad.dpad_up) {
            myWhiteBalance ++;
        }

        if (currentGamepad.dpad_down && !prevGamepad.dpad_down) {
            myWhiteBalance --;
        }

        if (currentGamepad.dpad_left && !prevGamepad.dpad_left) {
            myWhiteBalance += 10;
        }

        if (currentGamepad.dpad_right && !prevGamepad.dpad_right) {
            myWhiteBalance -= 10;
        }
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