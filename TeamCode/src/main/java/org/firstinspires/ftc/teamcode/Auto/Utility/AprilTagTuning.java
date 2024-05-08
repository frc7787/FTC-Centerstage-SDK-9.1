package org.firstinspires.ftc.teamcode.Auto.Utility;

import android.util.Size;

import androidx.annotation.NonNull;

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

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Utility - April Tag Tuning", group = "Utility")
public class AprilTagTuning extends OpMode {
    private final String SD_CARD_PATH = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/";

    private Set<AprilTagDetection> allAprilTagDetections;

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
        initAprilTagDetection();
        getDefaultCameraSettings();

        allAprilTagDetections = new HashSet<>();

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
                allAprilTagDetections.add(detection);

                telemetry.addData("Detected Tag", detection.id);
                telemetry.addData("Distance From Tag", detection.ftcPose.range);
                telemetry.addData("Yaw", detection.ftcPose.yaw);
                telemetry.addData("Pitch", detection.ftcPose.pitch);
                telemetry.addData("Roll", detection.ftcPose.roll);
            }
        }

        listenForCameraPropertyAdjustments();

        myExposure     = Range.clip(myExposure, minExposure, maxExposure);
        myGain         = Range.clip(myGain, minGain, maxGain);
        myWhiteBalance = Range.clip(myWhiteBalance, minWhiteBalance, maxWhiteBalance);

        setCameraProperties(myExposure, myGain, myWhiteBalance);

        telemetry.update();
        if(gamepad1.options){
            saveAprilTagData();
        }
    }

    @Override public void stop() {

    }

    private void initAprilTagDetection() {
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

    private void listenForCameraPropertyAdjustments() {
        if (currentGamepad.left_bumper && !prevGamepad.left_bumper) myExposure ++;

        if (currentGamepad.left_trigger > 0.5 && !(prevGamepad.left_trigger > 0.5)) myExposure --;

        if (currentGamepad.right_bumper && !prevGamepad.right_bumper) myGain ++;

        if (currentGamepad.right_trigger > 0.5 && !(prevGamepad.right_trigger > 0.5)) myGain --;

        if (currentGamepad.dpad_up && !prevGamepad.dpad_up) myWhiteBalance ++;

        if (currentGamepad.dpad_down && !prevGamepad.dpad_down) myWhiteBalance --;

        if (currentGamepad.dpad_left && !prevGamepad.dpad_left) myWhiteBalance += 1;

        if (currentGamepad.dpad_right && !prevGamepad.dpad_right) myWhiteBalance -= 10;
    }

    private void setCameraProperties(int exposureMS, int gain, int white) {
        // Wait for camera to start streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera");
            telemetry.update();
        }

        // Make sure that the telemetry clears properly after we exit the while loop
        telemetry.update();

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(white);
    }

    private void saveAprilTagData() {
        String currentDate =
                new SimpleDateFormat("yyyyMMdd", Locale.CANADA).format(new Date());
        String currentTime =
                new SimpleDateFormat("HHmmss", Locale.CANADA).format(new Date());

        String aprilTagLogFileName = "AprilTagLog_" + currentDate + "_" + currentTime + ".txt";


        String pathToAprilTagLogFile = SD_CARD_PATH + aprilTagLogFileName;



        try {
            File aprilTagLogFile = new File(pathToAprilTagLogFile);
            FileWriter fileWriter         = new FileWriter(aprilTagLogFile, true);
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            for (AprilTagDetection detection: allAprilTagDetections) {
                bufferedWriter.write(String.valueOf(detection.ftcPose.yaw));
                bufferedWriter.newLine();
            }

            bufferedWriter.close();
        } catch (IOException e) {
            telemetry.addData("Failed to write to file", e);
        }

    }
}