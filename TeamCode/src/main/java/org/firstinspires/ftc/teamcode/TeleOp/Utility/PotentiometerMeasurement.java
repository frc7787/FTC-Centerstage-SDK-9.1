package org.firstinspires.ftc.teamcode.TeleOp.Utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Utility - Potentiometer Measurement", group = "Utility")
@Disabled
public class PotentiometerMeasurement extends OpMode {
    private AnalogInput analogSensor;

    @Override public void init() {
        analogSensor = hardwareMap.analogInput.get("WormPotentiometer");
    }

    @Override public void loop() {
        telemetry.addData("Potentiometer Voltage", analogSensor.getVoltage());
        telemetry.update();
    }
}
