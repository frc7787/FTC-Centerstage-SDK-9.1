package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Test - Potentiometer", group = "Test")
public class PotentiometerTest extends OpMode {

    AnalogInput analogSensor;

    @Override public void init() {
        analogSensor = hardwareMap.analogInput.get("WormPotentiometer");
    }

    @Override public void loop() {
        telemetry.addData("Potentiometer Voltage", analogSensor.getVoltage());
    }
}
