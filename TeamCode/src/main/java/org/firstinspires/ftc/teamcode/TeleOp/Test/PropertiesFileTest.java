package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Properties;
import org.firstinspires.ftc.teamcode.RobotPropertyParser;

@TeleOp(name = "Test - Properties File", group = "Test")
public class PropertiesFileTest extends OpMode {

    @Override public void init() {}

    @Override public void loop() {
        telemetry.addData("Test Property", RobotPropertyParser.getDouble("TEST_PROPERTY"));
    }
}
