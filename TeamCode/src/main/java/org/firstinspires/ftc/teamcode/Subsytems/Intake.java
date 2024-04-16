package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;
import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_OUTTAKE_POWER;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Class to contain intake subsystem. Not to be confused
 * with the DeliveryTray subsystem.
 */
public class Intake {
    private static DcMotorImplEx intakeMotor;

    private static boolean isActive;

    /**
     * Initializes the motor subsystem.
     * This sets the zero power behavior of the intake motor to float
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        // Expansion Hub Motor Port 3
        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "IntakeMotor");
        intakeMotor.setZeroPowerBehavior(FLOAT);

        isActive = false;
    }

    /**
     * Function to run every iteration of the main TeleOp loop
     * This function updates whether or not the intake is currently active.
     */
    public static void update() {
        isActive = intakeMotor.getPower() != 0;
    }

    /**
     * @return True if the intake is currently running, false if it is not
     */
    public static boolean isActive() {
        return isActive;
    }

    /**
     * Spins the intake in the intake direction (FORWARD) at the supplied power
     *
     * @param intakePower The power to give the intake motor
     */
    public static void intake(double intakePower) {
        intakeMotor.setDirection(REVERSE);
        intakeMotor.setPower(intakePower);
    }

    /**
     * Spins the intake in the intake direction (FORWARD) by the powers defined by DEFAULT_INTAKE_POWER
     * and DEFAULT_BELT_POWER
     */
    public static void intake() {
        intake(DEFAULT_INTAKE_POWER);
    }

    /**
     * Spins the intake in the outtake direction at the speed defined by DEFAULT_OUTTAKE_POWER
     */
    public static void outtake() {
        intakeMotor.setDirection(FORWARD);
        intakeMotor.setPower(DEFAULT_OUTTAKE_POWER);
    }
    /**
     * Stops the intake by setting the power to 0
     */
    public static void stop() {
        intakeMotor.setPower(0);
    }

    /**
     * Displays debug information for the intake. To save loop time, this function DOES NOT call
     * telemetry.update().
     */
    public static void debug(@NonNull Telemetry telemetry, @NonNull CurrentUnit currentUnit) {
        telemetry.addLine("Intake Debug");

        telemetry.addData(
                "Intake direction",
                intakeMotor.getDirection());
        telemetry.addData(
                "Intake power",
                intakeMotor.getPower());
        telemetry.addData(
                "Intake Current",
                intakeMotor.getCurrent(currentUnit));
        telemetry.addData(
                "Intake Zero Power Behaviour",
                intakeMotor.getZeroPowerBehavior());
    }
}
