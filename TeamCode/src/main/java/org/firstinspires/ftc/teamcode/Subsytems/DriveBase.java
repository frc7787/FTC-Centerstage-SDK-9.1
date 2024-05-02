package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.UP;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;
import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_HIGH;
import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_LOW;
import static org.firstinspires.ftc.teamcode.Properties.STRAFE_OFFSET;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utility.MotorUtility;

/**
 * Class to contain the drive subsystem
 */
public class DriveBase {

    // --------- Hardware Declaration --------- //
    private static IMU imu;
    private static DcMotorImplEx frontLeft, frontRight, backLeft, backRight;
    private static DcMotorImplEx[] driveMotors;

    // --------- IMU Constants ---------- //
    private static final RevHubOrientationOnRobot controlHubOrientation = new RevHubOrientationOnRobot(LEFT, UP);
    private static final IMU.Parameters imuParameters                   = new IMU.Parameters(controlHubOrientation);

    public static void init(@NonNull HardwareMap hardwareMap) {
        // Control Hub Motor Port 0
        frontLeft  = hardwareMap.get(DcMotorImplEx.class, "FrontLeftDriveMotor");
        // Expansion Hub Motor Port 2
        frontRight = hardwareMap.get(DcMotorImplEx.class, "FrontRightDriveMotor");
        // Control Hub Motor Port 1
        backLeft   = hardwareMap.get(DcMotorImplEx.class, "BackLeftDriveMotor");
        // Expansion Hub Motor Port 1
        backRight  = hardwareMap.get(DcMotorImplEx.class, "BackRightDriveMotor");

        driveMotors = new DcMotorImplEx[]{frontLeft, frontRight, backLeft, backRight};

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(imuParameters);
        imu.resetYaw();

        MotorUtility.setDirection(REVERSE, frontLeft, backLeft);
        MotorUtility.setZeroPowerBehaviour(BRAKE, driveMotors);
    }

    /**
     * Returns the value if is outside the range defined by the DEAD_ZONE_LOW and DEAD_ZONE_HIGH values.
     * If the value is in range, returns 0.0d
     * @param value: The value to check
     */
    private static double deadZone(double value) {
        if (DEAD_ZONE_LOW < value && DEAD_ZONE_HIGH > value ) { return 0.0d; }
        return value;
    }

    /**
     * Drives the robot relative to the field
     * 
     * @param drive: The forward translational value
     * @param strafe: The side to side translational value
     * @param turn: THe rotational value
     */
    public static void driveManualFieldCentric(double drive, double strafe, double turn) {
        if (Math.abs(drive)  < 0.05) drive  = 0.0d;
        if (Math.abs(strafe) < 0.05) strafe = 0.0d;
        if (Math.abs(turn)   < 0.05) turn   = 0.0d;

        double theta = Math.atan2(drive, strafe);
        double power = Math.hypot(strafe, drive);

        theta -= imu.getRobotYawPitchRollAngles().getYaw(RADIANS);

        double sin_theta = Math.sin(theta - Math.PI / 4.0);
        double cos_theta = Math.cos(theta - Math.PI / 4.0);

        double max = Math.max(Math.abs(sin_theta), Math.abs(cos_theta));

        double frontLeftPower  = power * cos_theta / max + turn;
        double frontRightPower = power * sin_theta / max - turn;
        double backLeftPower   = power * sin_theta / max + turn;
        double backRightPower  = power * cos_theta / max - turn;

        if ((power + Math.abs(turn)) > 1.0) {
            frontLeftPower  /= power + turn;
            frontRightPower /= power + turn;
            backLeftPower   /= power + turn;
            backRightPower  /= power + turn;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Drives the robot relative to itself
     * 
     * @param drive The forward translational value
     * @param strafe The side to side translational value
     * @param turn The rotational value
     */
    public static void driveManualRobotCentric(double drive, double strafe, double turn) {
        // Our controller has slight drift so we artificially increase the dead zone
        drive  = deadZone(drive);
        strafe = deadZone(strafe) * STRAFE_OFFSET; // Mecanum strafing is not perfect so we slightly correct
        turn   = deadZone(turn);

        double motorPowerRatio = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double fLPower = (drive + strafe + turn) / motorPowerRatio;
        double fRPower = (drive - strafe - turn) / motorPowerRatio;
        double bLPower = (drive - strafe + turn) / motorPowerRatio;
        double bRPower = (drive + strafe - turn) / motorPowerRatio;

        frontLeft.setPower(fLPower);
        frontRight.setPower(fRPower);
        backLeft.setPower(bLPower);
        backRight.setPower(bRPower);
    }

    /**
     * Displays debug information about the drive base
     * @param telemetry The telemetry you are displaying the information on
     */
    public static void debug(@NonNull Telemetry telemetry, @NonNull CurrentUnit currentUnit) {
        telemetry.addLine("Drive Base Debug");

        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Left Direction", frontLeft.getDirection());

        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Front Right Direction", frontRight.getDirection());

        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Left Direction", backLeft.getDirection());

        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.addData("Back Right Direction", backRight.getDirection());

        switch (currentUnit) {
            case AMPS:
                telemetry.addData("Front Left Current (AMPS)", frontLeft.getCurrent(AMPS));
                telemetry.addData("Front Right Current (AMPS)", frontRight.getCurrent(AMPS));
                telemetry.addData("Back Left Current (AMPS)", backLeft.getCurrent(AMPS));
                telemetry.addData("Back Right Current (AMPS)", backRight.getCurrent(AMPS));
                telemetry.addData("Total drive base current (AMPS)",
                        frontLeft.getCurrent(AMPS) + frontRight.getCurrent(AMPS) + backLeft.getCurrent(AMPS) + backRight.getCurrent(AMPS));
            case MILLIAMPS:
                telemetry.addData("Front Left Current (MILLI-AMPS)", frontLeft.getCurrent(MILLIAMPS));
                telemetry.addData("Front Right Current (MILLI-AMPS)", frontRight.getCurrent(MILLIAMPS));
                telemetry.addData("Back Left Current (MILLI-AMPS)", backLeft.getCurrent(MILLIAMPS));
                telemetry.addData("Back Right Current (MILLI-AMPS)", backRight.getCurrent(MILLIAMPS));
                telemetry.addData("Total drive base current (MILLI-AMPS)",
                        frontLeft.getCurrent(MILLIAMPS) + frontRight.getCurrent(MILLIAMPS) + backLeft.getCurrent(MILLIAMPS) + backRight.getCurrent(MILLIAMPS));
        }

    }
}
