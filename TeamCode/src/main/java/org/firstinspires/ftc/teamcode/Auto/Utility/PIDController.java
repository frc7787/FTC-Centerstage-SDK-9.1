package org.firstinspires.ftc.teamcode.Auto.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    // Values that need to be kept track of between iterations
    private double Kp, Ki, Kd, lastError, integralSum;
    private boolean isReset;
    // Elapsed timer class from SDK
    private ElapsedTime timer;

    public PIDController(double proportionalGain, double integralGain, double derivativeGain) {
        Kp = proportionalGain;
        Ki = integralGain;
        Kd = derivativeGain;
        timer = new ElapsedTime();
        reset();
    }

    public double calculate(double currentPosition, double targetPosition) {
        // Calculate the error
        double error = targetPosition - currentPosition;
        double output = 0;

        if (isReset) {
            // Don't do integrals or derivatives on the first calculation after a reset since they have no meaning
            output = Kp * error;
            lastError = error;
            timer.reset(); // Start the timer for I and D calculations
            isReset = false; // Tell it to do I and D portion on the next run
        } else {
            // Rate of change of the error
            double deltaTime = timer.seconds();
            double derivative = deltaTime != 0 ? (error - lastError) / deltaTime : 0;

            // Sum of all error over time
            integralSum = integralSum + error * deltaTime;

            output = Kp * error + Ki * integralSum + Kd * derivative;
            lastError = error;

            // Reset the timer for the next time
            timer.reset();
        }

        return output;
    }

    public void reset() {
        lastError = 0;
        integralSum = 0;
        isReset = true;
    }
}
