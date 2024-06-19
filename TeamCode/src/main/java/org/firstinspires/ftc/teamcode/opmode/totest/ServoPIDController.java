package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoPIDController {
    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetPos;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;
    private boolean isAngle = true;

    public ServoPIDController(double target, double p, double i, double d, boolean isAngle) {
        kP = p;
        kI = i;
        kD = d;
        targetPos = target;
        this.isAngle = isAngle;
    }

    public void setPIDConstants(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public void setTargetPosition(double target){
        this.targetPos = target;
    }

    public double update(double currentAngle) {
        double error = targetPos - currentAngle;

        // Ensure the error is within the range -180 to 180
        if (error > 180) {
            error -= 360;
        } else if (error <= -180) {
            error += 360;
        }

        // Compute PID terms
        double proportional = kP * error;
        accumulatedError += error;
        double integral = kI * accumulatedError;
        double derivative = kD * (error - lastError);

        // Calculate the output power
        double output = proportional + integral + derivative;

        // Ensure the output is within the range -1 to 1
        output = Math.max(-1, Math.min(1, output));

        // Store current error for next iteration
        lastError = error;

        return output;
    }

    public double getLastSlope() {
        return lastSlope;
    }
}