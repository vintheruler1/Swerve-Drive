package org.firstinspires.ftc.teamcode.common.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utility.AbsoluteAnalogEncoder;

import java.util.Locale;

@Config
public class SwervePod {

    // PID Coefficients
    public static double P = 0.6, I = 0, D = 0.1;

    // Maximum servo and motor power
    public static double MAX_SERVO_POWER = 1, MAX_MOTOR_POWER = 1;

    // Motor flipping
    public static boolean MOTOR_FLIPPING = true;

    // Wheel specifications
    public static double WHEEL_RADIUS_INCHES = 3; // in inches
    public static double GEAR_RATIO = 1 / (30 / 14.0 * 30 / 20.0 * 2); // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 28;

    private DcMotorEx driveMotor;
    private CRServo steeringServo;
    private AbsoluteAnalogEncoder steeringEncoder;
    private PIDFController steeringController;
    private double rotationError = 0;
    private boolean wheelFlipped = false;
    private double targetAngle = 0.0;
    private double currentAngle = 0.0;
    private double lastMotorPower = 0;

    private Telemetry telemetry;

    public SwervePod(DcMotorEx driveMotor, CRServo steeringServo, AbsoluteAnalogEncoder steeringEncoder, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize drive motor
        this.driveMotor = driveMotor;
        MotorConfigurationType motorConfig = driveMotor.getMotorType().clone();
        motorConfig.setAchieveableMaxRPMFraction(MAX_MOTOR_POWER);
        driveMotor.setMotorType(motorConfig);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize steering servo
        this.steeringServo = steeringServo;
        ((CRServoImplEx) steeringServo).setPwmRange(new PwmControl.PwmRange(1000, 2000, 5000));

        // Initialize encoder and PID controller
        this.steeringEncoder = steeringEncoder;
        this.steeringController = new PIDFController(P, I, D, 0);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Read the current position of the steering module
    public void readSteeringPosition() {
        currentAngle = steeringEncoder.getCurrentPosition();
    }

    // Update the PID controller and set the power to the steering servo
    public void updateSteering() {
        steeringController.setPIDF(P, I, D, 0);
        double targetAngle = getTargetSteeringAngle();
        double currentAngle = getCurrentSteeringAngle();

        rotationError = normalizeRadians(targetAngle - currentAngle);

        if (MOTOR_FLIPPING && Math.abs(rotationError) > Math.PI / 2) {
            targetAngle = normalizeRadians(targetAngle - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        rotationError = normalizeRadians(targetAngle - currentAngle);
        double servoPower = Range.clip(steeringController.calculate(0, rotationError), -MAX_SERVO_POWER, MAX_SERVO_POWER);

        if (Double.isNaN(servoPower)) {
            servoPower = 0;
        }

        steeringServo.setPower(servoPower);
    }

    public double getTargetSteeringAngle() {
        return normalizeRadians(targetAngle - Math.PI);
    }

    public double getCurrentSteeringAngle() {
        return normalizeRadians(currentAngle - Math.PI);
    }

    public void setDriveMotorPower(double power) {
        if (wheelFlipped) {
            power *= -1;
        }
        lastMotorPower = power;
        driveMotor.setPower(power);
    }

    public void setTargetSteeringAngle(double targetAngle) {
        this.targetAngle = normalizeRadians(targetAngle);
    }

    public String getTelemetryData(String name) {
        return String.format(Locale.ENGLISH,
                "%s: Motor Flipped: %b \nCurrent Position: %.2f, Target Position: %.2f, Flip Modifier: %d, Motor Power: %.2f",
                name, wheelFlipped, getCurrentSteeringAngle(), getTargetSteeringAngle(), getFlipModifier(), lastMotorPower);
    }

    public int getFlipModifier() {
        return wheelFlipped ? -1 : 1;
    }

    public void setDriveMotorMode(DcMotor.RunMode runMode) {
        driveMotor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        driveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        driveMotor.setPIDFCoefficients(runMode, coefficients);
    }

    public double getSteeringServoPower() {
        return steeringServo.getPower();
    }

    public double getWheelPositionInches() {
        return encoderTicksToInches(driveMotor.getCurrentPosition());
    }

    public double getWheelVelocityInchesPerSec() {
        return encoderTicksToInches(driveMotor.getVelocity());
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(this);
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS_INCHES * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static class SwerveModuleState {
        private SwervePod module;
        private double wheelPosition, podRotation;

        public SwerveModuleState(SwervePod module) {
            this.module = module;
            this.wheelPosition = 0;
            this.podRotation = 0;
        }

        public SwerveModuleState updateState() {
            return setState(-module.getWheelPositionInches(), module.getCurrentSteeringAngle());
        }

        public SwerveModuleState setState(double wheelPosition, double podRotation) {
            this.wheelPosition = wheelPosition;
            this.podRotation = podRotation;
            return this;
        }

        public double getWheelPosition() {
            return wheelPosition;
        }

        public double getPodRotation() {
            return podRotation;
        }
    }
}
