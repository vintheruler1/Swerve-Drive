package org.firstinspires.ftc.teamcode.archived;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class CoolTest extends LinearOpMode {

    CRServo servo;
    AnalogInput pot;
    public static double targ = 90;

    public static double kP = 0.008; // 0.009
    public static double kI = 0.0;
    public static double kD = 0.1; // 0.07
    private DcMotorEx motor;

    public static double mp = 0;

    private double loopTime;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = hardwareMap.get(CRServo.class, "Servo");
        pot = hardwareMap.get(AnalogInput.class, "servo_pot");
        motor = hardwareMap.get(DcMotorEx.class, "moto");

        ServoPIDController pidController = new ServoPIDController(targ, kP, kI, kD, false);

        waitForStart();

        while (opModeIsActive()) {

            double currentAngle = pot.getVoltage() / 3.3 * 360;

            pidController.setTargetPosition(targ);

            pidController.setPIDConstants(kP, kI, kD);

            double power = pidController.update(currentAngle);

            motor.setPower(mp);

            servo.setPower(power);

            telemetry.addData("Power", power);
            telemetry.addData("Target", targ);
            telemetry.addData("Position", currentAngle);
            telemetry.addData("Power", power);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();

        }
    }
}
