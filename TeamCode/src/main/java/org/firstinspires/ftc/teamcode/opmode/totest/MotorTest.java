package org.firstinspires.ftc.teamcode.archived;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class MotorTest extends LinearOpMode {

    private DcMotorEx motor;

    public static int targetPosition = 0;
    public static double kP = 0.008;
    public static double kI = 0.0;
    public static double kD = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "moto");

        PIDController controller = new PIDController(targetPosition, kP, kI, kD, false);

        waitForStart();

        while (opModeIsActive()) {
            controller.setTargetPosition(targetPosition);
            controller.setPIDConstants(kP, kI, kD);

            double power = controller.update(motor.getCurrentPosition());

            motor.setPower(power);

            telemetry.addData("Current Draw", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Position", motor.getCurrentPosition());
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }
}
