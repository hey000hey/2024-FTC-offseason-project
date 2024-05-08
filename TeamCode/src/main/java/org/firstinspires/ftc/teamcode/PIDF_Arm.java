package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDF_Arm extends LinearOpMode {
    private PIDController controller;

    private static double p = 0, i = 0, d = 0, f = 0;

    private static int target = 0;

    private static double ticks_in_degrees = 700/180.0;

    private DcMotorEx arm_motor;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                controller.setPID(p, i, d);
                int armPos = arm_motor.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

                double power = pid * ff;

                arm_motor.setPower(power);

                telemetry.addData("pos", armPos);
                telemetry.addData("target", target);
                telemetry.update();
            }
        }
    }
}
