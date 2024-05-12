package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class SlidePID extends LinearOpMode {
    private PIDController controller;

    private static double p = 0, i = 0, d = 0, f = 0;

    private static int target = 0;

    private static double ticks_in_degrees = 700/180.0;

    private DcMotorEx left_slide;
    private DcMotorEx right_slide;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left_slide = hardwareMap.get(DcMotorEx.class, "ls");
        right_slide = hardwareMap.get(DcMotorEx.class, "rs");

        waitForStart();

        while (opModeIsActive()) {
            target = 0;
            if ((gamepad1.right_bumper || gamepad1.left_bumper) && left_slide.getCurrentPosition() < 100000 && right_slide.getCurrentPosition() < 1000000 && left_slide.getCurrentPosition() > 5 && right_slide.getCurrentPosition() > 5) {
                if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    target =  500;
                }
                if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                    target = -500;
                }

                //get velocity
                double left_slideVelocity = left_slide.getVelocity();
                double right_slideVelocity = right_slide.getVelocity();

                //use the controller we imported to calculate p,i,d
                controller.setPID(p, i, d);
                double pidLeft = controller.calculate(left_slideVelocity, target);
                double pidRight = controller.calculate(right_slideVelocity, target);

                left_slide.setPower(pidLeft);
                right_slide.setPower(pidRight);

                telemetry.addData("left velocity", left_slideVelocity);
                telemetry.addData("right velocity", right_slideVelocity);
                telemetry.addData("target", target);
                telemetry.update();
            }
        }
    }
}
