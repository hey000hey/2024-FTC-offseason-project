package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;


@Config
@TeleOp
public class SlidePID extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static double target = 0;

    private DcMotorEx left_slide;
    private DcMotorEx right_slide;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left_slide = hardwareMap.get(DcMotorEx.class, "ls");
        right_slide = hardwareMap.get(DcMotorEx.class, "rs");

        left_slide.setDirection(DcMotor.Direction.FORWARD);
        right_slide.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            // target = 0;
     //       if ((gamepad1.right_bumper || gamepad1.left_bumper) && left_slide.getCurrentPosition() < 1000 && left_slide.getCurrentPosition() > 5) {
                if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    target =  500;
                }
                if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                    target = -500;
                }

                //get velocity
                double left_slideVelocity = left_slide.getVelocity();

                // Use our controller to calculate p, i, and d
                controller.setPID(p, i, d);
                double pidLeft = controller.calculate(left_slide.getCurrentPosition(), target);

                if (left_slide.getCurrentPosition() > -1000 && left_slide.getCurrentPosition() < 5) {
                    left_slide.setPower(-pidLeft);
                    //right_slide.setPower(pidLeft);
                }
                telemetry.addData("left position", left_slide.getCurrentPosition());
                telemetry.addData("target", target);
                telemetry.addData("Motor Power", pidLeft);
                telemetry.update();
    //        }
        }
    }
}
