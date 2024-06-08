package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ArmTesting extends LinearOpMode {
    DcMotorEx leftArm = null;
    DcMotorEx rightArm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftArm = hardwareMap.get(DcMotorEx.class, "ls");
        rightArm = hardwareMap.get(DcMotorEx.class, "rs");

        leftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double armPower = (gamepad2.right_bumper ? 1 : 0) - (gamepad2.left_bumper ? 1 : 0);
            leftArm.setPower(armPower);
            rightArm.setPower(armPower);

            telemetry.addData("left arm power", leftArm.getPower());
            telemetry.addData("right arm power", rightArm.getPower());
            telemetry.addData("left arm pos", leftArm.getCurrentPosition());
            telemetry.addData("right arm pos", rightArm.getCurrentPosition());
            telemetry.update();
        }
    }
}