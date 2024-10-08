package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.OpMode;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.localization.TwoDeadWheelLocalizer;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import java.text.DecimalFormat;
import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.teamcode.localization.TwoDeadWheelLocalizer.getPose;

import static RobotUtilities.MovementVars.movement_turn;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class MyOpMode extends LinearOpMode {


    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 12;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 5;

    public static final double WHEEL_DIAMETER = 1.5;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;

    public static final double TICKS_TO_INCHES = 0.0005752428;

    public static double BotXPosition;
    public static double BotYPosition;
    public static double BotHeading;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    ArrayList<CurvePoint> allPoints = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");

        TwoDeadWheelLocalizer tdwl = new TwoDeadWheelLocalizer(hardwareMap, imu, 0.0029683465, 0.0029683465);

        System.out.println("Initializing MyOpMode...");

//        MotorEx encoderLeft, encoderRight;
//        encoderLeft = new MotorEx(hardwareMap, "left_front"); // parallel
//        encoderRight = new MotorEx(hardwareMap, "right_back");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

//        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        allPoints.add(new CurvePoint(48, 0, 0.05, 1.0, 3, Math.toRadians(50), 1.0 ));
        allPoints.add(new CurvePoint(48, 48, 0.05, 1.0, 3, Math.toRadians(50), 1.0 ));
        allPoints.add(new CurvePoint(0, 48, 0.05, 1.0, 3, Math.toRadians(50), 1.0 ));

//        allPoints.add(new CurvePoint(20, 0, 0.05, 1.0, 5, Math.toRadians(50), 1.0 ));
//        allPoints.add(new CurvePoint(40, 0, 0.05, 1.0, 5, Math.toRadians(50), 1.0 ));
//        allPoints.add(new CurvePoint(60, 0, 0.05, 1.0, 5, Math.toRadians(50), 1.0 ));

        waitForStart();

        DecimalFormat df = new DecimalFormat("#.#####");

        // Add main loop code here
        System.out.println("Initialized");
        RobotMovement movement = new RobotMovement();



//        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                BotXPosition = tdwl.getPose().position.x;
                BotYPosition = tdwl.getPose().position.y;
                BotHeading = tdwl.getPose().heading.toDouble();
                telemetry.addData("X Position", df.format(BotXPosition));
                telemetry.addData("Y Position",df.format(BotYPosition));
                telemetry.addData("Heading", df.format(Math.toDegrees(BotHeading)));
                telemetry.addData("movement y power", movement.movementYPower);
                telemetry.addData("movement turn", movement_turn);
                telemetry.addData("movement X power", movement.movementXPower);
                telemetry.addData("relative turn angle", movement.relativeTurnAngle);
                telemetry.update();


                movement.followCurve(allPoints, 0, leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, telemetry);

//                telemetry.addData ("relative turn angle: ", followCurve(allPoints, Math.toRadians(90), leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive));


            }
//        }).start();

//        new Thread(() -> {
//            while (opModeIsActive() && !isStopRequested()) {
//           telemetry.addData ("relative turn angle: ", followCurve(allPoints, Math.toRadians(90), leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive));
//           telemetry.update();
//          }
//        }).start();

    }
}