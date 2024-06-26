package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.OpMode;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import java.text.DecimalFormat;
import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.RobotMovement.followCurve;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    ArrayList<org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint> allPoints = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        System.out.println("Initializing MyOpMode...");

        MotorEx encoderLeft, encoderRight, encoderPerp;
        encoderLeft = new MotorEx(hardwareMap, "right_back");
        encoderRight = new MotorEx(hardwareMap, "right_back");
        encoderPerp = new MotorEx(hardwareMap, "left_front");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");


        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        HolonomicOdometry odometry = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(0, 11, 0.1, 1.0, 50, Math.toRadians(50), 1.0 ));
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(0, 20, 0.1, 1.0, 50, Math.toRadians(50), 1.0 ));
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(0, 30, 0.1, 1.0, 50, Math.toRadians(50), 1.0 ));


        org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint secondLastPoint = allPoints.get(allPoints.size() - 2);
        org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint lastPoint = allPoints.get(allPoints.size() - 1);
        double m = (lastPoint.y - secondLastPoint.y) / (lastPoint.x - secondLastPoint.x);

        org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint extend = lastPoint;
        if ((secondLastPoint.x - lastPoint.x) < 0) {
            extend.x += 30;
            extend.y += ((m * extend.x) + (m * lastPoint.x));
        } else if ((secondLastPoint.x - lastPoint.x) > 0 ) {
            extend.x -= 30;
            extend.y += ((m * extend.x) + (m * lastPoint.x));
        } else if ((secondLastPoint.y - lastPoint.y) < 0) {
            extend.y += 30;
        } else {
            extend.y -= 30;
        }

        allPoints.add(extend);
        // print the last point
//        System.out.println("LAST POINT.x: " + allPoints.get(-1).x);
//        System.out.println("LAST POINT.y: " + allPoints.get(-1).y);

        waitForStart();

        DecimalFormat df = new DecimalFormat("#.#####");

        // Add main loop code here
        System.out.println("Initialized");

//        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                odometry.updatePose(); // update the position
                PositionTracker.robotPose = odometry.getPose();
                BotXPosition = PositionTracker.robotPose.getX();
                BotYPosition = PositionTracker.robotPose.getY();
                BotHeading = PositionTracker.robotPose.getHeading();
                telemetry.addData("X Position", df.format(BotXPosition));
                telemetry.addData("Y Position",df.format(BotYPosition));
                telemetry.addData("Heading", df.format(BotHeading));

                followCurve(allPoints, Math.toRadians(90), leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

//                telemetry.addData ("relative turn angle: ", followCurve(allPoints, Math.toRadians(90), leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive));
                telemetry.update();

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