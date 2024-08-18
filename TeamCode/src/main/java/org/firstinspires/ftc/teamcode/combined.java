package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyOpMode;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.MathFunctions.AngleWrap;

import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.FloatPoint;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.Range;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.MathFunctions;
import org.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.Robot.*;
import static RobotUtilities.MovementVars.*;
import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.MathFunctions.lineCircleIntersection;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.OpMode;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import java.text.DecimalFormat;
import java.util.ArrayList;



import android.util.Log;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous
public class combined extends LinearOpMode {


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

    private static DcMotor leftFrontDrive = null;
    private static DcMotor leftBackDrive = null;
    private static DcMotor rightFrontDrive = null;
    private static DcMotor rightBackDrive = null;

    public static double followCurve(ArrayList<org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint> allPoints, double followAngle) {
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(BotXPosition,BotYPosition), allPoints.get(0).followDistance);
// checks if the bot is very close to the last point
        if(!((BotXPosition - allPoints.get(allPoints.size() - 2).x) < 0.1) || !((BotYPosition - allPoints.get(allPoints.size() - 2).y) < 0.1)) {
            return goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
        } else return 0;
    }

    public static CurvePoint getFollowPointPath(ArrayList<org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint (pathPoints.get (0));


        //BROKEN: IF THE BOT LOOPS BACK IT WILL JUMP - FIX SOOOOOOOOON
        for(int i = 0; i < pathPoints.size() - 1; i++) {

            org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint startLine = pathPoints.get(i);
            org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint endLine = pathPoints.get(i + 1);

            System.out.println("robot location " + robotLocation);
            System.out.println("followRadius " + followRadius);
            System.out.println("startLine.toPoint()" + startLine.toPoint());
            System.out.println("endLine.toPoint(): " +endLine.toPoint());

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000000;

            for(Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - MyOpMode.BotYPosition, thisIntersection.x - MyOpMode.BotXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - MyOpMode.BotHeading));
                System.out.println("Delta angle " + deltaAngle);
                System.out.println("deltaAngle");

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
//                System.out.println("allPoints: " + allPoints);
//                lastPoint = pathPoints.get(pathPoints.size() -1);
//                double lastSlope = (intersections.get(intersections.size() -1) - intersections.get(intersections.size() -1))
            }
            System.out.println(intersections);
//            System.out.println("Current line: " + i + " Current closest delta angle: " + closestAngle);
        }
        return followMe;
    }


    public static double goToPosition (double x, double Y, double movementSpeed, double preferredAngle, double turnSpeed) {



        //calculates the relative X and Y the bot has to move
        double distanceToTarget = Math.hypot(Y-MyOpMode.BotYPosition, x-MyOpMode.BotXPosition);

        double absoluteAngleToTarget = Math.atan2(Y-MyOpMode.BotYPosition, x-MyOpMode.BotXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (MyOpMode.BotHeading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        RobotLog.e("running go to position");

        leftFrontDrive.setPower(1);
        leftBackDrive.setPower(1);
        rightFrontDrive.setPower(1);
        rightBackDrive.setPower(1);

//        telemetry.addData("lf motor power: ", LeftFrontDrive.getPower());
//        telemetry.addData("lb motor power: ", LeftFrontDrive.getPower());
//        telemetry.addData("rf motor power: ", LeftFrontDrive.getPower());
//        telemetry.addData("lf motor power: ", LeftFrontDrive.getPower());


        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

//        if (relativeTurnAngle > 0.1) {
//            LeftBackDrive
//        }

//        telemetry.addData("inside go to position: bot x", MyOpMode.BotXPosition);
//        telemetry.update();

        return relativeTurnAngle;
    }

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

        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(11, 11, 0.1, 1.0, 50, Math.toRadians(50), 1.0 ));
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(20, 20, 0.1, 1.0, 50, Math.toRadians(50), 1.0 ));
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(30, 30, 0.1, 1.0, 50, Math.toRadians(50), 1.0 ));


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
        RobotLog.e("Initialized");

//        new Thread(() -> {
        while (opModeIsActive() && !isStopRequested()) {
            odometry.updatePose(); // update the position
            PositionTracker.robotPose = odometry.getPose();
//            BotXPosition = PositionTracker.robotPose.getX();
//            BotYPosition = PositionTracker.robotPose.getY();
            BotXPosition = 0;
            BotYPosition = 0;
            BotHeading = PositionTracker.robotPose.getHeading();
            telemetry.addData("X Position", df.format(BotXPosition));
            telemetry.addData("Y Position",df.format(BotYPosition));
            telemetry.addData("Heading", df.format(BotHeading));

            followCurve(allPoints, Math.toRadians(90));

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