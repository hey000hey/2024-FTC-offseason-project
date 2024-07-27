package org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyOpMode;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.MathFunctions.AngleWrap;

import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.FloatPoint;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.Range;
import org.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.Robot.*;
import static RobotUtilities.MovementVars.*;
import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.MathFunctions.lineCircleIntersection;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class RobotMovement {


    public static double followCurve(ArrayList<org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint> allPoints, double followAngle, DcMotor LeftFrontDrive, DcMotor RightFrontDrive, DcMotor LeftBackDrive, DcMotor RightBackDrive) {
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(MyOpMode.BotXPosition,MyOpMode.BotYPosition), allPoints.get(0).followDistance);
// checks if the bot is very close to the last point
//        if(!((MyOpMode.BotXPosition - allPoints.get(allPoints.size() - 2).x) < 0.1) || !((MyOpMode.BotYPosition - allPoints.get(allPoints.size() - 2).y) < 0.1)) {
            return goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed, LeftFrontDrive, RightFrontDrive, LeftBackDrive, RightBackDrive);
//        } else return 0;
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


    public static double goToPosition (double x, double Y, double movementSpeed, double preferredAngle, double turnSpeed, DcMotor LeftFrontDrive, DcMotor RightFrontDrive, DcMotor LeftBackDrive, DcMotor RightBackDrive) {



        //calculates the relative X and Y the bot has to move
        double distanceToTarget = Math.hypot(Y-MyOpMode.BotYPosition, x-MyOpMode.BotXPosition);

        double absoluteAngleToTarget = Math.atan2(Y-MyOpMode.BotYPosition, x-MyOpMode.BotXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (MyOpMode.BotHeading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;


        // vedant/maxwell movement code
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));
//
//        LeftFrontDrive.setPower(movementXPower);
//        LeftBackDrive.setPower(movementXPower);
//        RightFrontDrive.setPower(movementYPower);
//        RightBackDrive.setPower(movementYPower);
//
//        ApplyMovement();
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

//        double tl_power_raw = movementYPower-movement_turn+movementXPower*1.5;
//        double bl_power_raw = movementYPower-movement_turn- movementXPower*1.5;
//        double br_power_raw = -movementYPower-movement_turn-movementXPower*1.5;
//        double tr_power_raw = -movementYPower-movement_turn+movementXPower*1.5;

        double tl_power_raw = movementYPower-movement_turn+movementXPower;
        double bl_power_raw = movementYPower-movement_turn- movementXPower;
        double br_power_raw = -movementYPower-movement_turn-movementXPower;
        double tr_power_raw = -movementYPower-movement_turn+movementXPower;

        telemetry.addData("movement y power", movementYPower);
        telemetry.addData("movement turn", movement_turn);
        telemetry.addData("movement X power", movementXPower);

        telemetry.update();

        LeftFrontDrive.setPower(tl_power_raw);
        LeftBackDrive.setPower(bl_power_raw);
        RightFrontDrive.setPower(br_power_raw);
        RightBackDrive.setPower(tr_power_raw);

//        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        RightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // hardcode test
//        LeftFrontDrive.setPower(1);
//        LeftBackDrive.setPower(1);
//        RightFrontDrive.setPower(1);
//        RightBackDrive.setPower(1);

//        telemetry.addData("lf motor power: ", LeftFrontDrive.getPower());
//        telemetry.addData("lb motor power: ", LeftFrontDrive.getPower());
//        telemetry.addData("rf motor power: ", LeftFrontDrive.getPower());
//        telemetry.addData("lf motor power: ", LeftFrontDrive.getPower());

//        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

//        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

//        if (relativeTurnAngle > 0.1) {
//            LeftBackDrive
//        }

//        telemetry.addData("inside go to position: bot x", MyOpMode.BotXPosition);
//        telemetry.update();

        return relativeTurnAngle;
    }

    //gluten free movement code
    /*converts movement_y, movement_x, movement_turn into motor powers */
    public static void ApplyMovement() {

        double lf_power_raw = movement_y-movement_turn+movement_x*1.5;
        double lb_power_raw = movement_y-movement_turn- movement_x*1.5;
        double rb_power_raw = -movement_y-movement_turn-movement_x*1.5;
        double rf_power_raw = -movement_y-movement_turn+movement_x*1.5;




        //find the maximum of the powers
        double maxRawPower = Math.abs(lf_power_raw);
        if(Math.abs(lb_power_raw) > maxRawPower){ maxRawPower = Math.abs(lb_power_raw);}
        if(Math.abs(rb_power_raw) > maxRawPower){ maxRawPower = Math.abs(rb_power_raw);}
        if(Math.abs(rf_power_raw) > maxRawPower){ maxRawPower = Math.abs(rf_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }

        //double scaleDownAmount = Math.max(Math.abs(lf_power_raw), Math.abs(lb_power_raw), Math.abs(rb_power_raw), Math.abs(rf_power_raw), 1);
        lf_power_raw *= scaleDownAmount;
        lb_power_raw *= scaleDownAmount;
        rb_power_raw *= scaleDownAmount;
        rf_power_raw *= scaleDownAmount;


        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS (idk how they do that -alan)
//        LeftFrontDrive.setPower(lf_power_raw);
//        LeftBackDrive.setPower(lb_power_raw);
//        RightBackDrive.setPower(rf_power_raw);
//        RightFrontDrive.setPower(rf_power_raw);
    }

}
