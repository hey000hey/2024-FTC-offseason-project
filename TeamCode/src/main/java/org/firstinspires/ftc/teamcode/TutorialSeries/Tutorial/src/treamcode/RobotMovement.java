package org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode;

import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.MathFunctions.AngleWrap;

import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.FloatPoint;
import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.Range;
import org.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.com.company.Robot.*;
import static RobotUtilities.MovementVars.*;
import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.MathFunctions.lineCircleIntersection;


public class RobotMovement {


    public static void followCurve(ArrayList<org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint> allPoints, double followAngle) {
//        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition,worldYPosition), allPoints.get(0).followDistance);

//        if(!((worldXPosition - allPoints.get(allPoints.size() - 2).x) < 0.1) || !((worldYPosition - allPoints.get(allPoints.size() - 2).y) < 0.1)) {
//            goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
//        }
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
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));
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


    public static void goToPosition (double x, double Y, double movementSpeed, double preferredAngle, double turnSpeed) {

        //calculates the relative X and Y the bot has to move
        double distanceToTarget = Math.hypot(Y-worldYPosition, x-worldXPosition);

        double absoluteAngleToTarget = Math.atan2(Y-worldYPosition, x-worldXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        // simulator
        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;
    }


}
