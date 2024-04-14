package org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import org.opencv.core.Point;

import java.util.ArrayList;

public class MathFunctions {
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }

        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);
        double quadraticC = ((pow(m1,2) * pow(x1,2)) - (2.0 *  y1 * m1 * x1) + pow(y1,2) - pow(radius, 2));

        ArrayList<Point> allPoints = new ArrayList<>();

//        try {
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            System.out.println("A term: " + quadraticA);
            System.out.println("B term: " + quadraticB);
            System.out.println("C term: " + quadraticC);
            System.out.println("m1: " + m1);
            System.out.println("x1: " + x1);
            System.out.println("y1: " + y1);
            System.out.println("Point1: " + linePoint1);
            System.out.println("circleCenter: " + circleCenter);
            System.out.println("xRoot1 " + xRoot1);
            System.out.println("yRoot1 " + yRoot1);
            System.out.println("minX " + minX);
            System.out.println("maxX " + maxX);
            if (xRoot1 > minX && xRoot1 < maxX) {
                System.out.println("top");
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            System.out.println("xRoot2 " + xRoot2);
            System.out.println("yRoot2 " + yRoot2);

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > minX && xRoot2 < maxX) {
                System.out.println("bottom");
                allPoints.add(new Point(xRoot2, yRoot2));
            }

//        }catch(Exception e) {
//            System.out.println(e);
//        }
        System.out.println("all points: " + allPoints);
        return allPoints;
    }
}

