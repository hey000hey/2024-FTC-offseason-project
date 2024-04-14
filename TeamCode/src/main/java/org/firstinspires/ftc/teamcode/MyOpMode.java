package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.OpMode;

import java.util.ArrayList;
import static org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.RobotMovement.followCurve;

public class MyOpMode extends OpMode {

    ArrayList<org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint> allPoints = new ArrayList<>();
    @Override
    public void init() {
        // Add initialization code here
        System.out.println("Initializing MyOpMode...");
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(0, 0, 1, 1.0, 50, Math.toRadians(50), 1.0 ));
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(180, 180, 1 , 1.0, 50, Math.toRadians(50), 1.0 ));
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(220, 180, 1, 1.0, 50, Math.toRadians(50), 1.0 ));
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(280, 50, 1, 1.0, 10, Math.toRadians(50), 1.0 ));
        allPoints.add(new org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint(180, 10, 1, 1.0, 10, Math.toRadians(50), 1.0 ));


        org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint secondLastPoint = allPoints.get(allPoints.size() - 2);
        org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode.CurvePoint lastPoint = allPoints.get(-1);
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
        System.out.println("LAST POINT.x: " + allPoints.get(-1).x);
        System.out.println("LAST POINT.y: " + allPoints.get(-1).y);
    }

    @Override
    public void loop() {
        // Add main loop code here
        System.out.println("Initialized");
        followCurve(allPoints,Math.toRadians(90));
    }

    public static void main(String[] args) {
        MyOpMode myOpMode = new MyOpMode();
        myOpMode.init();
        myOpMode.loop();
    }
}