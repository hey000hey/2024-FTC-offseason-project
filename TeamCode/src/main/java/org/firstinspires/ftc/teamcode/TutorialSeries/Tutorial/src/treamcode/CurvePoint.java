package org.firstinspires.ftc.teamcode.TutorialSeries.Tutorial.src.treamcode;

import org.opencv.core.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;

    public double slowDownTurnAmount;

    public CurvePoint (double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadius, double slowDownTurnAmount) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public CurvePoint(CurvePoint thisPoint) {
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        slowDownTurnRadians =  thisPoint.slowDownTurnRadians;
        slowDownTurnAmount =  thisPoint.slowDownTurnAmount;
        pointLength =  thisPoint.pointLength;
    }

    public void setPoint(Point point) {
       x = point.x;
       y = point.y;
    }
}
