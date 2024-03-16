package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MathFunctions.AngleWrap;

public class RobotMovement {
    public static void goToPosition (double x, double Y, double movementSpeed) {

        worldXPosition=50;
        worldYPosition=140;
        worldAngle_rad=Math.toRadians(-180);

        //calculates the relative X and Y the bot has to move
        double distanceToTarget = Math.hypot(y-worldYPosition, x-worldXPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
    }
}
