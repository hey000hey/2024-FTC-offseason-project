package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MathFunctions.AngleWrap;

import com.qualcomm.robotcore.util.Range;

public class RobotMovement {
    public static void goToPosition (double x, double Y, double movementSpeed, double preferredAngle, double turnSpeed) {

        double worldXPosition=50;
        double worldYPosition=140;
        double worldAngle_rad = Math.toRadians(-180);

        //calculates the relative X and Y the bot has to move
        double distanceToTarget = Math.hypot(y-worldYPosition, x-worldXPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);
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
