package frc.lib.math;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

public class SimpleShootingSpeedCalculator {
    public static double getShooterSpeed(double currentX, double currentY, double targetX, double targetY) {
        double distanceFromGoal2D = getHypothenuse(targetX - currentX, targetY - currentY);
        double distanceFromGoal3D = getHypothenuse(FieldConstants.goalHeight - ShooterConstants.heightFromGround, distanceFromGoal2D);

        return distanceFromGoal3D * ShooterConstants.simpleShootingSpeedMultiplier;
    }

    private static double getHypothenuse(double base, double height) {
        return Math.sqrt(Math.pow(base, 2) + Math.pow(height, 2));
    }
}