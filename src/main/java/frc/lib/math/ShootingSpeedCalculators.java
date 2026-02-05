package frc.lib.math;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

public class ShootingSpeedCalculators {
    public static double simple(double currentX, double currentY, double targetX, double targetY) {
        double distanceFromGoal2D = getHypothenuse(targetX - currentX, targetY - currentY);
        double distanceFromGoal3D = getHypothenuse(FieldConstants.goalHeight - ShooterConstants.heightFromGround, distanceFromGoal2D);

        return distanceFromGoal3D * ShooterConstants.simpleShootingSpeedMultiplier;
    }

    public static double withoutAirResistance(double currentX, double currentY, double targetX, double targetY, double theta) {
        double x = Math.abs(currentX - targetX);
        double y = Math.abs(currentY - targetY);
        double g = 9.81;

        double v0 = Math.sqrt(
        (g * (x * x)) / 
        (2 * (Math.cos(theta) * Math.cos(theta)) * ((x * Math.tan(theta)) - y))
        );

        return v0 * ShooterConstants.noAirResistanceSpeedMultiplier;
    }

    private static double getHypothenuse(double base, double height) {
        return Math.sqrt(Math.pow(base, 2) + Math.pow(height, 2));
    }
}