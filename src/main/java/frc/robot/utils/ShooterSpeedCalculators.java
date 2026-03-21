package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSpeedCalculators {
    public static double length(Pose2d current, double multiplier, double targetX, double targetY) {
        double currentX = current.getX();
        double currentY = current.getY();

        return Math.hypot(currentX - targetX, currentY - targetY) * multiplier;
    }
}
