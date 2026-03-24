package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class ShooterSpeedCalculators {
    private static final double g = Constants.gravity;
    private static final double theta = Math.toRadians(Constants.Shooter.shooterLaunchAngle);
    private static final double h = Constants.Shooter.heightDisplacement;
    private static final double delta = Constants.Field.goalShortWidth/2;

    public enum ShooterMode {
        LENGTH,
        NODRAG
    }

    public static double length(Pose2d current, double targetX, double targetY,  double multiplier) {
        return getLength(current, targetX, targetY) * multiplier;
    }

    public static double noDrag(Pose2d current, double targetX, double targetY, double theta, int accuracy) {
        double r = Math.hypot(current.getX() - targetX, current.getY() - targetY);

        for (double i = 0; i <= delta; i += delta/accuracy) {
            if ((r + i*delta/accuracy) - h > 0) {
                return noDragCompute(r + i*delta/accuracy);
            } else if ((r - i*delta/accuracy) - h > 0) {
                return noDragCompute(r - i*delta/accuracy);
            }
        }

        return 0.0;
    }

    private static double getLength(Pose2d current, double targetX, double targetY) {
        double currentX = current.getX();
        double currentY = current.getY();

        return Math.hypot(currentX - targetX, currentY - targetY);
    }

    private static double noDragCompute(double r) {
        return Math.sqrt(
            (g * r * r)/
            ((2 * Math.pow(Math.cos(theta), 2)) * (r * Math.tan(theta) - h))
        );
    }
}
