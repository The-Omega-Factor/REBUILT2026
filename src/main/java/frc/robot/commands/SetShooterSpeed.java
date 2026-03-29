package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.utils.Alliance.getAlliance;
import static frc.robot.utils.ShooterSpeedCalculators.*;

import java.util.function.DoubleSupplier;

public class SetShooterSpeed extends Command {
    private final ShooterSubsystem shooter;
    private final Pose2d currentPose;
    private final ShooterMode mode;
    private final double targetX;
    private final double targetY;
    private final double theta;

    private final DoubleSupplier hopper;

    public SetShooterSpeed(ShooterSubsystem shooter, Pose2d currentPose, ShooterMode mode) {
        this.shooter = shooter;
        this.currentPose = currentPose;
        this.mode = mode;
        this.targetX = getAlliance() == DriverStation.Alliance.Red ? 
                        Field.redX : Field.blueX;
        this.targetY = Constants.Field.y;
        this.theta = Constants.Shooter.shooterLaunchAngle;
        this.hopper = null;
    }

    public SetShooterSpeed(ShooterSubsystem shooter, Pose2d currentPose, ShooterMode mode, DoubleSupplier hopper) {
        this.shooter = shooter;
        this.currentPose = currentPose;
        this.mode = mode;
        this.targetX = getAlliance() == DriverStation.Alliance.Red ? 
                        Field.redX : Field.blueX;
        this.targetY = Constants.Field.y;
        this.theta = Constants.Shooter.shooterLaunchAngle;
        this.hopper = hopper;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing only SHOOTER");
    }

    @Override
    public void execute() {
        switch (mode) {
            case LENGTH -> shooter.setShooterVelocity(length(currentPose, targetX, targetY, Shooter.distanceBasedMultiplier));
            case NODRAG -> shooter.setShooterVelocity(noDrag(currentPose, targetX, targetY, theta, 3));
            default -> System.out.println("No valid mode selected");
        }

        if (hopper != null) {
            shooter.setHopperVelocity(hopper.getAsDouble());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(0);
    }
}
