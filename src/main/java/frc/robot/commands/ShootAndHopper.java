package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndHopper extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier shooterSpeed;
    private final DoubleSupplier hopperSpeed;
    private final boolean willEnd;

    public ShootAndHopper(ShooterSubsystem shooter, DoubleSupplier shooterSpeed, DoubleSupplier hopperSpeed, boolean willEnd) {
        this.shooterSubsystem = shooter;
        this.shooterSpeed = shooterSpeed;
        this.hopperSpeed = hopperSpeed;
        this.willEnd = willEnd;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing shooter");
    }

    @Override
    public void execute() {
        // Send velocity in RPS
        shooterSubsystem.setShooterVelocity(shooterSpeed.getAsDouble());
        shooterSubsystem.setHopperVelocity(hopperSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter when command ends
        shooterSubsystem.setShooterVelocity(0);
        shooterSubsystem.setHopperVelocity(0);
        System.out.println("Shooter stopped");
    }

    @Override
    public boolean isFinished() {
        if (!willEnd) {
            return false;
        }

        double shooterTolerance = Constants.Shooter.shooterErrorTolerance;
        double hopperTolerance = Constants.Shooter.hopperErrorTolerance;

        return Math.abs(shooterSubsystem.getShooterVelocity() - shooterSpeed.getAsDouble()) < shooterTolerance &&
            Math.abs(shooterSubsystem.getHopperVelocity() - hopperSpeed.getAsDouble()) < hopperTolerance;
    }
}
