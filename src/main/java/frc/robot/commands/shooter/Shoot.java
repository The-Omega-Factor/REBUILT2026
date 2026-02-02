package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier velocitySupplier;

    public Shoot(ShooterSubsystem shooterSubsystem, DoubleSupplier velocitySupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.velocitySupplier = velocitySupplier;

        this.addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(velocitySupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterVelocity(0);
    }
}
