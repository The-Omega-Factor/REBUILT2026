package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootAndIndexer extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier shooterSpeed;
    private final DoubleSupplier indexerSpeed;

    public ShootAndIndexer(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterSpeed, DoubleSupplier indexerSpeed) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shooterSpeed;
        this.indexerSpeed = indexerSpeed;
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(shooterSpeed.getAsDouble());
        shooterSubsystem.setIndexerVelocity(indexerSpeed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopIndexer();
        shooterSubsystem.stopShooter();
    }
}
