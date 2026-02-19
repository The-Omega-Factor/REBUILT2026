package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndTransfer extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier shooterSpeed;
    private final DoubleSupplier transferSpeed;

    public ShootAndTransfer(ShooterSubsystem shooter, DoubleSupplier shooterSpeed, DoubleSupplier transferSpeed) {
        this.shooterSubsystem = shooter;
        this.shooterSpeed = shooterSpeed;
        this.transferSpeed = transferSpeed;

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
        shooterSubsystem.setTransferVelocity(transferSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter when command ends
        shooterSubsystem.setShooterVelocity(0);
        shooterSubsystem.setTransferVelocity(0);
        System.out.println("Shooter stopped");
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted
    }
}
