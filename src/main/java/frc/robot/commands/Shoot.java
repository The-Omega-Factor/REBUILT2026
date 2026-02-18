package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {

    private final ShooterSubsystem shooter;
    private final DoubleSupplier speed;

    public Shoot(ShooterSubsystem shooter, DoubleSupplier speed) {
        this.shooter = shooter;
        this.speed = speed;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing shooter");
    }

    @Override
    public void execute() {
        // Send velocity in RPS
        shooter.setVelocity(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter when command ends
        shooter.setVelocity(0);
        System.out.println("Shooter stopped");
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted
    }
}
