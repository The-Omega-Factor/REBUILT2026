package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putBoolean("Will end: ", willEnd);

        if (!willEnd) {
            System.out.println("the command won't end");
            return false;
        }

        double shooterTolerance = Constants.Shooter.shooterErrorTolerance;
        double hopperTolerance = Constants.Shooter.hopperErrorTolerance;
        double shooterError = Math.abs(shooterSubsystem.getShooterVelocity() - shooterSpeed.getAsDouble() * 40);
        double hopperError = Math.abs(shooterSubsystem.getHopperVelocity() - hopperSpeed.getAsDouble() * 40);

        SmartDashboard.putBoolean("First boolean", Math.abs(shooterSubsystem.getShooterVelocity() - shooterSpeed.getAsDouble()) < shooterTolerance);
        SmartDashboard.putBoolean("Second boolean", Math.abs(shooterSubsystem.getHopperVelocity() - hopperSpeed.getAsDouble()) < hopperTolerance);

        SmartDashboard.putNumber("Shooter Error", shooterError);
        SmartDashboard.putNumber("Hopper Error", hopperError);

        SmartDashboard.putNumber("Shooter.getVelocity()", shooterSubsystem.getShooterVelocity());
        SmartDashboard.putNumber("Hopper Veocity", shooterSubsystem.getHopperVelocity());
        SmartDashboard.putNumber("Target Shooter", shooterSpeed.getAsDouble());
        SmartDashboard.putNumber("Target Hopper", hopperSpeed.getAsDouble());

        return shooterError < shooterTolerance && hopperError < hopperTolerance;
    }
}
