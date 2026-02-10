package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunIndexer extends Command {
    private final ShooterSubsystem indexer;
    private DoubleSupplier velocity;

    public RunIndexer(ShooterSubsystem indexer, DoubleSupplier veloctity) {
        this.indexer = indexer;

        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.setIndexerVelocity(velocity.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIndexer();
    }
}
