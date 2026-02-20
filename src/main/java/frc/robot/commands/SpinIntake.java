package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinIntake extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier spinSpeed;

    public SpinIntake(IntakeSubsystem intakeSubsystem, DoubleSupplier spinSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.spinSpeed = spinSpeed;
    }

    @Override
    public void initialize() {
        System.out.println("Intialize Intake Subsystem");
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpinSpeed(spinSpeed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpinSpeed(0);
        System.out.println("Stopping intake");
    }
}
