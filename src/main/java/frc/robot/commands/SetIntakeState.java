package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeState extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier spinSpeed;
    private final DoubleSupplier pivotPos;

    public SetIntakeState(IntakeSubsystem intakeSubsystem, DoubleSupplier spinSpeed, DoubleSupplier pivotPos) {
        this.intakeSubsystem = intakeSubsystem;
        this.spinSpeed = spinSpeed;
        this.pivotPos = pivotPos;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Intialize Intake Subsystem");
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpinSpeed(spinSpeed.getAsDouble());
        intakeSubsystem.setPivotPosition(pivotPos.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpinSpeed(0);
        intakeSubsystem.setPivotPosition(intakeSubsystem.getPivotPosition());
        System.out.println("Stopping intake");
    }
}
