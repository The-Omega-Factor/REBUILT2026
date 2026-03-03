package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeState extends Command {
    private final double pivotErrorTolerance;
    private final double spinErrorTolerance;
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier spinSpeed;
    private final DoubleSupplier pivotPos;
    private final boolean willEnd;

    public SetIntakeState(IntakeSubsystem intakeSubsystem, DoubleSupplier spinSpeed, DoubleSupplier pivotPos, boolean willEnd) {
        this.intakeSubsystem = intakeSubsystem;
        this.spinSpeed = spinSpeed;
        this.pivotPos = pivotPos;
        this.willEnd = willEnd;
        this.pivotErrorTolerance = Constants.Intake.pivotErrorTolerance;
        this.spinErrorTolerance = Constants.Intake.spinErrorTolerance;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Intialize Intake Subsystem");
    }

    @Override
    public void execute() {
        double targetPos = pivotPos.getAsDouble();
        //targetPos = MathUtil.clamp(targetPos, Constants.Intake.pivotLowerLimit, Constants.Intake.pivotUpperLimit);

        intakeSubsystem.setIntakeSpinSpeed(spinSpeed.getAsDouble());
        intakeSubsystem.setPivotPosition(targetPos);
    }

    @Override
    public boolean isFinished() {
        if (!willEnd) {
            return false;
        }

        return (Math.abs(intakeSubsystem.getPivotPosition() - pivotPos.getAsDouble()) < pivotErrorTolerance &&
                Math.abs(intakeSubsystem.getSpinSpeed() - spinSpeed.getAsDouble()) < spinErrorTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpinSpeed(0);
        intakeSubsystem.setPivotPosition(intakeSubsystem.getPivotPosition());
        System.out.println("Stopping intake");
    }
}
