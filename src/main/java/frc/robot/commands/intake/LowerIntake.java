package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSystem;

public class LowerIntake extends Command {
    private final IntakeSystem intakePivot;

    public LowerIntake(IntakeSystem intakePivot) {
        this.intakePivot = intakePivot;

        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        if (Math.abs(intakePivot.getPivotPosition() - IntakeConstants.PIVOT_DISENGAGED_POSITION) < IntakeConstants.PID_ERROR_TOLERANCE) {
            intakePivot.setIntakePivotPosition(IntakeConstants.PIVOT_ENGAGED_POSITION);
        } else {
            cancel();
        }
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return Math.abs(intakePivot.getPivotPosition()) < IntakeConstants.PID_ERROR_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intakePivot.stopIntakePivot();
        } else {
            intakePivot.setIntakePivotPosition(intakePivot.getPivotPosition());
        }
    }
}
