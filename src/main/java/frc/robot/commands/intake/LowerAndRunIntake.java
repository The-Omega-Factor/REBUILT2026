package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSystem;

/**
 * Lower the intake (if it is not already there) while running the intake roller
 */

public class LowerAndRunIntake extends Command {
    private final IntakeSystem intakeSystem;
    private final DoubleSupplier speed;

    public LowerAndRunIntake(IntakeSystem intakeSystem, DoubleSupplier speed) {
        this.intakeSystem = intakeSystem;
        this.speed = speed;

        addRequirements(intakeSystem);
    }

    @Override
    public void initialize() {
        if (Math.abs(intakeSystem.getPivotPosition() - IntakeConstants.PIVOT_DISENGAGED_POSITION) < IntakeConstants.PID_ERROR_TOLERANCE) {
            intakeSystem.setIntakePivotPosition(IntakeConstants.PIVOT_ENGAGED_POSITION);
        } else {
            cancel();
        }
    }

    @Override
    public void execute() {
        intakeSystem.setIntakeSpeed(speed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intakeSystem.stopIntakePivot();
        } else {
            intakeSystem.setIntakePivotPosition(intakeSystem.getPivotPosition());
        }

        intakeSystem.setIntakeSpeed(0.0);
    }
}
