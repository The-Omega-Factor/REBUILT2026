package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSystem;

public class RunIntake extends Command {
    private final IntakeSystem intakeSpin;
    private final DoubleSupplier speed;

    public RunIntake(IntakeSystem intakeSystem, DoubleSupplier speed) {
        this.intakeSpin = intakeSystem;
        this.speed = speed;

        addRequirements(intakeSystem);
    }

    @Override
    public void execute() {
        intakeSpin.setIntakeSpeed(speed.getAsDouble());
        intakeSpin.holdIntakePivot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSpin.setIntakeSpeed(0.0);
    }   
}
