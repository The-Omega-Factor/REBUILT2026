package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSystem;

public class RunIntake extends Command {
    private final IntakeSystem intakeSystem;
    private final double speed;

    public RunIntake(IntakeSystem intakeSystem, double speed) {
        this.intakeSystem = intakeSystem;
        this.speed = speed;

        addRequirements(intakeSystem);
    }

    @Override
    public void execute() {
        intakeSystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSystem.setIntakeSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
