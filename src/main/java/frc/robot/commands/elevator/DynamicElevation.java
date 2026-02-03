package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class DynamicElevation extends Command {
    private final ElevatorSubsystem elevator;
    private final double target;

    public DynamicElevation(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.target = Math.abs(elevator.getPosition() - ElevatorConstants.extendedPosition) < ElevatorConstants.errorTolerance ?
                        ElevatorConstants.retractedPosition : ElevatorConstants.extendedPosition;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPosition(target);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Math.abs(elevator.getPosition() - target)) < ElevatorConstants.errorTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.stop();
        } else {
            elevator.setPosition(elevator.getPosition());
        }
    }
}
