package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RetractElevator extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private final double targetPosition = ElevatorConstants.retractedPosition;

    public RetractElevator(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSubsystem.getPosition() - targetPosition) < ElevatorConstants.errorTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevatorSubsystem.stop();
        } else {
            elevatorSubsystem.setPosition(elevatorSubsystem.getPosition());
        }
    }
}
