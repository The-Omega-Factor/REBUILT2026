package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    private final TalonFX elevator = new TalonFX(ElevatorConstants.elevatorID);

    public ElevatorSubsystem() {
        elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.elevatorGearing;

        elevatorConfig.Slot0.kP = ElevatorConstants.KP;
        elevatorConfig.Slot0.kI = ElevatorConstants.KI;
        elevatorConfig.Slot0.kD = ElevatorConstants.KD;

        elevator.setNeutralMode(NeutralModeValue.Brake);
        elevator.setPosition(ElevatorConstants.retractedPosition);
        elevator.getConfigurator().apply(elevatorConfig);
    }

    public void setPosition(double position) {
        elevator.setControl(positionRequest.withPosition(position));
    }

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public void stop() {
        elevator.stopMotor();
    }
}
