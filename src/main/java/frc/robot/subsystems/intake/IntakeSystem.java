package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
    private final PositionVoltage intakePivotPositionRequest = new PositionVoltage(0);
    private final TalonFXConfiguration intakePivotConfiguration = new TalonFXConfiguration();
    private final TalonFX intakePivot = new TalonFX(IntakeConstants.intakePivotID);

    private final TalonFXConfiguration intakeSpinConfiguration = new TalonFXConfiguration();
    private final TalonFX intakeSpin = new TalonFX(IntakeConstants.intakeSpinID);

    public IntakeSystem() {
        intakePivotConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakePivotConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.pivotGearing;

        intakePivotConfiguration.Slot0.kP = IntakeConstants.kP;
        intakePivotConfiguration.Slot0.kI = IntakeConstants.kI;
        intakePivotConfiguration.Slot0.kD = IntakeConstants.kD;

        intakePivot.getConfigurator().apply(intakePivotConfiguration);
        intakePivot.setNeutralMode(NeutralModeValue.Brake);
        intakePivot.setPosition(0.0);

        intakeSpinConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeSpin.getConfigurator().apply(intakeSpinConfiguration);
        intakeSpin.setNeutralMode(NeutralModeValue.Brake);

        setDefaultCommand(new RunCommand(
            () -> {
                setIntakeSpeed(0);
                setIntakePivotPosition(getPivotPosition());
            }, 
            this));
    }

    public void setIntakePivotPosition(double position) {
        intakePivot.setControl(intakePivotPositionRequest.withPosition(position));
    }

    public void setIntakeSpeed(double speed) {
        intakeSpin.set(speed);
    }

    public double getPivotPosition() {
        return intakePivot.getPosition().getValueAsDouble();
    }
}
