package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
    private final PositionVoltage intakePivotPositionRequest = new PositionVoltage(0);
    private final TalonFXConfiguration intakePivotConfiguration = new TalonFXConfiguration();
    private final TalonFX intakePivot = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID);

    private final TalonFXConfiguration intakeSpinConfiguration = new TalonFXConfiguration();
    private final TalonFX intakeSpin = new TalonFX(IntakeConstants.INTAKE_SPIN_ID);

    public IntakeSystem() {
        //TODO: Look into Motion Magic x44

        /*
         * intakePivot (The arm that moves the intake up and down)
         * 
         * Neutral Mode is BRAKE
         * A gear ratio of 1.0 is used (no gear ratio set)
         * Starting position must be when the intake is engaged (intake is lowered) 
         * 
         * Set Intake Pivot direction
         * Tune KP and KD
         */

        intakePivotConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakePivotConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEARING;

        intakePivotConfiguration.Slot0.kP = IntakeConstants.KP;
        intakePivotConfiguration.Slot0.kI = IntakeConstants.KI;
        intakePivotConfiguration.Slot0.kD = IntakeConstants.KD;

        intakePivot.getConfigurator().apply(intakePivotConfiguration);
        intakePivot.setNeutralMode(NeutralModeValue.Brake);
        intakePivot.setPosition(IntakeConstants.PIVOT_ENGAGED_POSITION);

        /*
         * intakeSpin (The rolling part of the intake)
         *
         * Neutral mode is COAST
         * 
         * Set intake direction
         */

        intakeSpinConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeSpin.getConfigurator().apply(intakeSpinConfiguration);
        intakeSpin.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Intake pivot moves to position using an internal, hardware PID loop.
     * The PID values can be tuned in the IntakeConstants class (in Constants.java file)
     * Benefits greatly from accurate gear ratios
     * 
     * @param position measured in rotations
     */

    public void setIntakePivotPosition(double position) {
        intakePivot.setControl(intakePivotPositionRequest.withPosition(position));
    }

    public void holdIntakePivot() {
        intakePivot.setControl(intakePivotPositionRequest.withPosition(intakePivot.getPosition().getValueAsDouble()));
    }
    
    /**
     * Force stop the intake pivot entirely
     * The motor enter neutral momde and brake
     */

    public void stopIntakePivot() {
        intakePivot.stopMotor();
    }

    public void setIntakeSpeed(double speed) {
        intakeSpin.set(speed);
    }

    public double getPivotPosition() {
        return intakePivot.getPosition().getValueAsDouble();
    }
}
