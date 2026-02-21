package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFXConfiguration spinConfig;
    private final TalonFXConfiguration pivotConfig;

    private final TalonFX spin;
    private final TalonFX pivot;

    private final VelocityVoltage spinRequest;
    private final PositionVoltage pivotRequest;

    public IntakeSubsystem() {
        //Insantiating values
        spinConfig = new TalonFXConfiguration();
        pivotConfig = new TalonFXConfiguration();

        spin = new TalonFX(Intake.spinID, Constants.canbus);
        pivot = new TalonFX(Intake.pivotID, Constants.canbus);

        spinRequest = new VelocityVoltage(0).withSlot(0);
        pivotRequest = new PositionVoltage(getPivotPosition()).withSlot(0);

        //Intake Spin configurations
        spinConfig.Slot0.kP = Constants.Intake.SpinPIDs.kP;
        spinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spinConfig.Feedback.SensorToMechanismRatio = Constants.Intake.spinGearRatio;

        spinConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.Intake.spinAmpsLimit))
        .withStatorCurrentLimitEnable(true));

        spin.getConfigurator().apply(spinConfig);
        spin.setNeutralMode(NeutralModeValue.Coast);

        //Intake Pivot configurations
        pivotConfig.Slot0.kP = Constants.Intake.pivotPIDs.kP;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Intake.pivotGearRatio;

        pivotConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.Intake.pivotAmpsLimt))
        .withStatorCurrentLimitEnable(true));

        pivot.getConfigurator().apply(pivotConfig);
        pivot.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setIntakeSpinSpeed(double speed) {
        spin.setControl(spinRequest.withVelocity(speed));
    }

    public double getPivotPosition() {
        return pivot.getPosition().getValueAsDouble();
    }

    public void setPivotPosition(double position) {
        pivot.setControl(pivotRequest.withPosition(position));
    }
}
