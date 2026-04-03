package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.SpinPIDs;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFXConfiguration spinConfig;
    private final TalonFXConfiguration pivotConfig;
    private final TalonFXConfiguration pivotLConfig;

    private final TalonFX spin;
    private final TalonFX pivot;
    private final TalonFX pivotL;

    private final VelocityVoltage spinRequest;
    private final PositionVoltage pivotRequest;

    private final DoublePublisher intakePositionPublisher;
    private final DoublePublisher intakeSpeedPublisher;

    public IntakeSubsystem() {
        spinConfig = new TalonFXConfiguration();
        pivotConfig = new TalonFXConfiguration();
        pivotLConfig = new TalonFXConfiguration();

        spin = new TalonFX(Intake.spinID, Constants.canbus2);
        pivot = new TalonFX(Intake.pivotRID, Constants.canbus2);
        pivotL = new TalonFX(Intake.pivotLID, Constants.canbus2);

        spinRequest = new VelocityVoltage(0).withSlot(0);
        pivotRequest = new PositionVoltage(0).withSlot(0);

        // Intake spin config
        spinConfig.Slot0.kP = Constants.Intake.SpinPIDs.kP;
        spinConfig.Slot0.kS = SpinPIDs.kS;
        spinConfig.Slot0.kV = SpinPIDs.kV;
        spinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spinConfig.Feedback.SensorToMechanismRatio = Constants.Intake.spinGearRatio;

        spinConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.spinStatorAmpsLimit;
        spinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spinConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.spinCurrentLimit;
        spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        spin.getConfigurator().apply(spinConfig);
        spin.setNeutralMode(NeutralModeValue.Coast);

        // Pivot master config
        pivotConfig.Slot0.kP = Constants.Intake.pivotPIDs.kP;
        pivotConfig.Slot0.kD = Constants.Intake.pivotPIDs.kD;
        pivotConfig.Slot0.kG = Constants.Intake.pivotPIDs.kG;

        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Intake.pivotGearRatio;

        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.pivotStatorAmpsLimit;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.pivotCurrentLimit;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.pivotUpperLimit;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.pivotLowerLimit;
        // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        pivot.getConfigurator().apply(pivotConfig);
        pivot.setNeutralMode(NeutralModeValue.Brake);

        // Pivot follower config
        pivotLConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.pivotStatorAmpsLimit;
        pivotLConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.pivotCurrentLimit;
        pivotLConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotLConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotLConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.pivotUpperLimit;
        pivotLConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.pivotLowerLimit;
        // pivotLConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // pivotLConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        pivotL.getConfigurator().apply(pivotLConfig);
        pivotL.setNeutralMode(NeutralModeValue.Brake);

        // Second pivot motor follows the first
        pivotL.setControl(new Follower(pivot.getDeviceID(), MotorAlignmentValue.Opposed));

        intakePositionPublisher = NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("Intake Position").publish();
        intakeSpeedPublisher = NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("Intake Speed").publish();
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

    public double getSpinSpeed() {
        return spin.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        intakePositionPublisher.set(getPivotPosition());
        intakeSpeedPublisher.set(getSpinSpeed());
    }
}