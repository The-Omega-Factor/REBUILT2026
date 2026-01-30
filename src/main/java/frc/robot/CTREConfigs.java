package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.DrivetrainConstants;

public final class CTREConfigs {

    public TalonFXConfiguration swerveSteerFXConfiguration = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfiguration = new TalonFXConfiguration();

    public CANcoderConfiguration swerveCANcoderConfiguration = new CANcoderConfiguration();

    public CTREConfigs() {

        swerveCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; //MK4I's use counterclockwise while MK4's use clockwise

        swerveSteerFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerveSteerFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        swerveSteerFXConfiguration.Feedback.SensorToMechanismRatio = DrivetrainConstants.steerMotorGearReduction;
        swerveSteerFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

        swerveSteerFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveSteerFXConfiguration.CurrentLimits.SupplyCurrentLimit = DrivetrainConstants.steerCurrentLimitAmps;
        
        swerveSteerFXConfiguration.Slot0.kP = 100.0;
        swerveSteerFXConfiguration.Slot0.kI = 0.0;
        swerveSteerFXConfiguration.Slot0.kD = 0.0;

        swerveDriveFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerveDriveFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        swerveDriveFXConfiguration.Feedback.SensorToMechanismRatio = DrivetrainConstants.driveMotorGearReduction;

        swerveDriveFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveDriveFXConfiguration.CurrentLimits.SupplyCurrentLimit = DrivetrainConstants.driveCurrentLimitAmps;

        swerveDriveFXConfiguration.Slot0.kP = 0.005;
        swerveDriveFXConfiguration.Slot0.kI = 0.0;
        swerveDriveFXConfiguration.Slot0.kD = 0.0;

        swerveDriveFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        swerveDriveFXConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        swerveDriveFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
        swerveDriveFXConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    }
}
