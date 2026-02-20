package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFXConfiguration spinConfig = new TalonFXConfiguration();
    private final TalonFX intakeSpin = new TalonFX(IntakeConstants.spinID, Constants.canbus);

    private final VelocityVoltage spinRequest = new VelocityVoltage(0).withSlot(0);

    public IntakeSubsystem() {
        spinConfig.Slot0.kP = Constants.IntakeConstants.SpinPIDs.kP;
        spinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spinConfig.Feedback.SensorToMechanismRatio = 10/20;

        spinConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.IntakeConstants.spinAmpsLimit))
        .withStatorCurrentLimitEnable(true));

        intakeSpin.getConfigurator().apply(spinConfig);
        intakeSpin.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setIntakeSpinSpeed(double speed) {
        intakeSpin.setControl(spinRequest.withVelocity(speed));
    }
}
