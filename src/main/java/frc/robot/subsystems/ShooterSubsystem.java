package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFXConfiguration shooterLConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration shooterRConfig = new TalonFXConfiguration();
    private final TalonFX shooterL = new TalonFX(Constants.ShooterConstants.shooterLID, Constants.canbus);
    private final TalonFX shooterR = new TalonFX(Constants.ShooterConstants.shooterRID, Constants.canbus);

    private final TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
    private final TalonFX hopper = new TalonFX(5, Constants.canbus);

    private final VelocityVoltage shooterRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage hopperRequest = new VelocityVoltage(0).withSlot(0);

    public ShooterSubsystem() {
        shooterLConfig.Slot0.kP = Constants.ShooterConstants.shooterPIDs.kP;
        shooterLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterLConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.shootersGearRatio;

        shooterLConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.ShooterConstants.shootersAmpsLimit))
        .withStatorCurrentLimitEnable(true));

        shooterL.getConfigurator().apply(shooterLConfig);
        shooterL.setNeutralMode(NeutralModeValue.Coast);

        shooterRConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.ShooterConstants.shootersAmpsLimit))
        .withStatorCurrentLimitEnable(true));

        shooterR.getConfigurator().apply(shooterRConfig);
        shooterR.setNeutralMode(NeutralModeValue.Coast);
        shooterR.setControl(new Follower(shooterL.getDeviceID(), MotorAlignmentValue.Opposed));

        hopperConfig.Slot0.kP = Constants.ShooterConstants.hopperPIDs.kP;
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.hopperGearRatio;

        hopperConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.ShooterConstants.hopperAmpsLimit))
        .withStatorCurrentLimitEnable(true));

        hopper.getConfigurator().apply(hopperConfig);
        hopper.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setShooterVelocity(double speed) {
        //rotation per second
        speed *= (6000/60);
        shooterL.setControl(shooterRequest.withVelocity(speed));
    }

    public void setHopperVelocity(double speed) {
        speed *= (6000/60);
        hopper.setControl(hopperRequest.withVelocity(speed));
    }
}
