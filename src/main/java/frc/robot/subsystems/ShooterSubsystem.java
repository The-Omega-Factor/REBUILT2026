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

    private final TalonFXConfiguration shooterLConfig;
    private final TalonFXConfiguration shooterRConfig;
    private final TalonFXConfiguration hopperConfig;

    private final TalonFX shooterL;
    private final TalonFX shooterR;
    private final TalonFX hopper;

    private final VelocityVoltage shooterRequest;
    private final VelocityVoltage hopperRequest;

    public ShooterSubsystem() {
        shooterLConfig = new TalonFXConfiguration();
        shooterRConfig = new TalonFXConfiguration();
        hopperConfig = new TalonFXConfiguration();

        shooterL = new TalonFX(Constants.Shooter.shooterLID, Constants.canbus);
        shooterR = new TalonFX(Constants.Shooter.shooterRID, Constants.canbus);
        hopper = new TalonFX(Constants.Shooter.hopperID, Constants.canbus);

        shooterRequest = new VelocityVoltage(0).withSlot(0);
        hopperRequest = new VelocityVoltage(0).withSlot(0);

        shooterLConfig.Slot0.kP = Constants.Shooter.ShooterPIDS.kP;
        shooterLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterLConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.shootersGearRatio;

        shooterLConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.Shooter.shootersAmpsLimit))
        .withStatorCurrentLimitEnable(true));

        shooterL.getConfigurator().apply(shooterLConfig);
        shooterL.setNeutralMode(NeutralModeValue.Coast);

        shooterRConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.Shooter.shootersAmpsLimit))
        .withStatorCurrentLimitEnable(true));

        shooterR.getConfigurator().apply(shooterRConfig);
        shooterR.setNeutralMode(NeutralModeValue.Coast);
        shooterR.setControl(new Follower(shooterL.getDeviceID(), MotorAlignmentValue.Opposed));

        hopperConfig.Slot0.kP = Constants.Shooter.HopperPIDs.kP;
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.hopperGearRatio;

        hopperConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(Constants.Shooter.hopperAmpsLimit))
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

    //TODO: Finished this method
    public double distanceBasedCalculator() {
        return 0.0;
    }
}
