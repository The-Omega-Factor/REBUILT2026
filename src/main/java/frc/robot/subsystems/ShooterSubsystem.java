package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final CANBus canbus = new CANBus("SwerveBase");

    private final TalonFXConfiguration shooterLConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration shooterRConfig = new TalonFXConfiguration();
    private final TalonFX shooterL = new TalonFX(0, canbus);
    private final TalonFX shooterR = new TalonFX(14, canbus);


    private final TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
    private final TalonFX hopper = new TalonFX(5, canbus);

    private final VelocityVoltage shooterRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage hopperRequest = new VelocityVoltage(0).withSlot(0);

    public ShooterSubsystem() {
        shooterLConfig.Slot0.kP = 0.5;
        shooterLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterLConfig.Feedback.SensorToMechanismRatio = 25/20;

        shooterLConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(67))
        .withStatorCurrentLimitEnable(true));

        shooterL.getConfigurator().apply(shooterLConfig);
        shooterL.setNeutralMode(NeutralModeValue.Coast);

        shooterRConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(67))
        .withStatorCurrentLimitEnable(true));

        shooterR.getConfigurator().apply(shooterRConfig);
        shooterR.setNeutralMode(NeutralModeValue.Coast);
        shooterR.setControl(new Follower(shooterL.getDeviceID(), MotorAlignmentValue.Opposed));

        hopperConfig.Slot0.kP = 0.5;
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperConfig.Feedback.SensorToMechanismRatio = 20/35;

        hopperConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(67))
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
