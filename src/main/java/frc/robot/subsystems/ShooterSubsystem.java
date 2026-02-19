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
    private final TalonFX shooterL = new TalonFX(0, canbus);
    private final TalonFX shooterR = new TalonFX(14, canbus);


    private final TalonFXConfiguration transferConfig = new TalonFXConfiguration();
    private final TalonFX transfer = new TalonFX(5, canbus);

    private final VelocityVoltage shooterRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage transferRequest = new VelocityVoltage(0).withSlot(1);

    public ShooterSubsystem() {
        shooterLConfig.Slot0.kP = 0.5;
        shooterLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterLConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(20))
        .withStatorCurrentLimitEnable(true));

        shooterL.getConfigurator().apply(shooterLConfig);
        shooterR.setControl(new Follower(shooterL.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterR.setNeutralMode(NeutralModeValue.Coast);

        transferConfig.Slot1.kP = 0.5;
        transferConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        transferConfig.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(20))
        .withStatorCurrentLimitEnable(true));

        transfer.getConfigurator().apply(transferConfig);
        transfer.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setShooterVelocity(double speed) {
        shooterL.setControl(shooterRequest.withVelocity(speed));
    }

    public TalonFX getShooterL() {
        return shooterL;
    }

    public void setTransferVelocity(double speed) {
        transfer.setControl(transferRequest.withVelocity(speed));
    }

    public TalonFX getTranfer() {
        return transfer;
    }
}
