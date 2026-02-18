package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFXConfiguration shooterLConfig = new TalonFXConfiguration();
    private final CANBus canbus = new CANBus("SwerveBase");
    private final TalonFX shooterL = new TalonFX(0, canbus);
    private final TalonFX shooterR = new TalonFX(14, canbus);

    private final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    public ShooterSubsystem() {
        shooterLConfig.Slot0.kP = 0.5;
        shooterLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterL.getConfigurator().apply(shooterLConfig);
        shooterR.setControl(new Follower(shooterL.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterL.setNeutralMode(NeutralModeValue.Coast);
        shooterR.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVelocity(double speed) {
        shooterL.setControl(request.withVelocity(speed));
    }

    public TalonFX getShooterL() {
        return shooterL;
    }
}
