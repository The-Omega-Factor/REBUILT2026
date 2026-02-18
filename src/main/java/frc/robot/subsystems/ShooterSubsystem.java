package frc.robot.subsystems;

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
    private final TalonFX shooterL = new TalonFX(0, "SwerveBase");
    private final TalonFX shooterR = new TalonFX(14, "SwerveBase");

    private final VelocityVoltage request = new VelocityVoltage(0);

    public ShooterSubsystem() {
        shooterLConfig.Slot0.kP = 0.5;
        shooterLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterL.getConfigurator().apply(shooterLConfig);
        //shooterR.setControl(new Follower(0, MotorAlignmentValue.Aligned));
        shooterR.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVelocity(double speed) {
        shooterL.setControl(request.withVelocity(speed));
    }

    public TalonFX getShooterL() {
        return shooterL;
    }
}
