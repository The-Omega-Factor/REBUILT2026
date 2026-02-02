package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final VelocityVoltage velocityControllerRequest = new VelocityVoltage(0);

    private final TalonFXConfiguration shooterLConfig = new TalonFXConfiguration();
    private final TalonFX shooterL = new TalonFX(ShooterConstants.shooterLID);

    private final TalonFX shooterR = new TalonFX(ShooterConstants.shooterRID);

    public ShooterSubsystem() {
        shooterLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterR.setControl(new Follower(ShooterConstants.shooterLID, MotorAlignmentValue.Opposed));

        shooterLConfig.Feedback.SensorToMechanismRatio = ShooterConstants.shooterGearing;

        shooterLConfig.Slot0.kP = ShooterConstants.KP;
        shooterLConfig.Slot0.kI = ShooterConstants.KI;
        shooterLConfig.Slot0.kD = ShooterConstants.KD;

        shooterL.setNeutralMode(NeutralModeValue.Coast);

        shooterL.getConfigurator().apply(shooterLConfig);
    }

    public void setShooterVelocity(double velocity) {
        shooterL.setControl(velocityControllerRequest.withVelocity(velocity));
    }

    public void stopShooter() {
        shooterL.stopMotor();
    }

    public double getVelocity() {
        return shooterL.getVelocity().getValueAsDouble();
    }
}