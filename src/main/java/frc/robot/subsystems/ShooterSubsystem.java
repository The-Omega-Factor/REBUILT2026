package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        shooterLConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.shootersStatorAmpsLimit;
        shooterLConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterLConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shootersCurrentLimit;
        shooterLConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterL.getConfigurator().apply(shooterLConfig);
        shooterL.setNeutralMode(NeutralModeValue.Coast);

        shooterRConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.shootersStatorAmpsLimit;
        shooterRConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterRConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shootersCurrentLimit;
        shooterRConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterR.getConfigurator().apply(shooterRConfig);
        shooterR.setNeutralMode(NeutralModeValue.Coast);
        shooterR.setControl(new Follower(shooterL.getDeviceID(), MotorAlignmentValue.Opposed));

        hopperConfig.Slot0.kP = Constants.Shooter.HopperPIDs.kP;
        hopperConfig.Slot0.kS = Constants.Shooter.HopperPIDs.kS;
        hopperConfig.Slot0.kV = Constants.Shooter.HopperPIDs.kV;
        hopperConfig.Slot0.kD = Constants.Shooter.HopperPIDs.kD;

        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.hopperGearRatio;

        hopperConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.hopperStatorAmpsLimit;
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hopperConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.hopperCurrentLimit;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hopper.getConfigurator().apply(hopperConfig);
        hopper.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setShooterVelocity(double speed) {
        //rotation per second
        speed *= (3000/60);
        shooterL.setControl(shooterRequest.withVelocity(speed));
    }

    public void setHopperVelocity(double speed) {
        speed *= (3000/60);
        hopper.setControl(hopperRequest.withVelocity(speed));
    }

    public double getHopperVelocity() {
        return hopper.getVelocity().getValueAsDouble();
    }

    public double getShooterVelocity() {
        return shooterL.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", getShooterVelocity());
        SmartDashboard.putNumber("Hopper Speed", getHopperVelocity());
    }
}
