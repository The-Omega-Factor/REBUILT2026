package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
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

    private String shooterMode;

    public ShooterSubsystem() {
        shooterMode = "power";

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
        shooterLConfig.CurrentLimits.SupplyCurrentLimit = 45;
        shooterLConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterL.getConfigurator().apply(shooterLConfig);
        shooterL.setNeutralMode(NeutralModeValue.Coast);

        shooterRConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.shootersStatorAmpsLimit;
        shooterRConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterRConfig.CurrentLimits.SupplyCurrentLimit = 45;
        shooterRConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterR.getConfigurator().apply(shooterRConfig);
        shooterR.setNeutralMode(NeutralModeValue.Coast);
        shooterR.setControl(new Follower(shooterL.getDeviceID(), MotorAlignmentValue.Opposed));

        hopperConfig.Slot0.kP = Constants.Shooter.HopperPIDs.kP;
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.hopperGearRatio;

        hopperConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.hopperStatorAmpsLimit;
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hopperConfig.CurrentLimits.SupplyCurrentLimit = 45;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

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

    public double distanceBasedShooterSpeed(Pose2d robotPose, double targetX, double targetY) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        
        return Math.sqrt(Math.pow(robotX - targetX, 2) + Math.pow(robotY - targetY, 2)) 
        * Constants.Shooter.distanceBasedMultiplier;
    }

    public void setShooterMode(String newMode) {
        this.shooterMode = newMode;
    }

    public String getShooterMode() {
        return shooterMode;
    }
}
