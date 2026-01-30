package frc.robot.subsystems.drive;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule extends SubsystemBase {

    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX steerMotor;
    private TalonFX driveMotor;
    private CANcoder steerEncoder;

    private final SimpleMotorFeedforward driveFeedForward = DrivetrainConstants.driveFF;

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final PositionVoltage steerPosition = new PositionVoltage(0);

    public SwerveModule(int driveMotorID, int steerMotorID, int canCoderID, Rotation2d offset, int moduleNumber) {

        this.moduleNumber = moduleNumber;
        this.angleOffset = offset;

        steerEncoder = new CANcoder(canCoderID, CANDevices.CANivoreName);
        steerEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfiguration);

        steerMotor = new TalonFX(steerMotorID, CANDevices.CANivoreName);
        steerMotor.getConfigurator().apply(Robot.ctreConfigs.swerveSteerFXConfiguration);
        
        driveMotor = new TalonFX(driveMotorID, CANDevices.CANivoreName);
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfiguration);
        driveMotor.getConfigurator().setPosition(0);

        Timer.delay(0.5);

        resetToAbsolute(); //1 second delay if needed

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        //desiredState = SwerveModuleState.optimize(desiredState, getState().angle); // Depricated line due to update
        desiredState.optimize(getState().angle);
        steerMotor.setControl(steerPosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);

    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (Math.abs(desiredState.speedMetersPerSecond) < 1e-3) {
            // keep current angle — do not attempt to rotate the wheel when speed is zero
            desiredState = new SwerveModuleState(0.0, getState().angle);
        }

        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / DrivetrainConstants.maxDriveSpeedMetersPerSec;
            driveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, DrivetrainConstants.wheelCircumferenceMeters);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANCoder() {

        return Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValueAsDouble());

    }

    public void resetToAbsolute() {

        double absolutePosition = getCANCoder().getRotations() - angleOffset.getRotations();
        steerMotor.setPosition(absolutePosition);

    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), DrivetrainConstants.wheelCircumferenceMeters),
            Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble())
        );

    }

    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), DrivetrainConstants.wheelCircumferenceMeters),
            Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble())
        );

    }
    
}
