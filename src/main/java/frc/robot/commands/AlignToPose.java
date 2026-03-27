package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Field;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static frc.robot.utils.Alliance.getAlliance;

public class AlignToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double targetX;
    private final double targetY;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
    private final PIDController thetaController = new PIDController(
        Constants.Swerve.AutoBuilderPIDs.Rotational.kP, 
        Constants.Swerve.AutoBuilderPIDs.Rotational.kI, 
        Constants.Swerve.AutoBuilderPIDs.Rotational.kD
        );
    private double targetHeading;

    private Pose2d currentPose;
    private double currentX;
    private double currentY;
    private double currentHeading;

    public AlignToPose(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.targetX = getAlliance() == DriverStation.Alliance.Red ? Field.redX : Field.blueX;
        this.targetY = Field.y;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        thetaController.reset();
        thetaController.setTolerance(Math.toRadians(Constants.Swerve.angularTolerance));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //thetaController.setIntegratorRange(-Constants.Swerve.minOmega, Constants.Swerve.maxOmega);
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getPose();

        currentX = currentPose.getX();
        currentY = currentPose.getY();
        currentHeading = MathUtil.angleModulus(currentPose.getRotation().getRadians());

        targetHeading = Math.atan2(targetY - currentY, targetX - currentX);

        double pidOutput = thetaController.calculate(currentHeading, targetHeading);

        // Clamp to max rotational speed
        pidOutput = MathUtil.clamp(
            pidOutput,
            -Constants.Swerve.maxOmega,
            Constants.Swerve.maxOmega
        );

        // Apply minimum threshold to prevent oscillation
        if (Math.abs(pidOutput) < Constants.Swerve.minOmega) {
            pidOutput = 0;
        }

        drivetrain.setControl(
            driveRequest.withRotationalRate(pidOutput)
        );
    }

    @Override
    public boolean isFinished() {
        return thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withRotationalRate(0));
    }   
}