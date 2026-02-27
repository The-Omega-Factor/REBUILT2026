package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToPoseV2 extends Command {
    public CommandSwerveDrivetrain drivetrain;
    public double targetX;
    public double targetY;

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

    private double error;

    public AlignToPoseV2(CommandSwerveDrivetrain drivetrain, double targetX, double targetY) {
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        thetaController.reset();
        thetaController.setTolerance(Math.toRadians(Constants.Swerve.angularTolerance));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getPose();

        currentX = currentPose.getX();
        currentY = currentPose.getY();
        currentHeading = MathUtil.angleModulus(currentPose.getRotation().getRadians());

        targetHeading = Math.atan2(targetY - currentY, targetX - currentX);
        error = MathUtil.clamp(
            thetaController.calculate(currentHeading, targetHeading),
            -Constants.Swerve.maxOmega,
            Constants.Swerve.maxOmega
            );

        drivetrain.setControl(driveRequest.withRotationalRate(error));
    }

    @Override
    public boolean isFinished() {
        return thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withRotationalRate(0));
    }

    public void setTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }
}