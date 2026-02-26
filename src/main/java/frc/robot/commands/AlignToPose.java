package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToPose extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;

    private final PIDController xController = new PIDController(3.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(3.0, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(4.0, 0.0, 0.0);

    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric();

    public AlignToPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        addRequirements(drivetrain);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(Math.toRadians(2));
    }

    @Override
    public void execute() {

        Pose2d currentPose = drivetrain.getPose();

        double rotationSpeed = thetaController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
        );

        drivetrain.setControl(
                driveRequest
                        .withRotationalRate(rotationSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        return thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
                driveRequest
                        .withRotationalRate(0)
        );
    }
}
