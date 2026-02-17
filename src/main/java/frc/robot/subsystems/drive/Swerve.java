package frc.robot.subsystems.drive;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DrivetrainConstants;

public class Swerve extends SubsystemBase {

    public RobotContainer robotContainer;

    private Field2d field = new Field2d();

    private CANBus canbus = new CANBus(CANDevices.CANivoreName);

    public Pigeon2 gyro = new Pigeon2(CANDevices.PigeonID, canbus);

    private final SwerveModule frontLeftMod = 
        new SwerveModule(
            CANDevices.frontLeftDriveMotorID, 
            CANDevices.frontLeftSteerMotorID, 
            CANDevices.frontLeftCANCoderID, 
            DrivetrainConstants.frontLeftModOffset,
            DrivetrainConstants.frontLeftModuleNumber
        );
    
        private final SwerveModule frontRightMod = 
        new SwerveModule(
            CANDevices.frontRightDriveMotorID, 
            CANDevices.frontRightSteerMotorID, 
            CANDevices.frontRightCANCoderID, 
            DrivetrainConstants.frontRightModOffset,
            DrivetrainConstants.frontRightModuleNumber
        );
    
        private final SwerveModule backLeftMod = 
        new SwerveModule(
            CANDevices.backLeftDriveMotorID, 
            CANDevices.backLeftSteerMotorID, 
            CANDevices.backLeftCANCoderID, 
            DrivetrainConstants.backLeftModOffset,
            DrivetrainConstants.backLeftModuleNumber
        );

        private final SwerveModule backRightMod = 
        new SwerveModule(
            CANDevices.backRightDriveMotorID, 
            CANDevices.backRightSteerMotorID, 
            CANDevices.backRightCANCoderID, 
            DrivetrainConstants.backRightModOffset,
            DrivetrainConstants.backRightModuleNumber
        );

    public boolean isLocked = false;

    public boolean isLocked() {

        return isLocked;

    }

    private boolean isFieldOriented = true;

    public boolean isFieldOriented() {

        return isFieldOriented;

    }

    private SwerveDrivePoseEstimator odometry = 
        new SwerveDrivePoseEstimator(
            DrivetrainConstants.kinematics, 
            getHeading(), 
            getModulePositions(), 
            new Pose2d()
        );
    
    public Swerve() {
        resetPose();
    }

    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getModulePositions()); //TODO: Set limelight name

        LimelightHelpers.SetRobotOrientation("limelight-right", getHeading().getDegrees() - 232, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

        //swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999));
        //swerveDrivePoseEstimator.addVisionMeasurement(pose.pose, pose.timestampSeconds);

        SmartDashboard.putData("Field" , field);

        if (LimelightHelpers.getTV("limelight-right")) {
            field.setRobotPose(pose.pose);
        } else {
            field.setRobotPose(odometry.getEstimatedPosition());
        }
    }

    private double speedFactor = 1.0; //TODO: Tune this

    public double getSpeedFactor() {

        return speedFactor;

    }

    public void setSpeedFactor(double speedFactor) {

        this.speedFactor = speedFactor;

    }

    public void drive(double driveX, double driveY, double rotation, boolean isFieldOriented) {

        if (driveX != 0.0 || driveY != 0.0 || rotation != 0.0) isLocked = false;

        if ( isLocked ) {
            setModuleStatesOpenLoop(new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI))
            });
        } else {
            driveX *= speedFactor;
            driveY *= speedFactor;
            rotation *= speedFactor;

            ChassisSpeeds speeds =
                isFieldOriented
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveX, driveY, rotation, getHeading())
                    : new ChassisSpeeds(driveX, driveY, rotation);

            SwerveModuleState[] states = DrivetrainConstants.kinematics.toSwerveModuleStates(speeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxDriveSpeedMetersPerSec);

            setModuleStatesOpenLoop(states);
        }

    }

    public void stop() {

        drive(0, 0,0, isFieldOriented);

    }

    public void lock() {

        isLocked = true;

    }

    //TODO: Small change in boolean that was wrong

    public void setModuleStatesClosedLoop(SwerveModuleState[] moduleStates) {

        frontLeftMod.setDesiredState(moduleStates[0], false);
        frontRightMod.setDesiredState(moduleStates[1], false);
        backLeftMod.setDesiredState(moduleStates[2], false);
        backRightMod.setDesiredState(moduleStates[3], false);

    }

    public void setModuleStatesOpenLoop(SwerveModuleState[] moduleStates) {

        frontLeftMod.setDesiredState(moduleStates[0], true);
        frontRightMod.setDesiredState(moduleStates[1], true);
        backLeftMod.setDesiredState(moduleStates[2], true);
        backRightMod.setDesiredState(moduleStates[3], true);

    }

    public ChassisSpeeds getChassisSpeeds() {

        /*
        double xVel = getAverageDriveVelocityMetersPerSec() * getDirectionOfTravel().getCos();
        double yVel = getAverageDriveVelocityMetersPerSec() * getDirectionOfTravel().getSin();
        double omega = Units.degreesToRadians(gyro.getAngularVelocityZWorld().getValueAsDouble());

        return new ChassisSpeeds(xVel, yVel, omega);
        */
        //TODO: Major change in getChassisSpeeds()

        return DrivetrainConstants.kinematics.toChassisSpeeds(getModuleStates());

    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {

        setModuleStatesClosedLoop(DrivetrainConstants.kinematics.toSwerveModuleStates(chassisSpeeds));

    }

    public SwerveModuleState[] getModuleStates() {

        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeftMod.getState().speedMetersPerSecond, frontLeftMod.getState().angle),
            new SwerveModuleState(frontRightMod.getState().speedMetersPerSecond, frontRightMod.getState().angle),
            new SwerveModuleState(backLeftMod.getState().speedMetersPerSecond, backLeftMod.getState().angle),
            new SwerveModuleState(backRightMod.getState().speedMetersPerSecond, backRightMod.getState().angle),
        };

    }

    public SwerveModulePosition[] getModulePositions() {

        return new SwerveModulePosition[] {
            frontLeftMod.getPosition(),
            frontRightMod.getPosition(),
            backLeftMod.getPosition(),
            backRightMod.getPosition()
        };

    }

    public double getAverageDriveVelocityMetersPerSec() {

        return(
            (Math.abs(frontLeftMod.getState().speedMetersPerSecond)
            + Math.abs(frontRightMod.getState().speedMetersPerSecond)
            + Math.abs(backLeftMod.getState().speedMetersPerSecond )
            + Math.abs(backRightMod.getState().speedMetersPerSecond))
            / 4.0
        );

    }

    public Rotation2d getDirectionOfTravel() {

        return new Rotation2d(
            (frontLeftMod.getState().angle.plus(new Rotation2d(frontLeftMod.getState().speedMetersPerSecond < 0.0 ? Math.PI : 0.0)).getRadians()
            + frontRightMod.getState().angle.plus(new Rotation2d(frontRightMod.getState().speedMetersPerSecond < 0.0 ? Math.PI : 0.0)).getRadians()
            + backLeftMod.getState().angle.plus(new Rotation2d(backLeftMod.getState().speedMetersPerSecond < 0.0 ? Math.PI : 0.0)).getRadians()
            + backRightMod.getState().angle.plus(new Rotation2d(backRightMod.getState().speedMetersPerSecond < 0.0 ? Math.PI : 0.0)).getRadians()
            ) / 4.0
        );

    }

    public Pose2d getPose() {

        return odometry.getEstimatedPosition();

    }

    public void resetPose() {

        setPose(new Pose2d());

    }

    public void setPose(Pose2d pose) {

        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);

    }

    public void setTranslation(Translation2d translation) {

        odometry.resetPosition(getHeading(), getModulePositions(), new Pose2d(translation, getHeading()));

    }

    public Rotation2d getHeading() {

        return Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(gyro.getYaw().getValueAsDouble())));

    }

    public Rotation2d getPitch() {

        return Rotation2d.fromDegrees(gyro.getPitch().getValueAsDouble());

    }

    public Rotation2d getRoll() {

        return Rotation2d.fromDegrees(gyro.getRoll().getValueAsDouble());

    }

    public void resetHeading() {

        gyro.setYaw(0.0);

    }

    public void setHeading(Rotation2d heading) {

        odometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));

    }

    public void zeroHeading() {

        odometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));

    }

    public Rotation2d getGyroYaw() {

        return getHeading();

    }

    public Command followPathCommand(String pathName) {

        try {

            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return new FollowPathCommand(
                path, 
                this::getPose, 
                this::getChassisSpeeds, 
                this::setChassisSpeeds, 
                new PPHolonomicDriveController(
                    new PIDConstants(AutoConstants.drivekP, AutoConstants.drivekD), 
                    new PIDConstants(AutoConstants.rotkP, AutoConstants.rotkD)), 
                RobotConfig.fromGUISettings(), 
                () -> {var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e) {

            return Commands.none();
            
        }

    }

    public Command findPathCommand(double x, double y, double rotation) {

        try {

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, Rotation2d.fromDegrees(rotation))
            );

            PathConstraints constraints = new PathConstraints(
                3.0,
                3.0,
                2 * Math.PI,
                4 * Math.PI
            );

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, Rotation2d.fromDegrees(rotation))
            );

            return new FollowPathCommand(
                path, 
                this::getPose, 
                this::getChassisSpeeds, 
                this::setChassisSpeeds, 
                new PPHolonomicDriveController(
                    new PIDConstants(AutoConstants.drivekP, AutoConstants.drivekD), 
                    new PIDConstants(AutoConstants.rotkP, AutoConstants.rotkD)), 
                RobotConfig.fromGUISettings(), 
                () -> {var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e) {

            return Commands.none();
            
        }

    }

    /*public Command findPathCommand(String pathName) {

        PathConstraints constraints;
        HolonomicDriveController driveController;
        Supplier<Pose2d> robotPoseSupplier;
        Consumer<ChassisSpeeds> robotRelativeSpeeds;
        Subsystem driveSubsystem;
        Pose2d targetPose;
        
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        driveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(5)));
        final Command pathFindToTargetRough = AutoBuilder.pathfindToPose(targetPose, constraints, 0.5),
            preciseAlignment = new FunctionalCommand(
                () -> {}, 
                () -> robotRelativeSpeeds.accept(driveController.calculate(
                    robotPoseSupplier.get(), 
                    targetPose, 
                    0,
                    targetPose.getRotation())), 
                (interrupted) -> robotRelativeSpeeds.accept(new ChassisSpeeds()), 
                driveController::atReference);


    }*/

}
