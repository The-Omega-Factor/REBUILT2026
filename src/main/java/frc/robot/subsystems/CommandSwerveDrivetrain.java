package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    /* Simulation */
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier;
    private double m_lastSimTime;
    private final Field2d field = new Field2d();
    /* Alliance Perspective */
    private static final Rotation2d kBlueAlliancePerspective = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspective = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    /* SysId */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* Robot Relative Drive */
    private final SwerveRequest.ApplyRobotSpeeds m_applyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds();

    private final String limelightName = Constants.limelightName;

    /* NetworkTables */
    private final NetworkTable drivetrainTable;
    private final StructPublisher<Pose2d> posePublisher; 
    private final StructPublisher<ChassisSpeeds> chassisPublisher;
    private final DoubleArrayPublisher modulesAnglePublisher;
    private final DoubleArrayPublisher modulesSpeedPublisher;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
        SmartDashboard.putData("Field", field);

        /* Instantiate NetworkTables Variables */
        drivetrainTable = NetworkTableInstance.getDefault().getTable("drivetrain");
        posePublisher = drivetrainTable.getStructTopic("pose", Pose2d.struct).publish();
        chassisPublisher = drivetrainTable.getStructTopic("chassis", ChassisSpeeds.struct).publish();
        modulesAnglePublisher = drivetrainTable.getDoubleArrayTopic("modulesAngle").publish();
        modulesSpeedPublisher = drivetrainTable.getDoubleArrayTopic("modulesSpeed").publish();

        posePublisher.setDefault(new Pose2d());
        chassisPublisher.setDefault(new ChassisSpeeds());
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(
                        Constants.Swerve.AutoBuilderPIDs.Translational.kP,
                        Constants.Swerve.AutoBuilderPIDs.Translational.kI,
                        Constants.Swerve.AutoBuilderPIDs.Translational.kD
                    ),
                    new PIDConstants(
                        Constants.Swerve.AutoBuilderPIDs.Rotational.kP,
                        Constants.Swerve.AutoBuilderPIDs.Rotational.kI,
                        Constants.Swerve.AutoBuilderPIDs.Rotational.kD
                    )
                ),
                config,
                () -> DriverStation.getAlliance()
                        .map(alliance -> alliance == Alliance.Red)
                        .orElse(false),
                this
            );

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void zeroAll() {
        resetPose(new Pose2d());
        System.out.println("Gyro + Pose reset to 0,0,0");
    }

    public void zeroHeadingOnly() {
        Pose2d current = getPose();
        resetPose(new Pose2d(current.getTranslation(), new Rotation2d()));
        System.out.println("Heading reset, translation preserved");
    }

    public void updateVisionPose() {
        if (!Limelight.getTV(limelightName) || Limelight.getTA(limelightName) < 0.05) {
            return;
        }

        Limelight.SetRobotOrientation(
            limelightName,
            getPose().getRotation().getDegrees(),
            0,
            0,
            0,
            0,
            0
        );

        Limelight.PoseEstimate estimate =
            Limelight.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (estimate == null || estimate.tagCount == 0) {
            return;
        }

        if (Math.abs(getRobotRelativeSpeeds().omegaRadiansPerSecond) > 2.0) {
            return;
        }

        if (estimate.tagCount >= 2) {
            addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(0.5, 0.5, 9999999.0)
            );
        } else {
            addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(1.2, 1.2, 9999999.0)
            );
        }

        SmartDashboard.putNumber("LL Tag Count", estimate.tagCount);
        SmartDashboard.putNumber("LL Pose X", estimate.pose.getX());
        SmartDashboard.putNumber("LL Pose Y", estimate.pose.getY());
        SmartDashboard.putNumber("LL Pose Theta", estimate.pose.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public void resetPose(Pose2d newPose) {
        super.resetPose(newPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(m_applyRobotSpeeds.withSpeeds(speeds));
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        updateVisionPose();
        field.setRobotPose(getPose());
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspective
                        : kBlueAlliancePerspective
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("X", getPose().getX());
        SmartDashboard.putNumber("Y", getPose().getY());
        SmartDashboard.putNumber("Theta", getPose().getRotation().getDegrees());

        publishToNetworkTables();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        super.addVisionMeasurement(
            pose,
            Utils.fpgaToCurrentTime(timestampSeconds)
        );
    }

    @Override
    public void addVisionMeasurement(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {
        super.addVisionMeasurement(
            pose,
            Utils.fpgaToCurrentTime(timestampSeconds),
            stdDevs
        );
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(
            Utils.fpgaToCurrentTime(timestampSeconds)
        );
    }

    public void setRotationZero() {
        System.out.println("Set new field centric");
        Rotation2d currentPose = this.getState().Pose.getRotation();
        currentPose = currentPose.rotateBy(Rotation2d.fromDegrees(180));
        setOperatorPerspectiveForward(currentPose);
    }

    public void setCurrentPoseZero() {
        System.out.println("Setting current pose to be 0");
        this.getState().Pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }

    public void publishToNetworkTables() {
        posePublisher.set(getPose());
        chassisPublisher.set(getRobotRelativeSpeeds());

        SwerveModuleState[] moduleStates = getState().ModuleStates;

        double[] modulesAngle = new double[4];
        double[] modulesSpeed = new double[4];

        for (int i = 0; i < 4; i++) {
            modulesAngle[i] = moduleStates[i].angle.getRadians();
            modulesSpeed[i] = moduleStates[i].speedMetersPerSecond;
        }

        modulesAnglePublisher.set(modulesAngle);
        modulesSpeedPublisher.set(modulesSpeed);
    }
}