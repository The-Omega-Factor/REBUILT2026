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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    /* ============================= */
    /* Constructor                   */
    /* ============================= */

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /* ============================= */
    /* AutoBuilder Configuration     */
    /* ============================= */

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

    /* ============================= */
    /* Zero Functions (IMPORTANT)    */
    /* ============================= */

    /** 
     * Fully resets gyro AND pose to (0,0,0).
     * DO NOT use during auto.
     */
    public void zeroAll() {
        resetPose(new Pose2d());
        System.out.println("Gyro + Pose reset to 0,0,0");
    }

    /**
     * Safer mid-match reset.
     * Keeps X/Y but zeros heading.
     */
    public void zeroHeadingOnly() {
        Pose2d current = getPose();
        resetPose(new Pose2d(current.getTranslation(), new Rotation2d()));
        System.out.println("Heading reset, translation preserved");
    }

    /* ============================= */
    /* Basic Accessors               */
    /* ============================= */

    public Pose2d getPose() {
        if (Limelight.getTV(limelightName)) {
            return Limelight.getBotPose2d(Constants.limelightName);
        } else {
            Pose2d currentPose = getState().Pose;
            return new Pose2d(
            currentPose.getX(),
            currentPose.getY(),
            new Rotation2d(currentPose.getRotation().getRadians())
            );
        }
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

    /* ============================= */
    /* SysId                         */
    /* ============================= */

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /* ============================= */
    /* Periodic                      */
    /* ============================= */

    @Override
    public void periodic() {
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

        SmartDashboard.putNumber("X: ", this.getPose().getX());
        SmartDashboard.putNumber("Y", this.getPose().getY());
        SmartDashboard.putNumber("Theta", this.getPose().getRotation().getDegrees());
    }

    /* ============================= */
    /* Simulation                    */
    /* ============================= */

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

    /* ============================= */
    /* Vision                        */
    /* ============================= */

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
}