// Copyright (c) FIRST and other WPILib contributors.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ShootAndHopper;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.SetOperatorForward;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShooterSpeedCalculators.ShooterMode;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double deadband = Constants.Controllers.deadband;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController joystick = new CommandXboxController(Constants.Controllers.swerveID);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CommandXboxController notSwerveController = new CommandXboxController(Constants.Controllers.notSwerveID);

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    public RobotContainer() {
        NamedCommands.registerCommand("B100Intake", new ParallelCommandGroup(
            new ShootAndHopper(shooter, () -> shooter.getShooterVelocity(), () -> {return 0.0;}, true),
            new SetIntakeState(intake, () -> {return 0.0;}, () -> {return intake.getPivotPosition();}, true))
        );
        NamedCommands.registerCommand("B101Shoot", new ShootAndHopper(shooter, () -> {return 0.0;}, () -> {return 0.0;}, true));
        NamedCommands.registerCommand("B102RaiseIntake", new ShootAndHopper(shooter, () -> {return 0.0;}, () -> {return 0.0;}, true));
        NamedCommands.registerCommand("B103Shoot", new ShootAndHopper(shooter, () -> {return 0.0;}, () -> {return 0.0;}, true));
        NamedCommands.registerCommand("Simple Shoot", new ShootAndHopper(shooter, () -> {return 1.32 * Constants.Shooter.shooterSpeedMultiplier;} , () -> {return -0.5;}, false).withTimeout(10));
        NamedCommands.registerCommand("Very Simple Shoot", new ShootAndHopper(shooter, () -> {return 1.15  * Constants.Shooter.shooterSpeedMultiplier;} , () -> {return -0.67;}, false));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser); // ✅ FIXED

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.applyDeadband(-joystick.getLeftY(), deadband) * MaxSpeed)
                    .withVelocityY(MathUtil.applyDeadband(-joystick.getLeftX(), deadband) * MaxSpeed)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate * 0.9)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.x().onTrue(new SetOperatorForward(drivetrain));
        joystick.y().onTrue(Commands.runOnce(drivetrain::zeroAll));
        joystick.leftBumper().onTrue(new AlignToPose(drivetrain));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

        shooter.setDefaultCommand(new ShootAndHopper(shooter, 
        () -> {
            if (notSwerveController.x().getAsBoolean()) return 1.36 * Constants.Shooter.shooterSpeedMultiplier;
            if (notSwerveController.a().getAsBoolean()) return 1 * Constants.Shooter.shooterSpeedMultiplier;
            if (notSwerveController.y().getAsBoolean()) return 2.0 * Constants.Shooter.shooterSpeedMultiplier;
            if (notSwerveController.b().getAsBoolean()) return -1 * Constants.Shooter.shooterSpeedMultiplier;
            return 0.0;
        }, 
        () -> MathUtil.applyDeadband(notSwerveController.getLeftY(), deadband),
        false)
        );

        notSwerveController.leftBumper().onTrue(new SetShooterSpeed(shooter, drivetrain.getPose(), ShooterMode.LENGTH));
        notSwerveController.rightBumper().onTrue(new SetShooterSpeed(shooter, drivetrain.getPose(), ShooterMode.LENGTH));

        intake.setDefaultCommand(new SetIntakeState(intake, () -> { 
            double right = MathUtil.applyDeadband(notSwerveController.getRightTriggerAxis(), deadband); 
            double left = MathUtil.applyDeadband(notSwerveController.getLeftTriggerAxis(), deadband); 
            return (right - left) * Constants.Intake.spinSpeedMultiplier; }, () -> 
            MathUtil.applyDeadband(notSwerveController.getRightX(), deadband)
            * Constants.Intake.pivotSpeedMultiplier + intake.getPivotPosition(), false));

        addGamepadsTelemetry();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void addGamepadsTelemetry() {
        SmartDashboard.putNumber("S-LeftY", joystick.getLeftY());
        SmartDashboard.putNumber("S-LeftX", joystick.getLeftX());
        SmartDashboard.putNumber("S-RightX", joystick.getRightX());

        SmartDashboard.putBoolean("S-A", joystick.a().getAsBoolean());
        SmartDashboard.putBoolean("S-B", joystick.b().getAsBoolean());
        SmartDashboard.putBoolean("S-X", joystick.x().getAsBoolean());
        SmartDashboard.putBoolean("S-Y", joystick.y().getAsBoolean());

        SmartDashboard.putNumber("N-LeftY", notSwerveController.getLeftY());
        SmartDashboard.putNumber("N-RightX", notSwerveController.getRightX());
        SmartDashboard.putNumber("N-LeftTrigger", notSwerveController.getLeftTriggerAxis());
        SmartDashboard.putNumber("N-RightTrigger", notSwerveController.getRightTriggerAxis()); // ✅ FIXED

        SmartDashboard.putBoolean("N-A", notSwerveController.a().getAsBoolean());
        SmartDashboard.putBoolean("N-X", notSwerveController.x().getAsBoolean());
        SmartDashboard.putBoolean("N-Y", notSwerveController.y().getAsBoolean());
    }
}