// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.SetOperatorForward;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    //TODO: set all commands to final (no creating a new command everytime)

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final double deadband = Constants.Controllers.deadband;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
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
        NamedCommands.registerCommand("Test Command", Commands.runOnce(()-> {System.out.println("Test success-------------------------------------------------------------------");}));

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Super simple auto", new ShootAndHopper(shooter, () -> {return 1 * Constants.Shooter.shooterSpeedMultiplier;} , () -> {return 0.0;}, false));
        SmartDashboard.putData(autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.applyDeadband(-joystick.getLeftY(), deadband) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.applyDeadband(-joystick.getLeftX(), deadband) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        // joystick.x().onTrue(Commands.runOnce(drivetrain::setRotationZero, drivetrain));
        joystick.x().onTrue(new SetOperatorForward(drivetrain));
        joystick.y().onTrue(Commands.runOnce(drivetrain::zeroAll));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

        //shooter
        shooter.setDefaultCommand(new ShootAndHopper(shooter, 
        () -> {
            if (notSwerveController.x().getAsBoolean()) return 1.2 * Constants.Shooter.shooterSpeedMultiplier;
            if (notSwerveController.a().getAsBoolean()) return 1 * Constants.Shooter.shooterSpeedMultiplier;
            if (notSwerveController.y().getAsBoolean()) return 2.0 * Constants.Shooter.shooterSpeedMultiplier;

            return 0.0;
        }, 
        () -> MathUtil.applyDeadband(notSwerveController.getLeftY(), deadband),
        false)
        );

        //intake
        
        intake.setDefaultCommand(new SetIntakeState(intake, 
        () -> {
            double right = MathUtil.applyDeadband(notSwerveController.getRightTriggerAxis(), deadband);
            double left = MathUtil.applyDeadband(notSwerveController.getLeftTriggerAxis(), deadband);

            return (right - left) * Constants.Intake.spinSpeedMultiplier;
        },
        () -> 
        MathUtil.applyDeadband(notSwerveController.getRightX(), deadband) * Constants.Intake.pivotSpeedMultiplier 
        + intake.getPivotPosition(),
        false)
        );

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
        SmartDashboard.putNumber("N-LeftTrigger", notSwerveController.getRightTriggerAxis());

        SmartDashboard.putBoolean("N-A", notSwerveController.a().getAsBoolean());
        SmartDashboard.putBoolean("N-X", notSwerveController.x().getAsBoolean());
        SmartDashboard.putBoolean("N-Y", notSwerveController.y().getAsBoolean());
    }
}
