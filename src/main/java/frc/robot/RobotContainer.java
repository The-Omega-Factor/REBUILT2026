package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.math.ShootingSpeedCalculators;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.elevator.DynamicElevation;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.RunIndexer;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAndIndexer;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  private String autoName;
  private String teamColor;

  private final IntakeSystem intakeSystem = new IntakeSystem();
  private final Swerve swerveDrive = new Swerve();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
  private final XboxController xboxController = new XboxController(0);
  private final XboxController swerveController = new XboxController(1);
  private final JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
  private final JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);

  private final SequentialCommandGroup autonomousCommand;
  private final SendableChooser<String> autoChooser = new SendableChooser<String>();
  
  public RobotContainer() {
    autoChooser.setDefaultOption(
      "Blue Left", null);
    autoChooser.addOption(
      "Blue Middle", null);
    autoChooser.addOption(
      "Blue Right", null);
    autoChooser.addOption(
      "Red Left", null);
    autoChooser.addOption(
      "Red Middle", null);
    autoChooser.addOption(
      "Red Right", null);

    this.autonomousCommand = new SequentialCommandGroup(
      new Shoot(shooterSubsystem, () -> ShootingSpeedCalculators.simple(
        swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), 
        teamColor.equals("blue") ? Constants.FieldConstants.blueX : Constants.FieldConstants.redX, 
        Constants.FieldConstants.hubY))
      );

    configureButtons();
  }

  private void configureButtons() {
    //TODO: Migrate swerve to another controller

    //Intake

    DoubleSupplier spinnerSpeed = () -> MathUtil.copyDirectionPow(
      MathUtil.applyDeadband(xboxController.getRightY(), 1e-2), 
      2
      );

    DoubleSupplier indexerSpeed = () -> MathUtil.copyDirectionPow(
      MathUtil.applyDeadband(xboxController.getLeftY(), 1e-2), 
      2
      );

    aButton.onTrue(new RaiseIntake(intakeSystem));
    bButton.onTrue(new LowerIntake(intakeSystem));

    intakeSystem.setDefaultCommand(
        new RunIntake(intakeSystem, spinnerSpeed)
    );

    //Swerve

    swerveDrive.setDefaultCommand(new DriveCommand(
      () -> MathUtil.applyDeadband(
          swerveController.getLeftY(),
          ControllerConstants.joystickDeadband
      ),
      () -> MathUtil.applyDeadband(
          swerveController.getLeftX(),
          ControllerConstants.joystickDeadband
      ),
      () -> MathUtil.applyDeadband(
          swerveController.getRightX(),
          ControllerConstants.joystickDeadband
      ),
      true,
      true,
      swerveDrive
  ));

  //Elevator

  xButton.onTrue(new DynamicElevation(elevatorSubsystem));

  //Shooter

  shooterSubsystem.setDefaultCommand(new ParallelCommandGroup(
    new ShootAndIndexer(shooterSubsystem, spinnerSpeed, indexerSpeed)
  ));
  
  //shooterSubsystem.setDefaultCommand(new AutoUpdateShootingSpeed(shooterSubsystem, swerveDrive, teamColor));
}


  public SendableChooser<String> getAutoChooser() {
    return autoChooser;
  }

  public void setAutoNameAndTeamColor(String autoName) {
    this.autoName = autoName;
    this.teamColor = this.autoName.split(" ")[0].strip().toLowerCase();
    shooterSubsystem.setTeamColor(this.teamColor);
  }

  public String getTeamColor() {
    return teamColor;
  }

  public Swerve getSwerve() {
    return swerveDrive;
  }

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }
}