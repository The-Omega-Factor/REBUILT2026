package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.elevator.DynamicElevation;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.intake.RunIntake;
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
  private final JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
  private final JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);

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

    configureButtons();
  }

  private void configureButtons() {
    //Intake

    DoubleSupplier spinnerSpeed = () -> MathUtil.copyDirectionPow(
      MathUtil.applyDeadband(xboxController.getRightY(), 1e-2), 
      2
      );

    aButton.onTrue(new RaiseIntake(intakeSystem));
    bButton.onTrue(new LowerIntake(intakeSystem));

    intakeSystem.setDefaultCommand(
      new ParallelCommandGroup(
        new RunIntake(intakeSystem, spinnerSpeed),
        new RunCommand(() -> intakeSystem.setIntakePivotPosition(intakeSystem.getPivotPosition()), intakeSystem)
      )
    );

    //Swerve

    swerveDrive.setDefaultCommand(new DriveCommand(
      () -> MathUtil.applyDeadband(
          xboxController.getLeftY(),
          ControllerConstants.joystickDeadband
      ),
      () -> MathUtil.applyDeadband(
          xboxController.getLeftX(),
          ControllerConstants.joystickDeadband
      ),
      () -> MathUtil.applyDeadband(
          xboxController.getRightX(),
          ControllerConstants.joystickDeadband
      ),
      true,
      true,
      swerveDrive
  ));

  //Elevator

  xButton.onTrue(new DynamicElevation(elevatorSubsystem));
}


  public SendableChooser<String> getAutoChooser() {
    return autoChooser;
  }

  public void setAutoNameAndTeamColor(String autoName) {
    this.autoName = autoName;
    this.teamColor = this.autoName.split(" ")[0];
    shooterSubsystem.setTeamColor(this.teamColor);
  }

  public String getTeamColor() {
    return teamColor;
  }

  public Swerve getSwerve() {
    return swerveDrive;
  }
}