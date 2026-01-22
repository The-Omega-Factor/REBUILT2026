package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.intake.IntakeSystem;

public class RobotContainer {
  private final IntakeSystem intakeSystem = new IntakeSystem();
  
  private final XboxController xboxController = new XboxController(0);
  private final JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);

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

    configureIntakeButtons();
  }

  private void configureIntakeButtons() {
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
  }

  public SendableChooser<String> getAutoChooser() {
    return autoChooser;
  }
}
