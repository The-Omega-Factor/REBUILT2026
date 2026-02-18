// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class motorSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(1, "SwerveBase");
  private VelocityVoltage velocityRquest = new VelocityVoltage(0);
  private final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

  public motorSubsystem() {
    talonFXConfiguration.Slot0.kP = 0.44;

    motor.getConfigurator().apply(talonFXConfiguration);
  }

  public void setVelocity(double speed) {
    motor.setControl(velocityRquest.withVelocity(speed));
  }
}
