package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static final Intake intakeSystem = new Intake();
    private final TalonFX intake;

    private Intake() {
        intake = new TalonFX(IntakeConstants.intakeID, new CANBus(IntakeConstants.intakeCanName));
    }

    public void setSpeed(double speed) {
        intake.set(speed);
    }

    public double getIntakePosition() {
        return intake.getPosition().getValueAsDouble();
    }
}
