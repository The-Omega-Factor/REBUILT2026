package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static final Intake intakeSystem = new Intake();
    private final TalonFX intake;
    private double targetPosition = 0.0;
    private PIDController intakePID = new PIDController(IntakeConstants.Kp, IntakeConstants.Ki, IntakeConstants.Kd);

    private Intake() {
        intake = new TalonFX(IntakeConstants.intakeID, new CANBus(IntakeConstants.intakeCanName));
        intake.setPosition(0);
    }

    public double getIntakePosition() {
        return intake.getPosition().getValueAsDouble();
    }

    public void goToTarget() {
        if (Math.abs(getIntakePosition() - IntakeConstants.targetPosition) < 1e-2) {
            targetPosition = 0.0;
        } else {
            targetPosition = IntakeConstants.targetPosition;
        }

        intakePID.setSetpoint(targetPosition);
    }

    public PIDController getPIDController() {
        return intakePID;
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public static Intake getInstance() {
        return intakeSystem;
    }
}
