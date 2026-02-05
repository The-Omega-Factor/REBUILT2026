package frc.robot.commands.shooter;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.math.ShootingSpeedCalculators;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoUpdateShootingSpeed extends Command {
    private final ShooterSubsystem shooter;
    private final Swerve swerve;
    private final double fieldX;
    private final double fieldY;
    private final String calculatorType;

    public AutoUpdateShootingSpeed(ShooterSubsystem shooter, Swerve swerve, String teamColor, String calculatorType) {
        this.shooter = shooter;
        this.swerve = swerve;

        this.fieldX = teamColor == "RED" ? FieldConstants.redX : FieldConstants.blueX;
        this.fieldY = FieldConstants.hubY; 

        this.calculatorType = calculatorType.strip().toLowerCase();

        addRequirements(List.<Subsystem>of(shooter, swerve));
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();

        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        
        if (calculatorType.equals("no air resistance")) {
            shooter.setShooterVelocity(ShootingSpeedCalculators.withoutAirResistance(currentX, currentY, fieldX, fieldY, 30));
        } else {
            shooter.setShooterVelocity(ShootingSpeedCalculators.simple(currentX, currentY, fieldX, fieldY));
        }
    }
}