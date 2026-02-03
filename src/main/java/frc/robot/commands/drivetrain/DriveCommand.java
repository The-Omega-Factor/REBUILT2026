package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Swerve;

public class DriveCommand extends Command {

    private final Swerve swerve;

    //TODO: Limelight Locking Commands

    private final XboxController Ctrl = new XboxController(ControllerConstants.driverGamepadPort);

    private final DoubleSupplier drive;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rot;

    private final boolean isFieldRelative;
    private final boolean squareInputs;

    double LL2Drive;
    double LL2Rot;
    boolean LLField;
    double LL3ADrive;
    double LL3ARot;

    public DriveCommand(DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier rot, boolean isFieldRelative, boolean squareInputs, Swerve swerve) {
        this.swerve = swerve;

        this.drive = drive;
        this.strafe = strafe;
        this.rot = rot;

        this.isFieldRelative = isFieldRelative;
        this.squareInputs = squareInputs;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        double drive = this.drive.getAsDouble();
        double strafe = this.strafe.getAsDouble();
        double rot = this.rot.getAsDouble();

        if (squareInputs) {
            double theta = Math.atan2(drive, strafe);
            double r = Math.pow(Math.hypot(drive, strafe), 2.0);

            drive = r * Math.sin(theta);
            strafe = r * Math.cos(theta);

            rot = Math.copySign(Math.pow(rot, 2.0), rot) * 1.4;
        }

        if (Ctrl.getXButton()) {

            double LL2tx;
            LL2tx = LimelightHelpers.getTX("limelight-right");
            double LL2ty;
            LL2ty = LimelightHelpers.getTY("limelight-right");

            if (LL2ty > 8 || LL2ty < 4) { //5.7
                LL2Drive = Math.abs(LL2ty * 0.05);
            } else {
                LL2Drive = 0;
            }

            if (LL2tx > -14 || LL2tx < -10) { //-9.7
                LL2Rot = LL2tx * 0.0002;
            } else {
                LL2Rot = 0;
            }

            LLField = false;

        } else if (Ctrl.getBButton()) {

            double LL3Atx;
            LL3Atx = LimelightHelpers.getTX("limelight-object");
            double LL3Aty;
            LL3Aty = LimelightHelpers.getTY("limelight-object");

            if (LL3Aty > 8 || LL3Aty < 4) {
                LL3ADrive = Math.abs(LL3Aty * 0.05);
            } else {
                LL3ADrive = 0;
            }

            if (LL3Atx > 3 || LL3Atx < -3) {
                LL3ARot = LL3Atx * 0.0002;
            } else {
                LL3ARot = 0;
            }

            LLField = false;

        } else {

            LL2Drive = 0;
            LL2Rot = 0;
            LL3ADrive = 0;
            LL3ARot = 0;
            LLField = isFieldRelative;

        }

        swerve.drive(
            (-drive + LL2Drive + LL3ADrive) * DrivetrainConstants.maxDriveSpeedMetersPerSec, 
            (-strafe - LL2Rot - LL3ARot) * DrivetrainConstants.maxDriveSpeedMetersPerSec, 
            (-rot) * DrivetrainConstants.maxTurnRateRadPerSec, 
            LLField //isFieldRelative and delete limelight stuff
        );

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {

        return true;
        
    }
    
}