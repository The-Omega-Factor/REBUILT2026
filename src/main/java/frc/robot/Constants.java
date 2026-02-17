// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //TODO: Check all constants

  public static class IntakeConstants {
    public static final int INTAKE_SPIN_ID = 67;
    public static final int INTAKE_PIVOT_ID = 24;

    public static final double PIVOT_GEARING = 1.0;        //reduction
    public static final double PIVOT_DISENGAGED_POSITION = 10;
    public static final double PIVOT_ENGAGED_POSITION = 0.0;

    public static final double KP = 0.7;
    public static final double KI = 0.0;
    public static final double KD = 0.3;

    public static final double PID_ERROR_TOLERANCE = 1e-2;
  }

  public static final class Swerve {
    public static final Translation2d flModuleOffset = new Translation2d(0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(0.546 / 2.0, -0.546 / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-0.546 / 2.0, -0.546 / 2.0);

    public static final double maxModuleSpeed = 4.5; // M/S

    public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);
  }

  public static final class Limelight {
    public static final String name = "Limelight"; 
  }

  public static final class CANDevices {

      public static final String CANivoreName = "SwerveBase";

      public static final int PigeonID = 16;

      public static final int frontLeftSteerMotorID = 2;
      public static final int frontLeftDriveMotorID = 13;
      public static final int frontLeftCANCoderID = 9;

      public static final int frontRightSteerMotorID = 4;
      public static final int frontRightDriveMotorID = 21;
      public static final int frontRightCANCoderID = 12;

      public static final int backLeftSteerMotorID = 6;
      public static final int backLeftDriveMotorID = 1;
      public static final int backLeftCANCoderID = 11;

      public static final int backRightSteerMotorID = 8;
      public static final int backRightDriveMotorID = 18;
      public static final int backRightCANCoderID = 10;
  }

  public static final class DrivetrainConstants {

        //TODO: Measure from center of each wheel to the center of the the other wheel

        public static final double trackWidth = Units.inchesToMeters(24);

        public static final double wheelBase = Units.inchesToMeters(24);

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth/ 2.0, wheelBase / 2.0), // front left module
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right module
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // back left module
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // back right module
            );

        public static final double driveMotorGearReduction = 6.75 / 1.0;
        public static final double steerMotorGearReduction = ((150.0 / 7.0) / 1.0);

        public static final double wheelRadiusMeter = Units.inchesToMeters(1.96875); //Diameter is ~ 4 inchs
        public static final double wheelCircumferenceMeters = 2.0 * wheelRadiusMeter * Math.PI;

        public static final double driveMetersPerEncoderRevolution = wheelCircumferenceMeters / driveMotorGearReduction;
        public static final double driveMetersPerSecRPM = driveMetersPerEncoderRevolution / 60;

        public static final double steerRadiansPerEncoderRevolution = 2.0 * Math.PI / steerMotorGearReduction;

        public static final double kFreeMetersPerSecond = 5600 * driveMetersPerSecRPM;

        public static final double steerMotorMaxSpeedRadianPerSecond = 2.0;
        public static final double steerMotorMaxAccelRadPerSecSq = 1.0;

        public static final double maxDriveSpeedMetersPerSec = Units.feetToMeters(15.5);
        public static final double maxTurnRateRadPerSec = Units.rotationsToRadians(2.0);

        public static final Rotation2d frontLeftModOffset = Rotation2d.fromRotations(0.564453); // Rotations + 0.5 Radians + PI Degrees + 180
        public static final Rotation2d frontRightModOffset = Rotation2d.fromRotations(0.393066);
        public static final Rotation2d backLeftModOffset = Rotation2d.fromRotations(0.471436);
        public static final Rotation2d backRightModOffset = Rotation2d.fromRotations(0.517578);

        public static final int frontLeftModuleNumber = 0;
        public static final int frontRightModuleNumber = 1;
        public static final int backLeftModuleNumber = 2;
        public static final int backRightModuleNumber = 3;

        public static final int driveCurrentLimitAmps = 40;
        public static final int steerCurrentLimitAmps = 20;
        public static final int driveCurrentLimitThreshold = 60;
        public static final int steerCurrentLimitThreshold = 40;

        public static final double drivekP = 0.005;
        public static final double drivekD = 0.0;

        public static final double steerkP = 0.37431;
        public static final double steerkD = 0.27186;

        public static final double ksVolts = 0.667;
        public static final double kvVoltSecsPerMeter = 2.44;
        public static final double kaVoltSecsPerMeterSq = 0.0;
        
        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);
     

    }

    public static final class AutoConstants {
            
        public static final double maxVelMetersPerSec = Units.feetToMeters(8);

        public static final double drivekP = 12.8;
        public static final double drivekD = 0.085;

        public static final PIDConstants driveConstants = new PIDConstants(drivekP, drivekD);

        public static final double rotkP = 1.27;
        public static final double rotkD = 0.5;

        public static final PIDConstants rotConstants = new PIDConstants(rotkP, rotkD);

    }

    public static final class ShooterConstants {
      public static final int shooterLID = 0;
      public static final int shooterRID = 14;
      public static final int indexerID = 5;

      public static final double shooterGearing = 1;
      public static final double indexerGeraing = 1;

      public static final double KP = 1;
      public static final double KI = 0;
      public static final double KD = 0.1;

      public static final double heightFromGround = 17; //inch

      public static final double simpleShootingSpeedMultiplier = 10;
      public static final double drivershootingSpeedMultipler = 50;
      public static final double noAirResistanceSpeedMultiplier = 10;
    }

    public static final class ElevatorConstants {
      public static final int elevatorID = 7;
      public static final double elevatorGearing = 1;

      public static final double retractedPosition = 0;
      public static final double extendedPosition = 1;

      public static final double KP = 1;
      public static final double KI = 0;
      public static final double KD = 0.1;

      public static final double errorTolerance = 1e-2;
    }

    public static final class FieldConstants {
      public static final double goalHeight = 70; //inch

      public static final double hubY = 15;

      public static final double redX = 15;
      public static final double blueX = 15;
    }

    public static final class ControllerConstants {

        public static final int driverGamepadPort = 0;
        public static final int manipulatorGamepadPort = 1;

        public static final double joystickDeadband = 0.1;

        public static final double triggerPressedThreshold = 0.25;

    }
}