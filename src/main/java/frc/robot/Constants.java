/*
public static double kP = 0.0;
public static double kI = 0.0;
public static double kD = 0.0;
public static double kS = 0.0;
public static double kV = 0.0;
public static double kA = 0.0; 
public static double kG = 0.0; 
 */

package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static CANBus canbus = new CANBus("SwerveBase");

    public static class Shooter {
        public static final int shooterLID = 0;
        public static final int shooterRID = 14;
        public static final int hopperID = 5;

        public static final double shootersGearRatio = 25/20;
        public static final double shootersStatorAmpsLimit = 100;
        public static final double shootersCurrentLimit = 45;
        public static final double shooterSpeedMultiplier = 1;
        public static final double shooterErrorTolerance = 1e-2;
        
        public static final double hopperGearRatio = 20/35;
        public static final double hopperStatorAmpsLimit = 67;
        public static final double hopperCurrentLimit = 45;
        public static final double hopperErrorTolerance = 1e-2;

        public static class ShooterPIDS {
            public static final double kV = 0.5;
            public static final double kP = 0.5;
            public static final double kS = 0.0;
            public static final double kA = 0.0; 
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static class HopperPIDs {
            public static final double kP = 0.0; 
            public static final double kS = 0.4;  
            public static final double kV = 0.1;  
            public static final double kD = 0.0;  
        }

        public static final double distanceBasedMultiplier = 1;
    }

    public static class Intake {
        public static final int spinID = 27;
        public static final double spinGearRatio = 20/20;
        public static final double spinStatorAmpsLimit = 50;
        public static final double spinCurrentLimit = 45;
        public static final double spinSpeedMultiplier = 80;
        public static final double spinErrorTolerance = 1e-2;

        public static class SpinPIDs {
            public static final double kP = 0.3;
            public static final double kV = 0.0;
            public static final double kS = 0.0;
            public static final double kI = 0.0; 
        }

        public static final int pivotID = 24;
        public static final double pivotGearRatio = 20/20;
        public static final double pivotStatorAmpsLimit = 50;
        public static final double pivotCurrentLimit = 45;
        public static final double pivotSpeedMultiplier = 2;
        public static final double pivotErrorTolerance = 1e-2;

        /*
        WARNING: USE "System.out.println(intakeSubsystem.getPivotPosition());" to in
        SetIntakeState.java within the "public void execute()" method to tune these 
        values before uncommenting the limiter lines.
        
        The limiter lines are in: 
        1. IntakeSubsystem.java - ~line 68-69 in the "public IntakeSystem()" constructor
        2. SetIntakeState.java - ~line 31 in the "public void execute()" method
         */
        public static final double pivotLowerLimit = 0;
        public static final double pivotUpperLimit = 0;

        public static class pivotPIDs {
            public static final double kP = 0.5;
            public static final double kD = 0.0;
            public static final double kG = 0.0;
        }
    }

    public static class Swerve {
        public static final double maxOmega = 6.0; //rad/s
        public static final double minOmega = 0.0;
        public static final double angularTolerance = 5; //rad

        public static class AutoBuilderPIDs {
            public static class Translational {
                public static final double kP = 0.5;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }

            public static class Rotational {
                public static final double kP = 0.5;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }
        }
    }

    public static final class Controllers {
        public static final int swerveID = 0;
        public static final int notSwerveID = 1; 
        
        public static final double deadband = 0.05;
    }

    public static final class Field {
        //(meters)

        public static final double y = 4;
        public static final double redX = 0;
        public static final double blueX = 0;
  }
}
