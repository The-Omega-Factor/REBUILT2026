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
        public static int shooterLID = 0;
        public static int shooterRID = 14;
        public static int hopperID = 5;

        public static double shootersGearRatio = 25/20;
        public static double shootersStatorAmpsLimit = 100;
        public static double shootersCurrentLimit = 45;
        public static double shooterSpeedMultiplier = 1;
        
        public static double hopperGearRatio = 20/35;
        public static double hopperStatorAmpsLimit = 67;
        public static double hopperCurrentLimit = 45;

        public static class ShooterPIDS {
            public static double kV = 0.5;
            public static double kP = 0.5;
            public static double kS = 0.0;
            public static double kA = 0.0; 
            public static double kI = 0.0;
            public static double kD = 0.0;
        }

        public static class HopperPIDs {
            public static double kP = 0.0; 
            public static double kS = 0.4;  
            public static double kV = 0.1;  
            public static double kD = 0.0;  
        }

        public static final double distanceBasedMultiplier = 1;
    }

    public static class Intake {
        public static final int spinID = 27;
        public static final double spinGearRatio = 20/20;
        public static final double spinStatorAmpsLimit = 50;
        public static final double spinCurrentLimit = 45;
        public static final double spinSpeedMultiplier = 80;

        public static class SpinPIDs {
            public static double kP = 0.3;
            public static double kV = 0.0;
            public static double kS = 0.0;
            public static double kI = 0.0; 
        }

        public static final int pivotID = 24;
        public static final double pivotGearRatio = 20/20;
        public static final double pivotStatorAmpsLimit = 50;
        public static final double pivotCurrentLimit = 45;
        public static final double pivotSpeedMultiplier = 2;

        public static class pivotPIDs {
            public static double kP = 0.5;
            public static double kD = 0.0;
            public static double kG = 0.0;
        }
    }

    public static class Swerve {
        public static final double maxOmega = 6.0; //rad/s
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
