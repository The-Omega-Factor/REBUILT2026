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
        public static double hopperGearRatio = 20/35;

        public static double shootersAmpsLimit = 100;
        public static double hopperAmpsLimit = 67;

        public static class ShooterPIDS {
            public static double kV = 0.5;
            public static double kP = 0.5;
            public static double kS = 0.0;
            public static double kA = 0.0; 
            public static double kI = 0.0;
            public static double kD = 0.0;
        }

        public static class HopperPIDs {
            public static double kP = 0.5; 
            public static double kS = 0.0;  
            public static double kV = 0.0;  
            public static double kD = 0.0;  
        }
    }

    public static class Intake {
        public static final int spinID = 100;
        public static final double spinGearRatio = 20/20;
        public static final double spinAmpsLimit = 50;

        public static class SpinPIDs {
            public static double kP = 0.5;
            public static double kV = 0.0;
            public static double kS = 0.0;
            public static double kI = 0.0; 
        }

        public static final int pivotID = 100;
        public static final double pivotGearRatio = 20/20;
        public static final double pivotAmpsLimt = 50;

        public static class pivotPIDs {
            public static double kP = 0.5;
            public static double kD = 0.0;
            public static double kG = 0.0;
        }
    }
}
