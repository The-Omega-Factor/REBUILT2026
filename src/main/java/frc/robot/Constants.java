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

    public static class ShooterConstants {
        public static int shooterLID = 0;
        public static int shooterRID = 14;
        public static int hopperID = 5;

        public static double shootersGearRatio = 25/20;
        public static double hopperGearRatio = 20/35;

        public static double shootersAmpsLimit = 67;
        public static double hopperAmpsLimit = 67;

        public static class shooterPIDs {
            public static double kV = 0.5;
            public static double kP = 0.5;
            public static double kS = 0.0;
            public static double kA = 0.0; 
            public static double kI = 0.0;
            public static double kD = 0.0;
        }

        public static class hopperPIDs {
            public static double kP = 0.5; 
            public static double kS = 0.0;  
            public static double kV = 0.0;  
            public static double kD = 0.0;  
        }
    }
}
