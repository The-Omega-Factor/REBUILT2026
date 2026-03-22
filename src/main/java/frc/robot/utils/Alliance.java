package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class Alliance {
    private static DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

    public static void update() {
        DriverStation.getAlliance().ifPresent(a -> alliance = a);
    }

    public static DriverStation.Alliance getAlliance() {
        return alliance;
    }
}
