package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class ChooserPublisher {
    private final NetworkTable table;
    private final SendableBuilderImpl builder;

    public ChooserPublisher(String tableName, SendableChooser<Command> chooser) {
        table = NetworkTableInstance.getDefault().getTable(tableName);

        builder = new SendableBuilderImpl();
        builder.setTable(table);

        SendableRegistry.publish(chooser, builder);
        builder.startListeners();
    }

    public void update() {
        builder.update();
    }
}
