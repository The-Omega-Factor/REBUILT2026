package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public final class AutoSelector implements AutoCloseable{
    private SendableChooser<Command> autoChooser;

    private final SendableBuilderImpl builder = new SendableBuilderImpl();
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Autonomous");

    public AutoSelector(SendableChooser<Command> autoChooser) {
        this.autoChooser = autoChooser;

        builder.setTable(table);
        SendableRegistry.publish(autoChooser, builder);
        builder.startListeners();
    }

    public Command getSelected() {
        return autoChooser.getSelected();
    }

    @Override
    public void close() {
        builder.close();
    }
}
