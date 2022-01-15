package frc.robot.Util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DriveSubsystem;

public class Dashboard {
    public void testDashboard(){
        Shuffleboard.getTab("test_")
        .add("test_", true)
        .withWidget(BuiltInWidgets.kBooleanBox);

    }
}
