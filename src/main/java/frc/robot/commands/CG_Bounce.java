package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class CG_Bounce extends SequentialCommandGroup {
    public CG_Bounce(){
        addCommands(
                new AutonomousDrive(driveSubsystem, "bounce_1"),
                new AutonomousDrive(driveSubsystem, "bounce_2"),
                new AutonomousDrive(driveSubsystem, "bounce_3"),
                new AutonomousDrive(driveSubsystem, "bounce_4")
        );
    }, 
}