package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CG_Bounce extends SequentialCommandGroup {
    
    public CG_Bounce(){
        addCommands(
                // new AutonomousDrive("bounce_1"),
                // new AutonomousDrive("bounce_2"),
                // new AutonomousDrive("bounce_3"),
                // new AutonomousDrive("bounce_4")
        );
    } 
}