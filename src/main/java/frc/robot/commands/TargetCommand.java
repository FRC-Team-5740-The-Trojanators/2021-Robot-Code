// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TargetCommand extends CommandBase {
  /** Creates a new TargetCommand. */
 ShooterSubsystem m_shooter;
 DriveSubsystem m_drivetrain;
 XboxController m_controller;

  public TargetCommand(ShooterSubsystem shooter, DriveSubsystem drivetrain, XboxController controller)
  {
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    addRequirements(m_shooter, m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_shooter.ledOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(m_shooter.getX() == 1 && m_shooter.getY() == 1 && m_shooter.seesTarget())
    {
      //TODO adjust robot pose
    };
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
