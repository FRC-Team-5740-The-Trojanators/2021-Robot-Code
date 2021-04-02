// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TargetCommand extends CommandBase {
  /** Creates a new TargetCommand. */
 ShooterSubsystem m_shooter;
 DriveSubsystem m_drivetrain;
 boolean m_isFinished;
 int m_count;

  public TargetCommand(ShooterSubsystem shooter, DriveSubsystem drivetrain)
  {
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    addRequirements(m_shooter, m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_shooter.ledOn();
    m_count = 0;
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_shooter.getAimPID();
    if ((!m_shooter.aimEnd()) && (m_count < 10))
    {
      m_drivetrain.drive(0, 0, m_shooter.turnShooter(), false);
    }
    
    if(m_shooter.aimEnd()) 
    { 
      m_count++;
    }
    
    if(m_shooter.aimEnd() && m_count >= 10)
    {
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_drivetrain.drive(0, 0, 0, false);
    m_isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return m_isFinished;
  }
}
