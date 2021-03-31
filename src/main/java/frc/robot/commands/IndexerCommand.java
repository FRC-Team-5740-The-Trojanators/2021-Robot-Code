// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexerCommand extends CommandBase {
  /** Creates a new ShootCommand. */
  DriveSubsystem m_drivetrain;
  IndexerSubsystem m_indexer;
  XboxController m_controller;
  boolean m_isFinished;
  public IndexerCommand( DriveSubsystem drivetrain, XboxController controller, IndexerSubsystem indexer) {
    m_drivetrain = drivetrain;
    m_indexer = indexer;
    addRequirements( m_drivetrain, m_indexer);
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_indexer.runIndexerWheel();
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_indexer.runIndexerWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_indexer.stopIndexerWheel();
    m_isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return m_isFinished;
  }
}
