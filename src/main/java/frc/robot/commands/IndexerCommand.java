// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends CommandBase {
  /** Creates a new IndexerCommand. */
  IndexerSubsystem m_indexer;
  Timer indexerTime;
  boolean m_isFinished;

  public IndexerCommand(IndexerSubsystem indexer) {
    m_indexer = indexer;
    addRequirements(m_indexer);
    indexerTime = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_indexer.runIndexerWheel();
    indexerTime.reset();
    indexerTime.start();
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(indexerTime.get()%1 < 0.5)
    {
      m_indexer.runIndexerWheel();
    }
    else
    {
      m_indexer.stopIndexerWheel();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    indexerTime.stop();
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
