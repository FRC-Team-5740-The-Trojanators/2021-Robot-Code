// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveModuleConstants.HoodConstants;
import frc.robot.subsystems.HoodSubsystem;

public class HoodDefaultCommand extends CommandBase {
  /** Creates a new HoodCommand. */
  HoodSubsystem m_hood;
  boolean m_isFinished;

  public HoodDefaultCommand(HoodSubsystem hood) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hood = hood;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_hood.hoodSetSetpoint(HoodConstants.k_retractQuadSetpoint);
    if(!m_hood.hoodMoveEnd())
    {
    m_hood.setHoodMotor(m_hood.hoodSetSetpoint(HoodConstants.k_retractQuadSetpoint)); 
    }
    m_isFinished = m_hood.hoodMoveEnd();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_isFinished = true;
    m_hood.forceStopHoodMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
