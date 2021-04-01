// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodCommand extends CommandBase {
  /** Creates a new HoodCommand. */
  HoodSubsystem m_hood;
  Boolean m_isFinished;

  public HoodCommand(HoodSubsystem hood) 
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
    m_hood.setHoodMotor(m_hood.hoodSetSetpoint(0.2));//m_hood.hoodAngleFinder());

    //m_hood.setHoodMotor(m_hood.hoodAngleFinder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
   m_hood.setHoodMotor(m_hood.hoodSetSetpoint(0.2));//m_hood.hoodAngleFinder());   
   SmartDashboard.putNumber("Hood Encoder", m_hood.getAbsEncoder());
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
