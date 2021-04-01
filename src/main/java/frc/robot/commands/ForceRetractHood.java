// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class ForceRetractHood extends CommandBase {
  /** Creates a new ForceRetractHood. */
  HoodSubsystem m_hood;

  public ForceRetractHood(HoodSubsystem hood)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hood = hood;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_hood.forceRunHoodMotorRetract();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_hood.forceRunHoodMotorRetract();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_hood.forceStopHoodMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
