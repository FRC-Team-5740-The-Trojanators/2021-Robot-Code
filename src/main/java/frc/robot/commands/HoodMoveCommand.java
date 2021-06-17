// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodMoveCommand extends CommandBase {
  /** Creates a new HoodMoveCommand. */
  HoodSubsystem m_hood;
  boolean m_isFinished;
  double m_setpoint;
  double m_ty;
  
  public HoodMoveCommand(HoodSubsystem hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hood = hood;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_isFinished = false;
    m_ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    m_setpoint = m_hood.hoodAngleFinder(m_ty);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    m_setpoint = m_hood.hoodAngleFinder(m_ty);
    SmartDashboard.putNumber("hood setpoint", m_setpoint);
    SmartDashboard.putNumber("LL ty", m_ty);

    m_hood.hoodSetSetpoint(m_setpoint);
    if(!m_hood.hoodMoveEnd())
    {
    m_hood.setHoodMotor(m_hood.hoodSetSetpoint(m_setpoint)); 
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
