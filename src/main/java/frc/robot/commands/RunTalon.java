// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TalonTesting;
import edu.wpi.first.wpilibj.GenericHID;



public class RunTalon extends CommandBase {
  /** Creates a new RunTalon. */

  private TalonTesting m_talon;
  private XboxController m_controller;

  public RunTalon(TalonTesting talon) {
    m_talon = talon;
    //m_controller = controller;
    addRequirements(talon);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_talon.setSetpoint(300);
    //m_talon.runTalon(m_controller.getX(GenericHID.Hand.kLeft));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
   {
    // m_talon.stopTalon();
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    //m_talon.stopTalon();
    return false;
  }
}
