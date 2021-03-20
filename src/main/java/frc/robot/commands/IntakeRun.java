// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRun extends CommandBase {
  /** Creates a new IntakeOperationCommand. */
  private final IntakeSubsystem m_intake;
  private final XboxController m_controller;

  public IntakeRun(IntakeSubsystem intake, XboxController controller)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);

    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_intake.startIntakeMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_intake.stopIntakeMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
