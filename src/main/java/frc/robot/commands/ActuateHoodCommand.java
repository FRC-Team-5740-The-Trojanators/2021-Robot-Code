// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ActuateHoodCommand extends CommandBase {
  /** Creates a new ActuateHoodCommand. */
  ShooterSubsystem m_shooter;
  Boolean m_isFinished;

  public ActuateHoodCommand(ShooterSubsystem shooter) 
  {
    m_shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_isFinished = false;
    m_shooter.setHoodMotor(m_shooter.actuateHood());
    m_shooter.runFlyWheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
   m_shooter.setHoodMotor(m_shooter.actuateHood());
   m_shooter.runFlyWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_isFinished = true;
    m_shooter.forceStopHoodMotor();
    m_shooter.stopFlyWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
