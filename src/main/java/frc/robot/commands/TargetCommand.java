// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.HoodConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TargetCommand extends CommandBase {
  /** Creates a new TargetCommand. */
 ShooterSubsystem m_shooter;
 DriveSubsystem m_drivetrain;
 XboxController m_controller;
 boolean m_isFinished;

  public TargetCommand(ShooterSubsystem shooter, DriveSubsystem drivetrain, XboxController controller)
  {
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    addRequirements(m_shooter, m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_shooter.ledOn();
    m_isFinished = false;
    m_drivetrain.drive(0, 0, m_shooter.turnShooter(), false);
  }


 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_drivetrain.drive(0, 0, m_shooter.turnShooter(), false);

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
