// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheelSubsystem;

public class FlyWheelCommand extends CommandBase {
  /** Creates a new FlyWheelCommand. */
  FlyWheelSubsystem m_flyWheel;
  Boolean m_isFinished;
  int flywheelSetpoint;
  double m_ty;

  public FlyWheelCommand(FlyWheelSubsystem flyWheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flyWheel = flyWheel;
    addRequirements(flyWheel);
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
    //flywheelSetpoint = (int) SmartDashboard.getNumber("Flywheel Velocity Input", 0);
    //SmartDashboard.putNumber("Flywheel Velocity Input", flywheelSetpoint);
    
    m_ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    m_flyWheel.setFlywheelRPM((int) m_flyWheel.flywheelSpeedFinder(m_ty));
    //m_flyWheel.setFlywheelRPM(4900);

    SmartDashboard.putNumber("Flywheel Velocity", m_flyWheel.getFlywheelVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_isFinished = true;
    m_flyWheel.stopFlyWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return m_isFinished;
  }
}
