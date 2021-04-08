// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.AutoChooser;
import frc.robot.Constants.SwerveDriveModuleConstants.rotationPID;
import frc.robot.paths.BlueAPath;
import frc.robot.paths.BlueBPath;
import frc.robot.paths.RedAPath;
import frc.robot.paths.RedBPath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RotateRobot extends CommandBase {
  /** Creates a new RotateRobot. */
DriveSubsystem m_driveSubsystem;
PIDController m_rotatePID;
IntakeSubsystem m_intakeSubsystem;
boolean m_isFinished;
ShooterSubsystem m_shooter;

  public RotateRobot(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooter)
  {
    m_driveSubsystem = driveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_shooter = shooter;
    addRequirements(driveSubsystem, intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_shooter.ledOn();
    m_isFinished = false;
    m_rotatePID = new PIDController(rotationPID.k_rotationP, rotationPID.k_rotationI, rotationPID.k_rotationD);
    m_rotatePID.setTolerance(rotationPID.k_rotationTolerance);
    m_rotatePID.setSetpoint(180);

    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    if( tx <= AutoChooser.k_RedB + AutoChooser.k_autoTolerance && tx > AutoChooser.k_RedB - AutoChooser.k_autoTolerance)
    {
      m_driveSubsystem.setTrajectory(RedBPath.getTrajectory());
      SmartDashboard.putBoolean("RedB", true);
    }
    else if(tx <= AutoChooser.k_RedA + AutoChooser.k_autoTolerance && tx > AutoChooser.k_RedA - AutoChooser.k_autoTolerance)
    {
      m_driveSubsystem.setTrajectory(RedAPath.getTrajectory());
      SmartDashboard.putBoolean("RedA", true);
    }
    else if(tx <= AutoChooser.k_BlueA + AutoChooser.k_autoTolerance && tx > AutoChooser.k_BlueA - AutoChooser.k_autoTolerance)
    {
      m_driveSubsystem.setTrajectory(BlueAPath.getTrajectory());
      SmartDashboard.putBoolean("BlueA", true);
    }
    else if(tx <= AutoChooser.k_BlueB + AutoChooser.k_autoTolerance && tx > AutoChooser.k_BlueB - AutoChooser.k_autoTolerance)
    {
      m_driveSubsystem.setTrajectory(BlueBPath.getTrajectory());
      SmartDashboard.putBoolean("BlueB", true);
    }

    m_driveSubsystem.resetIMU();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    double rot = m_rotatePID.calculate(m_driveSubsystem.m_imu.getAngle() * Math.PI/180);
    // if(rot >= .2)
    // {
    //   rot = .2;
    // }
    m_driveSubsystem.drive(0, 0, rot, false);
    SmartDashboard.putNumber("IMU", m_driveSubsystem.m_imu.getAngle());

     m_intakeSubsystem.extendIntake();
    ///m_intakeSubsystem.startIntakeMotors();

    if(m_driveSubsystem.m_imu.getAngle() >= 180)
    {
      m_isFinished = true;

    }
   
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_driveSubsystem.drive(0, 0, 0, false);
    m_isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
