// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TargetCommand extends CommandBase {
  /** Creates a new TargetCommand. */
 ShooterSubsystem m_shooter;
 DriveSubsystem m_drivetrain;
 XboxController m_controller;

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
  }

  public void turnShooter(){
    var tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if(tx > 1)
    {
      m_drivetrain.drive(0, 0, ShooterConstants.shooterRotationLeft, false);
    } else if(tx < -1)
    {
      m_drivetrain.drive(0, 0, ShooterConstants.shooterRotationRight, false);
    }
  }

  public void actuateHood()
  {
    var ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    //double angle = ty + HoodConstants.limelightAngle;
    //var distance = HoodConstants.heightDifference / tan(angle);
    // TODO turn distance into setpoint (is it proportional or do we need if/then)
    double temporarySetpoint = 100000;
    m_shooter.hoodSetSetpoint(temporarySetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    turnShooter();
    actuateHood();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
