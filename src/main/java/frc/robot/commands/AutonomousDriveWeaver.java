// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.AutoChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.IntakeSubsystem;
import lib.swerve.SwervePath;
import lib.swerve.SwervePathController;

public class AutonomousDriveWeaver extends CommandBase {

  private Trajectory m_trajectory; 

  private final DriveSubsystem m_driveSubsystem;

  private HolonomicDriveController m_driveController; 

  private PIDController m_xController;
  private PIDController m_yController;
  private ProfiledPIDController m_rotController; 
  private TrapezoidProfile.Constraints m_trapezoidProfile;
  private Trajectory.State m_goal;
  private Boolean m_isFinished;
  private Pose2d m_robotPose;
  private double m_timeGoal;
  private State m_desiredState;

  private Timer timer;
   SwervePathController pathController;
   double lastTime;

  /** Creates a new AutonomousDrive. */
  public AutonomousDriveWeaver(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem); 
    m_driveSubsystem = driveSubsystem;
    m_goal = new Trajectory.State();
    m_isFinished = false;

    PIDController posController = new PIDController(SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_D);
    PIDController headingController = new PIDController(SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_D);
    ProfiledPIDController rotationController = new ProfiledPIDController(SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_D,
           new TrapezoidProfile.Constraints(SwerveDriveModuleConstants.kMaxAngularSpeed, SwerveDriveModuleConstants.k_MaxAcceleration));
     this.pathController = new SwervePathController(posController, headingController, rotationController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_trajectory = Robot.getTrajectory();
    m_timeGoal = m_trajectory.getTotalTimeSeconds();
    timer = new Timer();
    timer.reset();
    
    Pose2d initialPose = m_trajectory.getInitialPose();
    m_isFinished = false;

   // m_rotController.reset(new TrapezoidProfile.State(0,0)); //(0,0) our position and velocity
   // m_rotController.enableContinuousInput(-Math.PI, Math.PI);

    //m_driveController = new HolonomicDriveController(m_xController, m_yController, m_rotController);
    //m_driveSubsystem.resetOdometry(new Pose2d(m_driveSubsystem.getPoseMeters().getTranslation(), initialState.getRotation()));
    m_driveSubsystem.resetEncoders();
    // pathController.reset(m_driveSubsystem.getPoseMeters());

    //m_driveSubsystem.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    // m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
    timer.start(); 

  }
  
  @Override
  public void execute() {

     double time = timer.get();
     m_desiredState = m_trajectory.sample(time);
   

    if(time < m_timeGoal)
    {
/* TODO fix this bad code to use trajectory state in calculate function */
      ChassisSpeeds targetSpeeds = pathController.calculate(m_driveSubsystem.getPoseMeters(), m_desiredState, time - lastTime, timer.hasElapsed(0.1));
      m_driveSubsystem.drive(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, false);
  
      lastTime = time;
    }else
    {
      m_isFinished = true;
    }

    

    // ChassisSpeeds targetSpeeds = pathController.calculate(m_driveSubsystem.getPoseMeters(), desiredState, time - lastTime, timer.hasElapsed(0.1));
    // m_driveSubsystem.drive(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, false);

    // lastTime = time;

      // curX = m_robotPose.getX();
      // curY = m_robotPose.getY();
      // goalX = m_goal.poseMeters.getX();
      // goalY = m_goal.poseMeters.getY();

      // SmartDashboard.putNumber("Current X Position", curX);
      // SmartDashboard.putNumber("Current Y Position", curY);
      // SmartDashboard.putNumber("Goal X Position", goalX);
      // SmartDashboard.putNumber("Goal Y Position", goalY);
      // SmartDashboard.putNumber("Error X Position", goalX -  curX);
      // SmartDashboard.putNumber("Error Y Position", goalY -  curY);

      // SmartDashboard.putNumber("Goal Rotation", m_goal.poseMeters.getRotation().getDegrees());

      // SmartDashboard.putNumber("Goal Velocity", m_goal.velocityMetersPerSecond);
      // SmartDashboard.putNumber("Error Velocity", m_goal.velocityMetersPerSecond - m_driveSubsystem.getModules()[0].getDriveVelocity());

      // SmartDashboard.putNumber("timer", timer.get());
      // SmartDashboard.putNumber("trajectory time", m_trajectory.getTotalTimeSeconds());
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // timer.stop();
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
   // return timer.hasElapsed(path.getRuntime());

  }
}
