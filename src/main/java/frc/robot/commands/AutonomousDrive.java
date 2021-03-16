// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.paths.TrajectoryMaker;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDrive extends CommandBase {

  private Trajectory m_trajectory; 

  private final DriveSubsystem m_driveSubsystem;

  private HolonomicDriveController m_driveController; 

  private PIDController m_xController;
  private PIDController m_yController;
  private ProfiledPIDController m_rotController; 
  private TrapezoidProfile.Constraints m_trapezoidProfile;
  private Trajectory.State m_goal;
  private Timer m_timer;
  private Boolean m_isFinished;
  private Pose2d m_pose2d;
  
  public void loadTrajectory()
  {
   // String trajectoryJSON = "output/StraightLine5.wpilib.json"; //Change this file name to change the path
    m_trajectory = TrajectoryMaker.MakeATrajectory();
    // try
    // {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // }
    // catch (IOException ex)
    // {
    //   DriverStation.reportError("Unable to open trajectory File: " + trajectoryJSON, ex.getStackTrace());
    // }
  }

  public void trajectoryGenerator()
  {
    TrajectoryConfig config = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.kMaxSpeed, 1).setKinematics(Constants.SwerveDriveModuleConstants.kinematics);
  
    m_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)), List.of(new Translation2d(1, 0), new Translation2d(2, 0)), new Pose2d(3, 0, new Rotation2d(0)), config);
    
  }
   

  /** Creates a new AutonomousDrive. */
  public AutonomousDrive(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_goal = new Trajectory.State();
    //m_trajectory = new Trajectory();
    m_isFinished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //loadTrajectory();
    trajectoryGenerator();
    //m_driveSubsystem.resetIMU();
    m_isFinished = false;

    m_xController = new PIDController(.01, 0, 0);
    m_yController = new PIDController(.01, 0, 0);
    m_trapezoidProfile = new TrapezoidProfile.Constraints(Math.PI, 2 * Math.PI);
    m_rotController = new ProfiledPIDController(.002, 0, 0, m_trapezoidProfile);

    m_driveController = new HolonomicDriveController(m_xController, m_yController, m_rotController);
    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();

   // m_driveSubsystem.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());

  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() < m_trajectory.getTotalTimeSeconds())
    {
      m_goal = m_trajectory.sample(m_timer.get());
      var m_rotation = m_goal.poseMeters.getRotation();
      m_pose2d = m_driveSubsystem.getPose();

      ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_pose2d, m_goal, m_rotation);
      
      //m_pose2d = m_driveSubsystem.updateOdometry();
      m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);
      SmartDashboard.putNumber("X Velocity", adjustedSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Y Velocity", adjustedSpeeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Rot Speed", adjustedSpeeds.omegaRadiansPerSecond);
      SmartDashboard.putNumber("Current X Position", m_driveSubsystem.getPoseX());
      SmartDashboard.putNumber("Current Y Position", m_driveSubsystem.getPoseY());
      SmartDashboard.putNumber("m_rotation", m_rotation.getDegrees());

      SmartDashboard.putNumber("Reading X Velocity LeftFront", m_driveSubsystem.getModules()[0].getDriveVelocity());
      SmartDashboard.putNumber("Reading X Velocity RightFront", m_driveSubsystem.getModules()[1].getDriveVelocity());
      SmartDashboard.putNumber("Reading X Velocity LeftRear", m_driveSubsystem.getModules()[2].getDriveVelocity());
      SmartDashboard.putNumber("Reading X Velocity RightRear", m_driveSubsystem.getModules()[3].getDriveVelocity());

      SmartDashboard.putNumber("timer", m_timer.get());
      SmartDashboard.putNumber("trajectory time", m_trajectory.getTotalTimeSeconds());
     
    } else {
      m_goal = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1); // ensures last state gets executed
      var m_rotation = m_goal.poseMeters.getRotation(); 
      m_pose2d = m_driveSubsystem.getPose();
      
      ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_pose2d, m_goal, m_rotation);
      
      m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);
      
      SmartDashboard.putNumber("timer", m_timer.get());
      SmartDashboard.putNumber("trajectory time", m_trajectory.getTotalTimeSeconds());
     
      m_driveSubsystem.getPose();
      
      SmartDashboard.putNumber("Current X Position", m_driveSubsystem.getPoseX());
      SmartDashboard.putNumber("Current Y Position", m_driveSubsystem.getPoseY());
      SmartDashboard.putNumber("m_rotation", m_rotation.getDegrees());
      //isFinished();
      m_isFinished = true;
  }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_isFinished = true;
    return m_isFinished;
  }
}
