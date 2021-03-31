// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.paths.BarrelRacePath;
import frc.robot.paths.BouncePath;
import frc.robot.paths.FiveMeterPath;
import frc.robot.paths.SlalomPath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;

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
    m_trajectory = FiveMeterPath.getTrajectory(); //change path name based on path we want to follow
  }

  /** Creates a new AutonomousDrive. */
  public AutonomousDrive(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_goal = new Trajectory.State();
    m_isFinished = false;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loadTrajectory();
    m_isFinished = false;

    m_xController = new PIDController(.02, 0, 0);
    m_yController = new PIDController(.02, 0, 0);
    m_trapezoidProfile = new TrapezoidProfile.Constraints(10, 20);
    m_rotController = new ProfiledPIDController(.015, 0, 0, m_trapezoidProfile);
   // m_rotController.reset(new TrapezoidProfile.State(0,0)); //(0,0) are position and velocity
   // m_rotController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveController = new HolonomicDriveController(m_xController, m_yController, m_rotController);
    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();

   // m_driveSubsystem.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
    m_driveSubsystem.resetEncoders();
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
      
      m_pose2d = m_driveSubsystem.updateOdometry();
      m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);
    
    } else {
      m_goal = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1); // ensures last state gets executed
      var m_rotation = m_goal.poseMeters.getRotation(); 
      m_pose2d = m_driveSubsystem.getPose();
      
      ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_pose2d, m_goal, m_rotation);
      
      m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);
     
      m_driveSubsystem.getPose();

      m_isFinished = true;
  }

      SmartDashboard.putNumber("Current X Position", m_driveSubsystem.getPoseX());
      SmartDashboard.putNumber("Current Y Position", m_driveSubsystem.getPoseY());
      SmartDashboard.putNumber("Goal X Position", m_goal.poseMeters.getX());
      SmartDashboard.putNumber("Goal Y Position", m_goal.poseMeters.getY());
      SmartDashboard.putNumber("Error X Position", m_goal.poseMeters.getX() -  m_driveSubsystem.getPoseX());
      SmartDashboard.putNumber("Error Y Position", m_goal.poseMeters.getY() -  m_driveSubsystem.getPoseY());
      SmartDashboard.putNumber("Odometry X Position", m_pose2d.getX());
      SmartDashboard.putNumber("Odometry Y Position", m_pose2d.getY());

      SmartDashboard.putNumber("Goal Rotation", m_goal.poseMeters.getRotation().getDegrees());

      SmartDashboard.putNumber("Goal Velocity", m_goal.velocityMetersPerSecond);
      SmartDashboard.putNumber("Error Velocity", m_goal.velocityMetersPerSecond - m_driveSubsystem.getModules()[0].getDriveVelocity());

      SmartDashboard.putNumber("timer", m_timer.get());
      SmartDashboard.putNumber("trajectory time", m_trajectory.getTotalTimeSeconds());
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
