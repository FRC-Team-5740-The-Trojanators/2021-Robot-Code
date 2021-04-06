// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.paths.BlueAPath;
import frc.robot.paths.BlueBPath;
import frc.robot.paths.BouncePath;
import frc.robot.paths.FiveMeterPath;
import frc.robot.paths.RedAPath;
import frc.robot.paths.RedBPath;
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
  private Pose2d m_robotPose;
  private double curX;
  private double goalX;
  private double curY;
  private double goalY;
  
  public void loadTrajectory()
  {
    //m_trajectory = FiveMeterPath.getTrajectory(); //change path name based on path we want to follow
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if( tx <= 12 && tx > 5)
    {
      m_trajectory = RedBPath.getTrajectory();
    }
    else if(tx <= 3 && tx > -1)
    {
      m_trajectory = RedAPath.getTrajectory();
    }
    else if(tx <= -2 && tx > -6)
    {
      m_trajectory = BlueAPath.getTrajectory();
    }
    else if(tx <= -12 && tx > -5)
    {
      m_trajectory = BlueBPath.getTrajectory();
    }
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

    m_xController = new PIDController(.2, 0, 0);
    m_yController = new PIDController(.2, 0, 0);
    m_trapezoidProfile = new TrapezoidProfile.Constraints(10, 20);
    m_rotController = new ProfiledPIDController(.02, 0, 0, m_trapezoidProfile);
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
    if (m_timer.get() <= m_trajectory.getTotalTimeSeconds())
    {
      m_goal = m_trajectory.sample(m_timer.get());
      var m_rotation = m_goal.poseMeters.getRotation();
      m_robotPose = m_driveSubsystem.getPose();

      ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
      
      m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);
    
    } else {
      m_goal = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1); // ensures last state gets executed
      var m_rotation = m_goal.poseMeters.getRotation(); 
      m_robotPose = m_driveSubsystem.getPose();
      
      ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
      
      m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, false);

      m_isFinished = true;
  }

      curX = m_robotPose.getX();
      curY = m_robotPose.getY();
      goalX = m_goal.poseMeters.getX();
      goalY = m_goal.poseMeters.getY();

      SmartDashboard.putNumber("Current X Position", curX);
      SmartDashboard.putNumber("Current Y Position", curY);
      SmartDashboard.putNumber("Goal X Position", goalX);
      SmartDashboard.putNumber("Goal Y Position", goalY);
      SmartDashboard.putNumber("Error X Position", goalX -  curX);
      SmartDashboard.putNumber("Error Y Position", goalY -  curY);

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
