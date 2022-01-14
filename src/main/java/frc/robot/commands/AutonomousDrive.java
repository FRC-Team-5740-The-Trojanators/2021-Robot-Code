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
import frc.robot.Constants.SwerveDriveModuleConstants.AutoChooser;
import frc.robot.pathsOLD.BarrelRacePath;
import frc.robot.pathsOLD.BlueAPath;
import frc.robot.pathsOLD.BlueBPath;
import frc.robot.pathsOLD.BouncePath;
import frc.robot.pathsOLD.BouncePath1;
import frc.robot.pathsOLD.BouncePath2;
import frc.robot.pathsOLD.BouncePath3;
import frc.robot.pathsOLD.BouncePath4;
import frc.robot.pathsOLD.FiveMeterPath;
import frc.robot.pathsOLD.RedAPath;
import frc.robot.pathsOLD.RedBPath;
import frc.robot.pathsOLD.SlalomPath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.IntakeSubsystem;
import lib.swerve.SwervePath;
import lib.swerve.SwervePathController;

public class AutonomousDrive extends CommandBase {

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
  private double curX;
  private double goalX;
  private double curY;
  private double goalY;

  private Timer timer;
  SwervePath path;
  SwervePathController pathController;
  double lastTime;
  boolean ignoreHeading;

  public void loadTrajectory()
  {}

  /** Creates a new AutonomousDrive. */
  public AutonomousDrive(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    m_driveSubsystem = driveSubsystem;
    //m_goal = new Trajectory.State(); :}
    m_isFinished = false;

    this.timer = new Timer();
    this.path = SwervePath.fromCSV("myPath");

    PIDController posController = new PIDController(SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_D);
    PIDController headingController = new PIDController(SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_D);
    ProfiledPIDController rotationController = new ProfiledPIDController(SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_D,
            new TrapezoidProfile.Constraints(SwerveDriveModuleConstants.kMaxAngularSpeed, SwerveDriveModuleConstants.k_MaxAcceleration));
    this.pathController = new SwervePathController(posController, headingController, rotationController);
    this.ignoreHeading = false;

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start(); 
    SwervePath.State initialState = path.getInitialState();
    //m_isFinished = false;

   // m_rotController.reset(new TrapezoidProfile.State(0,0)); //(0,0) are position and velocity
   // m_rotController.enableContinuousInput(-Math.PI, Math.PI);

    //m_driveController = new HolonomicDriveController(m_xController, m_yController, m_rotController);
    m_driveSubsystem.resetOdometry(new Pose2d(m_driveSubsystem.getPoseMeters().getTranslation(), initialState.getRotation()));
    m_driveSubsystem.resetEncoders();
   // pathController.reset(m_driveSubsystem.getPoseMeters());

    lastTime = 0;

   // m_driveSubsystem.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    //m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
  
    

  }
  
  @Override
  public void execute() {

    double time = timer.get();
    SwervePath.State desiredState = path.sample(time);

    if(ignoreHeading) desiredState.rotation = new Rotation2d(0);

   // ChassisSpeeds targetSpeeds = pathController.calculate(m_driveSubsystem.getPoseMeters(), desiredState, time - lastTime, timer.hasElapsed(0.1));
   // m_driveSubsystem.drive(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, false);

     m_driveSubsystem.drive(desiredState.getVelocity() * desiredState.getHeading().getCos(), desiredState.getVelocity() * desiredState.getHeading().getSin(), 0, false);
    // SmartDashboard.putNumber("Calc VelX", desiredState.getVelocity() * desiredState.getHeading().getCos());
    // SmartDashboard.putNumber("Calc VelY", desiredState.getVelocity() * desiredState.getHeading().getSin());

    lastTime = time;

      curX = m_driveSubsystem.getOdometry().getPoseMeters().getX();
      curY = m_driveSubsystem.getOdometry().getPoseMeters().getY();

      // goalX = m_goal.poseMeters.getX();
      // goalY = m_goal.poseMeters.getY();

       SmartDashboard.putNumber("AutoXPos", curX);
       SmartDashboard.putNumber("AutoYPos", curY);
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
    timer.stop();
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_isFinished;
    return timer.hasElapsed(path.getRuntime());

  }
}
