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
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.AutoChooser;
import frc.robot.paths.BarrelRacePath;
import frc.robot.paths.BlueAPath;
import frc.robot.paths.BlueBPath;
import frc.robot.paths.BouncePath;
import frc.robot.paths.BouncePath1;
import frc.robot.paths.BouncePath2;
import frc.robot.paths.BouncePath3;
import frc.robot.paths.BouncePath4;
import frc.robot.paths.FiveMeterPath;
import frc.robot.paths.RedAPath;
import frc.robot.paths.RedBPath;
import frc.robot.paths.SlalomPath;
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

  private boolean bounce1done;
  private boolean bounce2done;
  private boolean bounce3done;
  private boolean bounce4done;



  public void loadTrajectory()
  {
    m_trajectory = BouncePath.getTrajectory(); //change path name based on path we want to follow
    
   /*Galactic Search
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
  if( tx <= AutoChooser.k_RedB + AutoChooser.k_autoTolerance && tx > AutoChooser.k_RedB - AutoChooser.k_autoTolerance)
    {
      m_trajectory = RedBPath.getTrajectory();
    }
  else if(tx <= AutoChooser.k_RedA + AutoChooser.k_autoTolerance && tx > AutoChooser.k_RedA - AutoChooser.k_autoTolerance)
    {
      m_trajectory = RedAPath.getTrajectory();
    }
  else if(tx <= AutoChooser.k_BlueA + AutoChooser.k_autoTolerance && tx > AutoChooser.k_BlueA - AutoChooser.k_autoTolerance)
    {
      m_trajectory = BlueAPath.getTrajectory();
    }
  else if(tx <= AutoChooser.k_BlueB + AutoChooser.k_autoTolerance && tx > AutoChooser.k_BlueB - AutoChooser.k_autoTolerance)
    {
      m_trajectory = BlueBPath.getTrajectory();
    }*/
  }

  /** Creates a new AutonomousDrive. */
  // public AutonomousDrive(DriveSubsystem driveSubsystem, String pathname) {
  //    addRequirements(driveSubsystem);
  //   m_driveSubsystem = driveSubsystem;
  //   m_goal = new Trajectory.State();
  //   m_isFinished = false;

  //   this.timer = new Timer();
  //   this.path = SwervePath.fromCSV(pathname);

  //   PIDController posController = new PIDController(SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_POS_ERROR_CONTROLLER_D);
  //   PIDController headingController = new PIDController(SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_HEADING_ERROR_CONTROLLER_D);
  //   ProfiledPIDController rotationController = new ProfiledPIDController(SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_P, SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_I, SwerveDriveModuleConstants.DRIVE_ROTATION_CONTROLLER_D,
  //           new TrapezoidProfile.Constraints(SwerveDriveModuleConstants.kMaxAngularSpeed, SwerveDriveModuleConstants.k_MaxAcceleration));
  //   this.pathController = new SwervePathController(posController, headingController, rotationController);
  //   this.ignoreHeading = false;

  // }


  public AutonomousDrive(String pathname) {
    addRequirements(RobotContainer.m_robotDrive);
    this.timer = new Timer();
    this.path = SwervePath.fromCSV(pathname);
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
    //loadTrajectory();
    //m_isFinished = false;

    m_xController = new PIDController(.2, 0, 0);
    m_yController = new PIDController(.2, 0, 0);
    m_trapezoidProfile = new TrapezoidProfile.Constraints(SwerveDriveModuleConstants.k_MaxSpeed, SwerveDriveModuleConstants.k_MaxAcceleration);
    m_rotController = new ProfiledPIDController(.02, 0, 0, m_trapezoidProfile);
   // m_rotController.reset(new TrapezoidProfile.State(0,0)); //(0,0) are position and velocity
   // m_rotController.enableContinuousInput(-Math.PI, Math.PI);

    //m_driveController = new HolonomicDriveController(m_xController, m_yController, m_rotController);
    m_driveSubsystem.resetOdometry(new Pose2d(m_driveSubsystem.getPoseMeters().getTranslation(), initialState.getRotation()));
    m_driveSubsystem.resetEncoders();
    pathController.reset(m_driveSubsystem.getPoseMeters());

    lastTime = 0;

   // m_driveSubsystem.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    //m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
    

  }
  
//   public void BouncePath()
//   {
//       if (timer.get() <= m_trajectory.getTotalTimeSeconds())
//       {
//         bounce1done = false;

//         m_goal = m_trajectory.sample(timer.get());
//         var m_rotation = m_goal.poseMeters.getRotation();
//         m_robotPose = m_driveSubsystem.getPose();
  
//         ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
        
//         m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);
      
//       } else {
//         m_goal = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1); // ensures last state gets executed
//         var m_rotation = m_goal.poseMeters.getRotation(); 
//         m_robotPose = m_driveSubsystem.getPose();
        
//         ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
        
//         m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true);
  
//         System.out.println("Done First Path!");
//         bounce1done = true;

//         timer.reset();
//         m_trajectory = BouncePath2.getTrajectory();
  
//        // m_isFinished = true;
//   }
// }

// public void BouncePath2(){
//   if(timer.get() <= m_trajectory.getTotalTimeSeconds())
//   {
//     bounce2done = false;
//     m_goal = m_trajectory.sample(timer.get());
//     var m_rotation = m_goal.poseMeters.getRotation();
//     m_robotPose = m_driveSubsystem.getPose();

//     ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
    
//     m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);  
//   } else {
//       m_goal = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1); // ensures last state gets executed
//       var m_rotation = m_goal.poseMeters.getRotation(); 
//       m_robotPose = m_driveSubsystem.getPose();
      
//       ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
      
//       m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true);
//       System.out.println("Done Second Path!");
//       bounce2done = true;
//       timer.reset();
//       m_trajectory = BouncePath3.getTrajectory();

//     //  m_isFinished = true;
//   }
// }

//   public void BouncePath3(){
//     if(timer.get() <= m_trajectory.getTotalTimeSeconds())
//     {
//       bounce3done = false;
//       m_goal = m_trajectory.sample(timer.get());
//       var m_rotation = m_goal.poseMeters.getRotation();
//       m_robotPose = m_driveSubsystem.getPose();

//       ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
      
//       m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);  
//     } else {
//         m_goal = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1); // ensures last state gets executed
//         var m_rotation = m_goal.poseMeters.getRotation(); 
//         m_robotPose = m_driveSubsystem.getPose();
        
//         ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
        
//         m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true);
//         System.out.println("Done Third Path!");
         
//         bounce3done = true;
//         timer.reset();

//         m_trajectory = BouncePath4.getTrajectory();

//        // m_isFinished = true;
//     }

//   }

//   public void BouncePath4(){

//     if(timer.get() <= m_trajectory.getTotalTimeSeconds())
//     {
//       bounce4done = false;
//       m_goal = m_trajectory.sample(timer.get());
//       var m_rotation = m_goal.poseMeters.getRotation();
//       m_robotPose = m_driveSubsystem.getPose();

//       ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
      
//       m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);  
//     } else {
//         m_goal = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1); // ensures last state gets executed
//         var m_rotation = m_goal.poseMeters.getRotation(); 
//         m_robotPose = m_driveSubsystem.getPose();
        
//         ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
//         System.out.println("Done Fourth Path!");

//         bounce4done = true;
//         m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true);
//         m_isFinished = true;
//       }
//   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    // if(bounce1done == false){
    //   m_isFinished = false;
    //   BouncePath();
    // }else if(bounce1done == true && bounce2done == false){
    //   BouncePath2();
    // }else if(bounce2done == true && bounce3done == false){
    //   BouncePath3();
    // }else if(bounce3done == true && bounce4done == false){
    //   BouncePath4();
    // }else{
    //   m_isFinished = true;
    // }
    
    // if(m_timer.get() <= m_trajectory.getTotalTimeSeconds())
    // {
    //   m_goal = m_trajectory.sample(m_timer.get());
    //   var m_rotation = m_goal.poseMeters.getRotation();
    //   m_robotPose = m_driveSubsystem.getPose();
  
    //   ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
      
    //   m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond /* SwerveDriveModuleConstants.k_RobotRadius*/, false);  
    // } else {
    //     m_goal = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1); // ensures last state gets executed
    //     var m_rotation = m_goal.poseMeters.getRotation(); 
    //     m_robotPose = m_driveSubsystem.getPose();
        
    //     ChassisSpeeds adjustedSpeeds = m_driveController.calculate(m_robotPose, m_goal, m_rotation);
        
    //     m_driveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true);
    //      m_isFinished = true;
    // }

    double time = timer.get();
    SwervePath.State desiredState = path.sample(time);

    if(ignoreHeading) desiredState.rotation = new Rotation2d(0);

    ChassisSpeeds targetSpeeds = pathController.calculate(m_driveSubsystem.getPoseMeters(), desiredState, time - lastTime, timer.hasElapsed(0.1));
    m_driveSubsystem.drive(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, false);

    lastTime = time;

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

      SmartDashboard.putNumber("timer", timer.get());
      SmartDashboard.putNumber("trajectory time", m_trajectory.getTotalTimeSeconds());
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
