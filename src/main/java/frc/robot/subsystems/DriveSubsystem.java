// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.CANBusIDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//"Swerve Drive Tuning"
@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase 
{
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
    
    Trajectory m_trajectory; 

    // The gyro sensor
    public static final ADIS16470_IMU m_imu = new ADIS16470_IMU();
    private Pose2d m_Robotpose = new Pose2d();

    private SwerveModuleState[] m_states = new SwerveModuleState[]
    {
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0))
    };

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(SwerveDriveModuleConstants.kinematics, Rotation2d.fromDegrees(optimizeAngle()));

    public SwerveModule[] modules = new SwerveModule[]
     {
        new SwerveModule(new CANSparkMax(CANBusIDs.k_LeftFront_DriveMotor, MotorType.kBrushless), new CANSparkMax(CANBusIDs.k_LeftFront_SteeringMotor, MotorType.kBrushless), new CANCoder(CANBusIDs.frontLeftCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.frontLeftOffset)), // Front Left
        new SwerveModule(new CANSparkMax(CANBusIDs.k_RightFront_DriveMotor, MotorType.kBrushless), new CANSparkMax(CANBusIDs.k_RightFront_SteeringMotor, MotorType.kBrushless), new CANCoder(CANBusIDs.frontRightCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.frontRightOffset)), // Front Right
        new SwerveModule(new CANSparkMax(CANBusIDs.k_LeftRear_DriveMotor, MotorType.kBrushless), new CANSparkMax(CANBusIDs.k_LeftRear_SteeringMotor, MotorType.kBrushless), new CANCoder(CANBusIDs.backLeftCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.backLeftOffset)), // Back Left
        new SwerveModule(new CANSparkMax(CANBusIDs.k_RightRear_DriveMotor, MotorType.kBrushless), new CANSparkMax(CANBusIDs.k_RightRear_SteeringMotor, MotorType.kBrushless), new CANCoder(CANBusIDs.backRightCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.backRightOffset)) //Back Right
    };

    // /** Creates a new DriveSubsystem. */
    public DriveSubsystem( boolean calibrateGyro)
    {
            if(calibrateGyro)
            {
                m_imu.reset(); //recalibrates gyro offset
            }
        
        for(int i = 0; i < 4; i++)
        {
            modules[i].resetDriveEncoder();
            modules[i].setDriveP(SwerveDriveModuleConstants.DriveModulePIDValues.k_driveP[i]);
            modules[i].setDriveFF(SwerveDriveModuleConstants.DriveModulePIDValues.k_driveFF[i]);
            modules[i].setSteerP(SwerveDriveModuleConstants.SteeringControllerPIDValues.k_steerP[i]);
            modules[i].setSteerD(SwerveDriveModuleConstants.SteeringControllerPIDValues.k_steerD[i]);
        }
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
    {       
      m_states =
        SwerveDriveModuleConstants.kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(optimizeAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(m_states, SwerveDriveModuleConstants.k_MaxSpeed);
        for (int i = 0; i < m_states.length; i++) 
        {
            SwerveModule module = modules[i];
        //    SmartDashboard.putNumber(String.valueOf(i) + " Drive Velocity", module.getDriveVelocity());
            module.setDesiredState(m_states[i]);
        } 

    }


    
    public void resetEncoders(){
        modules[0].resetDriveEncoder();
        modules[1].resetDriveEncoder();
        modules[2].resetDriveEncoder();
        modules[3].resetDriveEncoder();
    }

    public SwerveModuleState[] getStates()
    {
        return m_states;
    }

    public double optimizeAngle()
    {
        double angle = m_imu.getAngle() % 360;
        if(angle > 180){
            angle = 360 - angle; 
        } else if(angle < -180){
            angle = 360 + angle;
        }
        return -angle;
    }

    public void resetIMU()
    {
        m_imu.reset();
    }

    @Override
    public void periodic()
    {
      //setSteerPthroughDashboard();
      //setSteerDthroughDashboard();
      // This method will be called once per scheduler run
      var gyroAngle = Rotation2d.fromDegrees(optimizeAngle());
      m_Robotpose = m_odometry.update(gyroAngle, modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
    }
  
    @Override
    public void simulationPeriodic() 
    {
      // This method will be called once per scheduler run during simulation
    }
    
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose()
    {
      return m_Robotpose;
    }

    public double getPoseX()
    {
      return m_Robotpose.getX();
    }

    public double getPoseY()
    {
      return m_Robotpose.getY();
    }


    public SwerveDriveOdometry getOdometry()
    {
        return m_odometry;
    }

    public SwerveModule[] getModules()
    {
        return modules;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose)
    {
        m_odometry.resetPosition(pose, m_imu.getRotation2d());
    }

    public void setSteerPthroughDashboard()
    {
        double P0 = SmartDashboard.getNumber("Steer P0 Input", 0);
        SmartDashboard.putNumber("Steer P0 Input", P0);
        double P1 = SmartDashboard.getNumber("Steer P1 Input", 0);
        SmartDashboard.putNumber("Steer P1 Input", P1);
        double P2 = SmartDashboard.getNumber("Steer P2 Input", 0);
        SmartDashboard.putNumber("Steer P2 Input", P2);
        double P3 = SmartDashboard.getNumber("Steer P3 Input", 0);
        SmartDashboard.putNumber("Steer P3 Input", P3);
        modules[0].setSteerP(P0);
        modules[1].setSteerP(P1);
        modules[2].setSteerP(P2);
        modules[3].setSteerP(P3);
    }
    public void setSteerDthroughDashboard()
    {
        double D0 = SmartDashboard.getNumber("Steer D0 Input", 0);
        SmartDashboard.putNumber("Steer D0 Input", D0);
        double D1 = SmartDashboard.getNumber("Steer D1 Input", 0);
        SmartDashboard.putNumber("Steer D1 Input", D1);
        double D2 = SmartDashboard.getNumber("Steer D2 Input", 0);
        SmartDashboard.putNumber("Steer D2 Input", D2);
        double D3 = SmartDashboard.getNumber("Steer D3 Input", 0);
        SmartDashboard.putNumber("Steer D3 Input", D3);
        modules[0].setSteerD(D0);
        modules[1].setSteerD(D1);
        modules[2].setSteerD(D2);
        modules[3].setSteerD(D3);
    }
    public Trajectory getTrajectory()
    {
        return m_trajectory;
    }

    public void setTrajectory(Trajectory trajectory)
    {
        m_trajectory = trajectory;
    }
}
