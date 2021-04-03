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
import frc.robot.subsystems.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//"Swerve Drive Tuning"

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase 
{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    // The gyro sensor
    public static final ADIS16470_IMU m_imu = new ADIS16470_IMU();
    private Pose2d m_pose = new Pose2d();

    private SwerveModuleState[] m_states = new SwerveModuleState[]
    {
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0))
    };

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(SwerveDriveModuleConstants.kinematics, Rotation2d.fromDegrees(m_imu.getAngle()));

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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_imu.getAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(m_states, SwerveDriveModuleConstants.k_MaxSpeed);
        for (int i = 0; i < m_states.length; i++) 
        {
            SwerveModule module = modules[i];
            SmartDashboard.putNumber(String.valueOf(i) + " Drive Velocity", module.getDriveVelocity());
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

    public void resetIMU()
    {
        m_imu.reset();
    }

    @Override
    public void periodic()
    {
      // This method will be called once per scheduler run
      var gyroAngle = Rotation2d.fromDegrees(m_imu.getAngle());

     m_odometry.update(gyroAngle, modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());

     var x = tx.getDouble(0.0);

     SmartDashboard.putNumber("Gryo Angle", m_imu.getAngle());

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
    
    return m_odometry.getPoseMeters();

    }

    public double getPoseX(){

        return m_odometry.getPoseMeters().getX();
    }

    public double getPoseY(){

        return m_odometry.getPoseMeters().getY();
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

   public Pose2d updateOdometry()
   {
    var gyroAngle = Rotation2d.fromDegrees(m_imu.getAngle());
    return m_odometry.update(gyroAngle, modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
   }
}
