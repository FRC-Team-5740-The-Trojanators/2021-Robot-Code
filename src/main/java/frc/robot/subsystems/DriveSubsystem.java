// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
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

//Below are the imports needed from the MK3 File
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



//"Swerve Drive Tuning"

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase 
{
    // The gyro sensor
    public static final ADIS16470_IMU m_imu = new ADIS16470_IMU();
    private Pose2d m_pose = new Pose2d();

    private SwerveModuleState[] m_states = new SwerveModuleState[4];



    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(SwerveDriveModuleConstants.kinematics, new Rotation2d(m_imu.getAngle())); // TODO: use radians here?

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
        //modules[0].setSteerToCANCoderAbsolute();
        //modules[1].setSteerToCANCoderAbsolute();
        //modules[2].setSteerToCANCoderAbsolute();
        //modules[3].setSteerToCANCoderAbsolute();

        //if(calibrateGyro)
        //{
        //    modules[0].setSteerToCANCoderAbsolute();
        //    modules[1].setSteerToCANCoderAbsolute();
        //    modules[2].setSteerToCANCoderAbsolute();
        //    modules[3].setSteerToCANCoderAbsolute();

            if(calibrateGyro)
            {
                m_imu.reset(); //recalibrates gyro offset
            }
        //}
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
    {       
      m_states =
        SwerveDriveModuleConstants.kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_imu.getAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(m_states, SwerveDriveModuleConstants.kMaxSpeed);
        for (int i = 0; i < m_states.length; i++) 
        {
            SwerveModule module = modules[i];
            SmartDashboard.putNumber(String.valueOf(i) + "Drive Velocity", module.getDriveMotor());
            //SmartDashboard.putNumber(String.valueOf(i) + "Steer Encoder", module.getSteeringEncoderValue());
            //below is a line to comment out from step 5
            module.setDesiredState(m_states[i]);
            //SmartDashboard.putNumber("gyro Angle", m_imu.getAngle());
        } 

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
      var gyroAngle = Rotation2d.fromDegrees(-m_imu.getAngle());

      //m_pose = m_odometry.update(gyroAngle, modules[0].)
      
    }
  
    @Override
    public void simulationPeriodic() 
    {
      // This method will be called once per scheduler run during simulation
    }

    //Everything after this was not used in the example file
//     @Override
//     public void periodic()
//     {
//     }
    
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose()
    {
      return m_odometry.update(Rotation2d.fromDegrees(-m_imu.getAngle()), m_states[0], m_states[1], m_states[2], m_states[3]);
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

//    public Pose2d updateOdometry(){
//        return m_odometry.update(m_imu.getAngle(), /*get module states here*/);
//     }


//     /** 
//      * Zeroes the heading of the robot. 
//      */
//     public void zeroHeading()
//     {
//         //m_gyro.reset();
//         m_imu.reset();
//     }
}
