// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveModuleConstants;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase 
{        
    private final SwerveModule m_LeftFrontModule = 
        new SwerveModule(
            SwerveDriveModuleConstants.CANBusIDs.k_LeftFront_DriveMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveLeftFront_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_LeftFront_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveLeftFront_Steering,
            SwerveDriveModuleConstants.k_LeftFrontSteeringEncoderReversed,
            SwerveDriveModuleConstants.k_LeftFrontSteeringEncoderReversed);


   private final SwerveModule m_RightFrontModule = 
        new SwerveModule(
            SwerveDriveModuleConstants.CANBusIDs.k_RightFront_DriveMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightFront_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_RightFront_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightFront_Steering,
            SwerveDriveModuleConstants.k_RightFrontSteeringEncoderReversed,
            SwerveDriveModuleConstants.k_RightFrontSteeringEncoderReversed);

    private final SwerveModule m_LeftRearModule = 
        new SwerveModule(
            SwerveDriveModuleConstants.CANBusIDs.k_LeftRear_DriveMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveLeftRear_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_LeftRear_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveLeftRear_Steering,
            SwerveDriveModuleConstants.k_LeftRearSteeringEncoderReversed,
            SwerveDriveModuleConstants.k_LeftRearSteeringEncoderReversed);

    private final SwerveModule m_RightRearModule = 
        new SwerveModule(
            SwerveDriveModuleConstants.CANBusIDs.k_RightRear_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightRear_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_RightRear_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightRear_Steering,
            SwerveDriveModuleConstants.k_RightRearSteeringEncoderReversed,
            SwerveDriveModuleConstants.k_RightRearSteeringEncoderReversed);

    // The gyro sensor
     //private final Gyro m_gyro = new ADXRS450_Gyro();
    public static final ADIS16470_IMU m_imu = new ADIS16470_IMU();

    // Odometry class for tracking robot pose
   SwerveDriveOdometry m_odometry =
       new SwerveDriveOdometry(SwerveDriveModuleConstants.k_DriveKinematics, new Rotation2d(m_imu.getAngle())); // TODO: use radians here?

    // SwerveDriveOdometry m_Odometry_TEMP = new SwerveDriveOdometry(SwerveDriveModuleConstants.k_DriveKinematics_ONEWHEEL,
    //                                                  new Rotation2d(0.0, 0.0));

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem()
    {
    }

    @Override
    public void periodic()
    {
        // Update the odeometry in the periodic block
        // (Please provide the states in the same order in which you instantiated your 
        // SwerveDriveKinematics.)
    //    m_odometry.update(new Rotation2d(getHeading()),
    //         m_LeftFrontModule.getState(),
    //         m_RightFrontModule.getState(),
    //         m_LeftRearModule.getState(),
    //         m_RightRearModule.getState());


    //    m_Odometry_TEMP.update(new Rotation2d(0.0), m_LeftFrontModule.getState());
    }


    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    // public Pose2d getPose()
    // {
    //    // return m_odometry.getPoseMeters();
    // }


    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose)
    {
        m_odometry.resetPosition(pose, m_imu.getRotation2d());
    }


    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
    {
       var swerveModuleStates =
            SwerveDriveModuleConstants.k_DriveKinematics.toSwerveModuleStates(
               // fieldRelative
                   // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    new ChassisSpeeds(xSpeed, ySpeed, rot));
        
        SwerveDriveKinematics.normalizeWheelSpeeds(
            swerveModuleStates, SwerveDriveModuleConstants.k_MaxSpeedMetersPerSecond);

        m_LeftFrontModule.setDesiredState(swerveModuleStates[0]);
        m_RightFrontModule.setDesiredState(swerveModuleStates[1]);
        m_LeftRearModule.setDesiredState(swerveModuleStates[2]);
        m_RightRearModule.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.normalizeWheelSpeeds(
            desiredStates, SwerveDriveModuleConstants.k_MaxSpeedMetersPerSecond);

        m_LeftFrontModule.setDesiredState(desiredStates[0]);
        m_RightFrontModule.setDesiredState(desiredStates[1]);
        m_LeftRearModule.setDesiredState(desiredStates[2]);
        m_RightRearModule.setDesiredState(desiredStates[3]);
    }

    /** 
     * Resets the drive encoders to currently read a position of 0. 
     */
    public void resetEncoders()
    {
        m_LeftFrontModule.resetEncoders();
        m_RightFrontModule.resetEncoders();
        m_LeftRearModule.resetEncoders();
        m_RightRearModule.resetEncoders();
    }


    /** 
     * Zeroes the heading of the robot. 
     */
    public void zeroHeading()
    {
        //m_gyro.reset();
        m_imu.reset();
    }


    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180 // TODO: Is this still correct, that it's -180 to +180?
     */
    public double getHeading()
    {
        //return m_gyro.getRotation2d().getDegrees();
        return m_imu.getAngle();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate()
    {
        //return m_gyro.getRate() * (SwerveDriveModuleConstants.k_GyroReversed ? -1.0 : 1.0);
       return m_imu.getRate() * (SwerveDriveModuleConstants.k_GyroReversed ? -1.0 : 1.0);
    }
}
