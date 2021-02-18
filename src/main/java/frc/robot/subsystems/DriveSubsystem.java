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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants;

import frc.robot.subsystems.SwerveModule;

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
            SwerveDriveModuleConstants.CANBusIDs.k_RightRear_DriveMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightRear_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_RightRear_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightRear_Steering,
            SwerveDriveModuleConstants.k_RightRearSteeringEncoderReversed,
            SwerveDriveModuleConstants.k_RightRearSteeringEncoderReversed);

    public double kP_Drive, kI_Drive, kD_Drive, kIz_Drive, kFF_Drive, kMaxOutput_Drive, kMinOutput_Drive, maxRPM_Drive, maxVel_Drive, minVel_Drive, maxAcc_Drive, allowedErr_Drive;

    public double kP_Steer, kI_Steer, kD_Steer, kIz_Steer, kFF_Steer, kMaxOutput_Steer, kMinOutput_Steer, maxRPM_Steer, maxVel_Steer, minVel_Steer, maxAcc_Steer, allowedErr_Steer;
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
        kP_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_Prop;
        kI_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_Inter;
        kD_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_Diff;
        kIz_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_Iz;
        kFF_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_FF;
        kMaxOutput_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_MaxOutput;
        kMinOutput_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_MinOutput;
        maxRPM_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_maxRPM;
        maxVel_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_maxVel;
        minVel_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_minVel;
        maxAcc_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_maxAcc;
        allowedErr_Steer = SwerveDriveModuleConstants.SteeringControllerPIDValues.k_allowedError;

        kP_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_Proportional;
        kI_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_Intergral;
        kD_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_Differential;
        kIz_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_Iz;
        kFF_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_FF;
        kMaxOutput_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_MaxOutput;
        kMinOutput_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_MinOutput;
        maxRPM_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_maxRPM;
        maxVel_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_maxVel;
        minVel_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_minVel;
        maxAcc_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_maxAcc;
        allowedErr_Drive = SwerveDriveModuleConstants.DriveModulePIDValues.k_allowedError;
        
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Steer P Gain", kP_Steer);
        SmartDashboard.putNumber("Steer I Gain", kI_Steer);
        SmartDashboard.putNumber("Steer D Gain", kD_Steer);
        SmartDashboard.putNumber("kIz_Steer Zone", kIz_Steer);
        SmartDashboard.putNumber("kFF_Steer Feed Forward", kFF_Steer);
        SmartDashboard.putNumber("kMaxOutput_Steer Max Output", kMaxOutput_Steer);
        SmartDashboard.putNumber("kMinOutput_Steer Output", kMinOutput_Steer);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("maxVel_Steer Max Velocity", maxVel_Steer);
        SmartDashboard.putNumber("minVel_Steer Min Velocity", minVel_Steer);
        SmartDashboard.putNumber("maxAcc_Steer Acceleration", maxAcc_Steer);
        SmartDashboard.putNumber("allowedErr_Drive Allowed Closed Loop Error", allowedErr_Steer);
        // SmartDashboard.putNumber("Set Position Steer", 0);
        // SmartDashboard.putNumber("Set Velocity Steer", 0);
        
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Drive P Gain", kP_Drive);
        SmartDashboard.putNumber("Drive I Gain", kI_Drive);
        SmartDashboard.putNumber("kD_Drive Gain", kD_Drive);
        SmartDashboard.putNumber("kIz_Drive Zone", kIz_Drive);
        SmartDashboard.putNumber("kFF_Drive Feed Forward", kFF_Drive);
        SmartDashboard.putNumber("kMaxOutput_Drive Max Output", kMaxOutput_Drive);
        SmartDashboard.putNumber("kMinOutput_Drive Output", kMinOutput_Drive);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("maxVel_Drive Max Velocity", maxVel_Drive);
        SmartDashboard.putNumber("minVel_Drive Min Velocity", minVel_Drive);
        SmartDashboard.putNumber("maxAcc_Drive Acceleration", maxAcc_Drive);
        SmartDashboard.putNumber("allowedErr_Drive Allowed Closed Loop Error", allowedErr_Drive);
        // SmartDashboard.putNumber("Set Position Drive", 0);
        // SmartDashboard.putNumber("Set Velocity Drive", 0);

        // button to toggle between velocity and smart motion modes
        SmartDashboard.putBoolean("Mode", true);
    }

    @Override
    public void periodic()
    {
        SteeringUpdate();

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

    public void SteeringUpdate()
    {
        // read PID coefficients from SmartDashboard
        double local_kP_Steer = SmartDashboard.getNumber("Steer P Gain", 0);
        double local_kP_Drive = SmartDashboard.getNumber("Drive P Gain", 0);

        double local_kI_Steer = SmartDashboard.getNumber("Steer I Gain", 0);
        double local_kI_Drive = SmartDashboard.getNumber("Drive I Gain", 0);

        double local_kD_Steer = SmartDashboard.getNumber("Steer D Gain", 0);
        double local_kD_Drive = SmartDashboard.getNumber("kD_Drive Gain", 0);

        double local_kIz_Steer = SmartDashboard.getNumber("kIz_Steer Zone", 0);
        double local_kIz_Drive = SmartDashboard.getNumber("kIz_Drive Zone", 0);

        double local_kFF_Steer = SmartDashboard.getNumber("kFF_Steer Feed Forward", 0);
        double local_kFF_Drive = SmartDashboard.getNumber("kFF_Drive Feed Forward", 0);

        double local_kMaxOutput_Steer = SmartDashboard.getNumber("kMaxOutput_Steer Max Output", 0);
        double local_kMaxOutput_Drive = SmartDashboard.getNumber("kMaxOutput_Drive Max Output", 0);

        double local_kMinOutput_Steer = SmartDashboard.getNumber("kMinOutput_Steer Output", 0);
        double local_kMinOutput_Drive = SmartDashboard.getNumber("kMinOutput_Drive Output", 0);

        double local_maxVel_Steer = SmartDashboard.getNumber("maxVel_Steer Max Velocity", 0);
        double local_maxVel_Drive = SmartDashboard.getNumber("maxVel_Drive Max Velocity", 0);

        double local_minVel_Steer = SmartDashboard.getNumber("minVel_Steer Min Velocity", 0);
        double local_minVel_Drive = SmartDashboard.getNumber("minVel_Drive Min Velocity", 0);

        double local_maxAcc_Steer = SmartDashboard.getNumber("maxAcc_Steer Acceleration", 0);
        double local_maxAcc_Drive = SmartDashboard.getNumber("maxAcc_Drive Acceleration", 0);

        double local_allowedErr_Steer = SmartDashboard.getNumber("allowedErr_Drive Allowed Closed Loop Error", 0);
        double local_allowedErr_Drive = SmartDashboard.getNumber("allowedErr_Drive Allowed Closed Loop Error", 0);




        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if (kP_Steer != local_kP_Steer) 
        {  
            m_LeftFrontModule.setSteerMotor_P(local_kP_Steer);
            m_RightFrontModule.setSteerMotor_P(local_kP_Steer);
            m_LeftRearModule.setSteerMotor_P(local_kP_Steer);
            m_RightRearModule.setSteerMotor_P(local_kP_Steer);

            kP_Steer = local_kP_Steer; 
        }

        if (kP_Drive != local_kP_Drive)
        {
            m_LeftFrontModule.setDriveMotor_P(local_kP_Drive);
            m_RightFrontModule.setDriveMotor_P(local_kP_Drive);
            m_LeftRearModule.setDriveMotor_P(local_kP_Drive);
            m_RightRearModule.setDriveMotor_P(local_kP_Drive);
        }



        if((i !=  kI_S)) {  m_steeringPIDController.setI(i);  kI_S = i; }
        if((d !=  kD_S)) {  m_steeringPIDController.setD(d);  kD_S = d; }
        if((iz !=  kIz_S)) {  m_steeringPIDController.setIZone(iz);  kIz_S = iz; }
        if((ff !=  kFF_S)) {  m_steeringPIDController.setFF(ff);  kFF_S = ff; }
        if((max !=  kMaxOutput_S) || (min !=  kMinOutput_S)) { 
         m_steeringPIDController.setOutputRange(min, max); 
         kMinOutput_S = min;  kMaxOutput_S = max; 
        }
        if((maxV !=  maxVel_S)) {  m_steeringPIDController.setSmartMotionMaxVelocity(maxV,0);  maxVel_S = maxV; }
        if((minV !=  minVel_S)) {  m_steeringPIDController.setSmartMotionMinOutputVelocity(minV,0);  minVel_S = minV; }
        if((maxA !=  maxAcc_S)) {  m_steeringPIDController.setSmartMotionMaxAccel(maxA,0);  maxAcc_S = maxA; }
        if((allE !=  allowedErr_S)) {  m_steeringPIDController.setSmartMotionAllowedClosedLoopError(allE,0);  allowedErr_S = allE; }
 
        double setPoint, processVariable;
        boolean mode = SmartDashboard.getBoolean("Mode", false);
        if(mode) {
        setPoint = SmartDashboard.getNumber("Set Velocity", 0);
         m_steeringPIDController.setReference(setPoint, ControlType.kVelocity);
        processVariable =  m_steeringEncoder.getVelocity();
        } else {
        setPoint = SmartDashboard.getNumber("Set Position", 0);
        /**
         * As with other PID modes, Smart Motion is set by calling the
         * setReference method on an existing pid object and setting
         * the control type to kSmartMotion
         */
         m_steeringPIDController.setReference(setPoint, ControlType.kSmartMotion);
        processVariable =  m_steeringEncoder.getPosition();
 
        //SwerveModule.ConfigureSteeringMotor(0, SwerveModule.m_steeringMotor);
 
        }
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("Process Variable", processVariable);
        SmartDashboard.putNumber("Output", m_steeringMotor.getAppliedOutput());
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
