// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.SwerveDriveModuleConstants;



/** 
 * Swerve Module is based on the WPI Swerve module code.
 * 
 */
 public class SwerveModule
 {
     // REV robotics Spark Max motor controllers
     private final CANSparkMax m_driveMotor;
     private final CANSparkMax m_steeringMotor;

//     // REV Robotics Spark Max encoder signal
     private final CANEncoder m_driveEncoder;

//     // CTRE SRX Magnetic Encoder signal
     private final CANEncoder m_steeringEncoder;

     private final PIDController m_driverPIDController = new PIDController(SwerveDriveModuleConstants.DriveModule.k_Proportional, 
                                                                           SwerveDriveModuleConstants.DriveModule.k_Intergral,
                                                                           SwerveDriveModuleConstants.DriveModule.k_Differential);
                                                                          
    private final ProfiledPIDController m_turningPIDController = 
        new ProfiledPIDController(
            SwerveDriveModuleConstants.TurningController.k_Prop,
            SwerveDriveModuleConstants.TurningController.k_Inter,
            SwerveDriveModuleConstants.TurningController.k_Diff,
            new TrapezoidProfile.Constraints(
                SwerveDriveModuleConstants.k_MaxModuleAngularSpeedRadiansPerSecond,
                SwerveDriveModuleConstants.k_MaxModuleAngularAccelerationRadiansPerSecondSquared) );

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     */
  public SwerveModule(
        int driveMotorChannel,
        MotorType driveMotorType,
        int turningMotorChannel,
        MotorType turningMotorType,
        boolean driveEncoderReversed,
        boolean turningEncoderReversed)
        {
            m_driveMotor = new CANSparkMax(driveMotorChannel, driveMotorType);

            this.m_driveEncoder = m_driveMotor.getEncoder();   //assumes the use of the internal motor encoder
            m_driveEncoder.setVelocityConversionFactor(SwerveDriveModuleConstants.k_DriveEncoderDistancePerPulse); 

            m_steeringMotor = new CANSparkMax(turningMotorChannel, turningMotorType);
            m_steeringEncoder = m_steeringMotor.getAlternateEncoder(SwerveDriveModuleConstants.k_AlternateEncoderType, 
                                                                    SwerveDriveModuleConstants.k_AltEnc_CountPerRev);

             // Steering encoder is a Cross-The-Road Electronics CANCoder
             //this.m_steeringEncoder = new CANEncoder(steeringEncoderChannel); 
            
              CANCoderConfiguration m_canCoderConfiguration = new CANCoderConfiguration();
              m_canCoderConfiguration.unitString = "radians";
              m_canCoderConfiguration.sensorCoefficient = SwerveDriveModuleConstants.k_SteeringEncoderCoefficient;

              //m_steeringEncoder.configAllSettings(m_canCoderConfiguration);

            
            // Limit the PID Controller's input range between -pi and pi and set the input
            // to be continuous.
            m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
         }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState()
    {
        
        return new SwerveModuleState(m_driveEncoder.getVelocity(), // the getVelocity() has been scaled to go from RPM to m/s
                                     new Rotation2d(m_steeringEncoder.getPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state)
    {
        // Calculate the drive output from the drive PID controller.
        final var driveOutput = m_driverPIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final var steerOutput = m_turningPIDController.calculate(m_steeringEncoder.getPosition(), state.angle.getRadians());

        m_driveMotor.set(driveOutput);
        m_steeringMotor.set(steerOutput);
    }


    /** Zeros all the SwerveModule encoders. */
   public void resetEncoders()
    {
        m_driveEncoder.setPosition(0);
        m_steeringEncoder.setPosition(0);
        
    }

}
