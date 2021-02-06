// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.SwerveDriveModuleConstants;

//Swerve Module is based on the WPILib Swerve module example code.

/** 
 * A Swerve Module consists of a drive motor, a steering motor, and encoders to provide feedback on the state of those motors.
 * This code provides accessors to those motors' controllers and encoders, as well as defining the feedback loops used to 
 *  enhance their control.
 */
public class SwerveModule
{
    // REV robotics Spark Max motor controllers
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_steeringMotor;

    // REV Robotics Spark Max encoder signal -- internal to the NEO motor
    private final CANEncoder m_driveEncoder;

    // CTRE CANCoder encoder signal -- mounted on the MK3 swerve module
    private final CANCoder m_steeringEncoder;

    // Proportional-Integral-Differential controller used to enhance drive motor control
    private final PIDController m_driverPIDController = new PIDController(SwerveDriveModuleConstants.DriveModule.k_Proportional, 
                                                                          SwerveDriveModuleConstants.DriveModule.k_Intergral,
                                                                          SwerveDriveModuleConstants.DriveModule.k_Differential);
    
    // Proportional-Integral-Differential controller used to enhance steering motor control
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
     * @param driveMotorChannel CANBus ID for the driving motor
     * @param driveMotorType Driving motor motor-type (brushed or brushless)
     * @param turningMotorChannel CANBus ID for the steering motor
     * @param turningMotorType Steering motor motor-type (brushed or brushless)
     * @param steeringEncoderChannel CANBus ID for the encoder mounted on the Swerve Module
     * @param driveEncoderReversed Is the Drive Motor encoder reversed
     * @param turningEncoderReversed Is the Swerve Module encoder (steering encoder) reverse
     */
    public SwerveModule(
        int driveMotorChannel,
        MotorType driveMotorType,
        int turningMotorChannel,
        MotorType turningMotorType,
        int steeringEncoderChannel,
        boolean driveEncoderReversed,
        boolean turningEncoderReversed)
        {
            // instantiate the drive motor and steering motor contollers
            m_driveMotor = new CANSparkMax(driveMotorChannel, driveMotorType);
            m_steeringMotor = new CANSparkMax(turningMotorChannel, turningMotorType);

            // instantiage the drive motor encoder -- it's internal to the NEO motor
            m_driveEncoder = m_driveMotor.getEncoder();   //assumes the use of the internal motor encoder
            m_driveEncoder.setVelocityConversionFactor(SwerveDriveModuleConstants.k_DriveEncoderDistancePerPulse); 

            // instantiate the steering encoder -- it's a Cross-The-Road Electronics CANCoder
            m_steeringEncoder = new CANCoder(steeringEncoderChannel); 
            
            // Here we set the measurement unit as well as a human-readable string to describe the units
            CANCoderConfiguration m_canCoderConfiguration = new CANCoderConfiguration();
            m_canCoderConfiguration.unitString = "radians";
            m_canCoderConfiguration.sensorCoefficient = SwerveDriveModuleConstants.k_SteeringEncoderCoefficient;

            m_steeringEncoder.configAllSettings(m_canCoderConfiguration);

            
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


    /**
     * Zeros all the SwerveModule encoders.
     */
    public void resetEncoders()
    {
        m_driveEncoder.setPosition(0);
        m_steeringEncoder.setPosition(0);
    }
}
