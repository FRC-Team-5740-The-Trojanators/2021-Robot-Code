// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.SwerveDriveModuleConstants;

import frc.robot.Constants.SwerveDriveModuleConstants.DriveModulePIDValues;
import frc.robot.Constants.SwerveDriveModuleConstants.SteeringControllerPIDValues;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.util.Units;



/** 
 * A Swerve Module consists of a drive motor, a steering motor, and encoders to provide feedback on the state of those motors.
 * This code provides accessors to those motors' controllers and encoders, as well as defining the feedback loops used to 
 *  enhance their control.
 */
public class SwerveModule
{
    // // CTRE SRX Magnetic Encoder signal; encoder is external and mounted on the swerve module
    private CANEncoder m_steeringEncoder;

    // PID Loops using the Spark API
    private CANPIDController m_driverPIDController;
    private PIDController m_steeringPIDController;

    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private CANCoder canCoder;
    private Rotation2d offset;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     */
    public SwerveModule(CANSparkMax driveMotor, CANSparkMax angleMotor, CANCoder canCoder, Rotation2d offset)
    {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;
        this.offset = offset;

        m_driverPIDController = driveMotor.getPIDController();
       // m_steeringPIDController = angleMotor.getPIDController();
       m_steeringEncoder = this.angleMotor.getEncoder();
       this.m_steeringEncoder = m_steeringEncoder;

       //Sets steering PID values using WPI version
        m_steeringPIDController = new PIDController(SteeringControllerPIDValues.k_steerP, 
                                                    SteeringControllerPIDValues.k_steerI,
                                                    SteeringControllerPIDValues.k_steerD);

        //Sets steering PID Values using Rev Robotics Version                                            
        m_steeringPIDController.setP(SteeringControllerPIDValues.k_steerP);
        m_steeringPIDController.setI(SteeringControllerPIDValues.k_steerI);
        m_steeringPIDController.setD(SteeringControllerPIDValues.k_steerD);
        m_steeringPIDController.setTolerance(SteeringControllerPIDValues.k_steerAllowedError);

        m_driverPIDController.setP(DriveModulePIDValues.k_driveP);
        m_driverPIDController.setI(DriveModulePIDValues.k_driveI);
        m_driverPIDController.setD(DriveModulePIDValues.k_driveD);
        m_driverPIDController.setFF(DriveModulePIDValues.k_driveFF);

        
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoder.configAllSettings(canCoderConfiguration);
    }


    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */

    // public SwerveModuleState getState()
    // {
    //     return new SwerveModuleState(m_driveEncoder.getVelocity(), // the getVelocity() has been scaled to go from RPM to m/s
    //                                  new Rotation2d(m_steeringEncoder.getPosition()));
    // }

  

    /**
     * 
    * @return Relative value of CANEncoder in radians
    */
    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(canCoder.getPosition());
    }

    public double getRawAngle()
    {
        return canCoder.getAbsolutePosition();
    }

    public double getSteeringEncoderValue()
    {
        return m_steeringEncoder.getPosition();
    }

    //TODO Move this to init so it's called less
    public void setSteerToCANCoderAbsolute()
    {
        if  (canCoder.getAbsolutePosition() != m_steeringEncoder.getPosition() )
        {
            m_steeringEncoder.setPosition(canCoder.getAbsolutePosition()); 
        }
    }

    double setpoint;

    /**
    * Sets the desired state for the module to drive the robot
    *
    * @param desiredState SwerveModule state with desired speed and angle.
    */
    public void setDesiredState(SwerveModuleState desiredState)
    {      
       //Steering Motor Calc
       //Using WPI PID Controller
        Rotation2d currentRotation = getAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        Rotation2d rotationDelta = state.angle.minus(currentRotation); //takes our current rotatation and subtracts the last state rotation

        double deltaTicks = (rotationDelta.getDegrees() / 360) * SwerveDriveModuleConstants.kEncoderTicksPerRotation;
        double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
        double desiredTicks = currentTicks + deltaTicks;
        angleMotor.set(m_steeringPIDController.calculate(currentTicks, desiredTicks));


        //Using Rev PID Controller
        // double steeringCurrentPos_degrees = ((m_steeringEncoder.getPosition() / m_steeringEncoder.getCountsPerRevolution())* 360) % 360; //converts the fraction of a rotation to degrees
        // Rotation2d currentRotation = Rotation2d.fromDegrees(steeringCurrentPos_degrees);
        // SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        // setpoint = steeringCurrentPos_degrees - state.angle.getDegrees();
        // m_steeringPIDController.setReference(setpoint, ControlType.kPosition);


       //Drive Motor Calc
        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
        m_driverPIDController.setReference(feetPerSecond / SwerveDriveModuleConstants.kMaxSpeed, ControlType.kVoltage);
    }

    public double getSetpoint()
    {
        return setpoint;
    }


    // /** Zeros all the SwerveModule encoders. */
    // public void resetEncoders()
    // {
    //     m_driveEncoder.setPosition(0);
    //     m_steeringEncoder.setPosition(0);
        
    // }
}
