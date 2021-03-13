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
   // private CANCoder m_steeringEncoder;
    private CANEncoder m_driveEncoder;

    // PID Loops using the Spark API
    private CANPIDController m_driverPIDController;
    private PIDController m_steeringPIDController;

    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    //CTRE SRX Magnetic Encoder signal; encoder is external and mounted on the swerve module
    private CANCoder m_moduleSteeringEncoder;
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
        this.m_moduleSteeringEncoder = canCoder;
        this.offset = offset;

        m_driverPIDController = driveMotor.getPIDController();
        m_driveEncoder = driveMotor.getEncoder(); 
       // m_steeringPIDController = angleMotor.getPIDController();

       //Sets steering PID values using WPI version
        m_steeringPIDController = new PIDController(SteeringControllerPIDValues.k_steerP, 
                                                    SteeringControllerPIDValues.k_steerI,
                                                    SteeringControllerPIDValues.k_steerD);

        //Sets steering PID Values using Rev Robotics Version                                            
        m_steeringPIDController.setP(SteeringControllerPIDValues.k_steerP);
        m_steeringPIDController.setI(SteeringControllerPIDValues.k_steerI);
        m_steeringPIDController.setD(SteeringControllerPIDValues.k_steerD);
        m_steeringPIDController.setTolerance(SteeringControllerPIDValues.k_ToleranceInTicks);

        m_driverPIDController.setP(DriveModulePIDValues.k_driveP);
        m_driverPIDController.setI(DriveModulePIDValues.k_driveI);
        m_driverPIDController.setD(DriveModulePIDValues.k_driveD);
        m_driverPIDController.setFF(DriveModulePIDValues.k_driveFF);

        m_driverPIDController.setOutputRange(-1, 1);
        
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoder.configAllSettings(canCoderConfiguration);

        m_driveEncoder.setVelocityConversionFactor(SwerveDriveModuleConstants.k_CANEncoderVelocityCoefficient);
        m_driveEncoder.setPositionConversionFactor(SwerveDriveModuleConstants.k_CANEncoderPositionCoefficient);
    }


    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    
    //TODO Properly scale the velocity
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(m_moduleSteeringEncoder.getPosition()));
    }

  

    /**
     * 
    * @return Relative value of CANEncoder in radians
    */
    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(m_moduleSteeringEncoder.getAbsolutePosition());
    }

    public double getRawAngle()
    {
        return m_moduleSteeringEncoder.getAbsolutePosition();
    }

    public double getSteeringEncoderValue()
    {
        return m_moduleSteeringEncoder.getPosition();
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
        double setAngle;
        Rotation2d currentRotation = getAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        Rotation2d rotationDelta = state.angle.minus(currentRotation); //takes our current rotatation and subtracts the last state rotation

        double deltaTicks = (rotationDelta.getDegrees() / 360) * SwerveDriveModuleConstants.kEncoderTicksPerRotation;
        double currentTicks = m_moduleSteeringEncoder.getPosition() / m_moduleSteeringEncoder.configGetFeedbackCoefficient();
        double desiredTicks = currentTicks + deltaTicks;
        setAngle = m_steeringPIDController.calculate(currentTicks, desiredTicks);

        if(Math.abs(setAngle) > SteeringControllerPIDValues.k_steerDeadband)
        {
          angleMotor.set(setAngle);
        } 
        else 
        {
            angleMotor.set(0);
        } 

        //Using Rev PID Controller
        // double steeringCurrentPos_degrees = ((m_steeringEncoder.getPosition() / m_steeringEncoder.getCountsPerRevolution())* 360) % 360; //converts the fraction of a rotation to degrees
        // Rotation2d currentRotation = Rotation2d.fromDegrees(steeringCurrentPos_degrees);
        // SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        // setpoint = steeringCurrentPos_degrees - state.angle.getDegrees();
        // m_steeringPIDController.setReference(setpoint, ControlType.kPosition);


       //Drive Motor Calc
        //double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
       // m_driverPIDController.setReference(state.speedMetersPerSecond / SwerveDriveModuleConstants.kMaxSpeed, ControlType.kDutyCycle);
       m_driverPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }



    public double getDriveMotor()
    {
        return driveMotor.get();
    }

    public double getDriveVelocity()
    {
        return m_driveEncoder.getVelocity();
    }


    public double getSetpoint()
    {
        return setpoint;
    }

    public double getSparkMaxPosition(){
        return m_driveEncoder.getPosition();
    }

    public double getRotationDegrees(){
        return m_moduleSteeringEncoder.getPosition();
    }

    public double getDrivePIDF(String parameter){
        if(parameter.toLowerCase() == "p")
        {
            return m_driverPIDController.getP();
        }else if(parameter.toLowerCase() == "i")
        {
            return m_driverPIDController.getI();
        }else if(parameter.toLowerCase() == "d")
        {
            return m_driverPIDController.getD();
        }else if(parameter.toLowerCase() == "ff")
        {
            return m_driverPIDController.getFF();
        }
        else
        {
            return -1;
        }

    }

    /** Zeros all the SwerveModule encoders. */
    public void resetDriveEncoder()
    {
        m_driveEncoder.setPosition(0);
        m_moduleSteeringEncoder.setPosition(0);
        
    }
}
