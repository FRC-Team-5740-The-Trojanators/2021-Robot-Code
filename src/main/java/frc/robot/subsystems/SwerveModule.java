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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveDriveModuleConstants;

import frc.robot.Constants.SwerveDriveModuleConstants.DriveModulePIDValues;
import frc.robot.Constants.SwerveDriveModuleConstants.SteeringControllerPIDValues;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;


/**
 * A Swerve Module consists of a drive motor, a steering motor, and encoders to
 * provide feedback on the state of those motors. This code provides accessors
 * to those motors' controllers and encoders, as well as defining the feedback
 * loops used to enhance their control.
 */
public class SwerveModule
{
    private CANEncoder m_driveEncoder;

    // PID Loops using the Spark API
    private CANPIDController m_driverPIDController;
    private PIDController m_steeringPIDController;

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_angleMotor;
    private CANCoder m_moduleSteeringEncoder;
    private Rotation2d m_offset;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     */
    public SwerveModule(CANSparkMax driveMotor, CANSparkMax angleMotor, CANCoder canCoder, Rotation2d offset)
    {
        m_driveMotor = driveMotor;
        m_angleMotor = angleMotor;
        m_moduleSteeringEncoder = canCoder;
        m_offset = offset;

        m_driverPIDController = driveMotor.getPIDController();
        m_driveEncoder = driveMotor.getEncoder(); 

       //Sets steering PID values using WPI version
        m_steeringPIDController = new PIDController(0, 0, 0);

        m_steeringPIDController.setTolerance(SteeringControllerPIDValues.k_ToleranceInTicks);

        m_driverPIDController.setP(DriveModulePIDValues.k_driveP);
        m_driverPIDController.setI(DriveModulePIDValues.k_driveI);
        m_driverPIDController.setD(DriveModulePIDValues.k_driveD);
        m_driverPIDController.setFF(DriveModulePIDValues.k_driveFF);

        SmartDashboard.putNumber("P Gain", DriveModulePIDValues.k_driveP);
        SmartDashboard.putNumber("I Gain", DriveModulePIDValues.k_driveI);
        SmartDashboard.putNumber("D Gain", DriveModulePIDValues.k_driveD);
        SmartDashboard.putNumber("FF Gain", DriveModulePIDValues.k_driveFF);

        m_driverPIDController.setOutputRange(-1, 1);
        
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = m_offset.getDegrees();
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
        Rotation2d currentRotation = getAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        Rotation2d rotationDelta = state.angle.minus(currentRotation); //takes our current rotatation and subtracts the last state rotation
       
        double deltaTicks = calculateDeltaTicks(rotationDelta);
        double currentTicks = calculateCurrentTicks();
        double desiredTicks = currentTicks + deltaTicks;
        double setAngle = m_steeringPIDController.calculate(currentTicks, desiredTicks);

        m_angleMotor.set(filterAngleMotorDeadband(setAngle));
        
        m_driverPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        SmartDashboard.putNumber("Drive Encoder", m_driveEncoder.getPosition());
        //SmartDashboard.putNumber("Heading Read", m_moduleSteeringEncoder.getAbsolutePosition());

    }

    public double calculateCurrentTicks() {
        double currentTicks = m_moduleSteeringEncoder.getPosition() / m_moduleSteeringEncoder.configGetFeedbackCoefficient();
        return currentTicks;
    }



    public double calculateDeltaTicks(Rotation2d rotationDelta) 
    {
        return (rotationDelta.getDegrees() / 360) * SwerveDriveModuleConstants.kEncoderTicksPerRotation;
    }

    public double filterAngleMotorDeadband(double setAngle)
    {
        if(Math.abs(setAngle) > SteeringControllerPIDValues.k_steerDeadband)
        {
           return setAngle;
        } 
        return 0.0;
    }

    public void setDriveP(double value)
    {
        m_driverPIDController.setP(value);
    }

    public void setDriveI(double value)
    {
        m_driverPIDController.setI(value);
    }

    public void setDriveD(double value)
    {
        m_driverPIDController.setD(value);
    }

    public void setDriveFF(double value)
    {
        m_driverPIDController.setFF(value);
    }

    public double getDriveMotor()
    {
        return m_driveMotor.get();
    }

    public double getDriveVelocity()
    {
        return m_driveEncoder.getVelocity();
    }


    public void setSteerP(double value)
    {
        m_steeringPIDController.setP(value);
    }
    
    public void setSteerD(double value)
    {
        m_steeringPIDController.setD(value);
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

    public double getAbsoluteRotationDegrees(){
        return m_moduleSteeringEncoder.getAbsolutePosition();
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

    public double getDriveEncoder() 
    {
        return m_driveEncoder.getPosition();
    }

    public void adjustPIDValues()
    {
        double p = SmartDashboard.getNumber("P Gain", DriveModulePIDValues.k_driveP);
        double i = SmartDashboard.getNumber("I Gain", DriveModulePIDValues.k_driveI);
        double d = SmartDashboard.getNumber("D Gain", DriveModulePIDValues.k_driveD);
        double ff = SmartDashboard.getNumber("FF Gain", DriveModulePIDValues.k_driveFF);

        if((p != DriveModulePIDValues.k_driveP)) { m_driverPIDController.setP(p); DriveModulePIDValues.k_driveP = p; System.out.println("CHANGED TO: " + p); }
        if((i != DriveModulePIDValues.k_driveI)) { m_driverPIDController.setI(i); DriveModulePIDValues.k_driveI = i; }
        if((d != DriveModulePIDValues.k_driveD)) { m_driverPIDController.setD(d); DriveModulePIDValues.k_driveD = d; }
        if((ff != DriveModulePIDValues.k_driveFF)) { m_driverPIDController.setP(ff); DriveModulePIDValues.k_driveFF = ff; }
    }
    
}
