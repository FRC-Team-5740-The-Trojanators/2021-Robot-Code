// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.Set;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.DriveModulePIDValues;
import frc.robot.Constants.SwerveDriveModuleConstants.SteeringControllerPIDValues;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.util.Units;



/** 
 * Swerve Module is based on the WPI Swerve module code.
 * 
 */
public class SwerveModule
{
    // REV robotics Spark Max motor controllers
    // private CANSparkMax m_driveMotor;
    // private CANSparkMax m_steeringMotor;

    // // REV Robotics Spark Max encoder signal; the encoder built-in to the motor
    // private CANEncoder m_driveEncoder;

    // // CTRE SRX Magnetic Encoder signal; encoder is external and mounted on the swerve module
    private CANEncoder m_steeringEncoder;

    // PID Loops using the Spark API
    private CANPIDController m_driverPIDController;
    private PIDController m_steeringPIDController;

    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private CANCoder canCoder;
    private Rotation2d offset;

    private static double kEncoderTicksPerRotation = 4096;

    // These two PID controllers are from the WPILib and we're temporarily commenting them to use the REV Spark PIDs
    // private final PIDController m_driverPIDController = new PIDController(SwerveDriveModuleConstants.DriveModule.k_Proportional, 
    //                                                                       SwerveDriveModuleConstants.DriveModule.k_Intergral,
    //                                                                       SwerveDriveModuleConstants.DriveModule.k_Differential);

    // private final ProfiledPIDController m_turningPIDController = 
    //     new ProfiledPIDController(
    //         SwerveDriveModuleConstants.TurningController.k_Prop,
    //         SwerveDriveModuleConstants.TurningController.k_Inter,
    //         SwerveDriveModuleConstants.TurningController.k_Diff,
    //         new TrapezoidProfile.Constraints(
    //             SwerveDriveModuleConstants.k_MaxModuleAngularSpeedRadiansPerSecond,
    //             SwerveDriveModuleConstants.k_MaxModuleAngularAccelerationRadiansPerSecondSquared) );
    

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     */
    public SwerveModule(CANSparkMax driveMotor, CANSparkMax angleMotor, CANCoder canCoder, Rotation2d offset) {

        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;
        this.offset = offset;

        m_driverPIDController = driveMotor.getPIDController();
       // m_steeringPIDController = angleMotor.getPIDController();
       m_steeringEncoder = this.angleMotor.getEncoder();
       this.m_steeringEncoder = m_steeringEncoder;

         m_steeringPIDController = new PIDController(SteeringControllerPIDValues.k_steerP, 
                                                     SteeringControllerPIDValues.k_steerI,
                                                    SteeringControllerPIDValues.k_steerD);

        m_steeringPIDController.setP(SteeringControllerPIDValues.k_steerP);
        m_steeringPIDController.setI(SteeringControllerPIDValues.k_steerI);
        m_steeringPIDController.setD(SteeringControllerPIDValues.k_steerD);

        m_driverPIDController.setP(DriveModulePIDValues.k_driveP);
        m_driverPIDController.setI(DriveModulePIDValues.k_driveI);
        m_driverPIDController.setD(DriveModulePIDValues.k_driveD);
        m_driverPIDController.setFF(DriveModulePIDValues.k_driveFF);

        
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoder.configAllSettings(canCoderConfiguration);
            
            
            //ConfigureDriveMotor(driveMotorChannel, driveMotorType);
           //ConfigureSteeringMotor(steeringMotorChannel, steeringMotorType);

            // m_steeringEncoder = m_steeringMotor.getAlternateEncoder(SwerveDriveModuleConstants.k_AlternateEncoderType, 
            //                                                         SwerveDriveModuleConstants.k_AltEnc_CountPerRev);

            // // Steering encoder is a Cross-The-Road Electronics CANCoder
            // this.m_steeringEncoder = new CANCoder(steeringEncoderChannel); 
            
            // CANCoderConfiguration m_canCoderConfiguration = new CANCoderConfiguration();
            // m_canCoderConfiguration.unitString = "radians";
            // m_canCoderConfiguration.sensorCoefficient = SwerveDriveModuleConstants.k_SteeringEncoderCoefficient;

            // m_steeringEncoder.configAllSettings(m_canCoderConfiguration);

            
            // Limit the PID Controller's input range between -pi and pi and set the input
            // to be continuous.
           // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    public double getRawAngle() {
        return canCoder.getAbsolutePosition();
    }

    public double getSteeringEncoderValue(){
        return m_steeringEncoder.getPosition();
    }

    public void setEncoders(){
    if (
        canCoder.getAbsolutePosition() != m_steeringEncoder.getPosition()
    ){
        m_steeringEncoder.setPosition(canCoder.getAbsolutePosition()); 
        System.out.println("Hey it works!");
    }
    }

    double setpoint;

    public void setDesiredState(SwerveModuleState desiredState)
    {      
       //Steering Motor Calc
        Rotation2d currentRotation = getAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
         Rotation2d rotationDelta = state.angle.minus(currentRotation);

        // double steeringCurrentPos_degrees = ((m_steeringEncoder.getPosition() / m_steeringEncoder.getCountsPerRevolution())* 360) % 360; //converts the fraction of a rotation to degrees
        // Rotation2d currentRotation = Rotation2d.fromDegrees(steeringCurrentPos_degrees);
        // SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        // setpoint = steeringCurrentPos_degrees - state.angle.getDegrees();
       // m_steeringPIDController.setReference(setpoint, ControlType.kPosition);


       double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
        double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
        double desiredTicks = currentTicks + deltaTicks;
       angleMotor.set(m_steeringPIDController.calculate(currentTicks, desiredTicks));


       //Drive Motor Calc
        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
        m_driverPIDController.setReference(feetPerSecond / SwerveDriveModuleConstants.kMaxSpeed, ControlType.kVoltage);

     
          
        
        // // Calculate the drive output from the drive PID controller.
        // final var driveOutput = m_driverPIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

        // // Calculate the turning motor output from the turning PID controller.
        // final var steerOutput = m_turningPIDController.calculate(m_steeringEncoder.getPosition(), state.angle.getRadians());

        // m_driveMotor.set(driveOutput);
        // m_steeringMotor.set(steerOutput);

       // m_driverPIDController.setReference(state.speedMetersPerSecond, ControlType.kSmartVelocity);
      //  m_steeringPIDController.setReference(state.angle.getRadians(), ControlType.kSmartMotion);
    
    
    }

    public double getSetpoint(){
        return setpoint;
    }


//Everything after this point was not in the Example Code
    // /** Zeros all the SwerveModule encoders. */
    // public void resetEncoders()
    // {
    //     m_driveEncoder.setPosition(0);
    //     m_steeringEncoder.setPosition(0);
        
    // }

//     private void ConfigureDriveMotor(int channel, MotorType motorType)
//     {
//         m_driveMotor = new CANSparkMax(channel, motorType);

//         m_driveEncoder = m_driveMotor.getEncoder();   //assumes the use of the internal motor encoder
//         //m_driveEncoder.setVelocityConversionFactor(SwerveDriveModuleConstants.k_DriveEncoderDistancePerPulse); // <- This line for use in WPILib PID loops, not Spark
       
//         m_driverPIDController = m_driveMotor.getPIDController();

//         // following the example found here: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
//         m_driverPIDController.setP(DriveModulePIDValues.k_driveP);
//         m_driverPIDController.setI(DriveModulePIDValues.k_driveI);
//         m_driverPIDController.setD(DriveModulePIDValues.k_driveD);
//         m_driverPIDController.setIZone(DriveModulePIDValues.k_driveIz);
//         m_driverPIDController.setFF(DriveModulePIDValues.k_driveFF);
//         m_driverPIDController.setOutputRange(DriveModulePIDValues.k_driveMinOutput, DriveModulePIDValues.k_driveMaxOutput);

//         /**
//          * Smart Motion coefficients are set on a CANPIDController object
//          * 
//          * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
//          * the pid controller in Smart Motion mode
//          * - setSmartMotionMinOutputVelocity() will put a lower bound in
//          * RPM of the pid controller in Smart Motion mode
//          * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
//          * of the pid controller in Smart Motion mode
//          * - setSmartMotionAllowedClosedLoopError() will set the max allowed
//          * error for the pid controller in Smart Motion mode
//          */

//          /*
//         m_driverPIDController.setSmartMotionMaxVelocity(DriveModulePIDValues.k_maxVel, DriveModulePIDValues.k_smartMotionSlot);
//         m_driverPIDController.setSmartMotionMinOutputVelocity(DriveModulePIDValues.k_minVel, DriveModulePIDValues.k_smartMotionSlot);
//         m_driverPIDController.setSmartMotionMaxAccel(DriveModulePIDValues.k_maxAcc, DriveModulePIDValues.k_smartMotionSlot);
//         m_driverPIDController.setSmartMotionAllowedClosedLoopError(DriveModulePIDValues.k_allowedError, DriveModulePIDValues.k_smartMotionSlot);
//         */
        

    
// private void ConfigureSteeringMotor(int channel, MotorType motorType)
//     {
//         m_steeringMotor = new CANSparkMax(channel, motorType);

//         // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Alternate%20Encoder/src/main/java/frc/robot/Robot.java
//         m_steeringEncoder = m_steeringMotor.getAlternateEncoder(SwerveDriveModuleConstants.k_AlternateEncoderType, SwerveDriveModuleConstants.k_AltEnc_CountPerRev);   // Using alternate external SRX Magnetic encoder
        
//         m_steeringPIDController = m_steeringMotor.getCANPIDController();
//         m_steeringPIDController.setFeedbackDevice(m_steeringEncoder);

//         // following the example found here: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
//         m_steeringPIDController.setP(SteeringControllerPIDValues.k_steerP);
//         m_steeringPIDController.setI(SteeringControllerPIDValues.k_steerI);
//         m_steeringPIDController.setD(SteeringControllerPIDValues.k_steerD);
//        // m_steeringPIDController.setIZone(SteeringControllerPIDValues.k_steerIz);
//        // m_steeringPIDController.setFF(SteeringControllerPIDValues.k_steerFF);
//         m_steeringPIDController.setOutputRange(SteeringControllerPIDValues.k_steerMinOutput, SteeringControllerPIDValues.k_steerMaxOutput);

//         /**
//          * Smart Motion coefficients are set on a CANPIDController object
//          * 
//          * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
//          * the pid controller in Smart Motion mode
//          * - setSmartMotionMinOutputVelocity() will put a lower bound in
//          * RPM of the pid controller in Smart Motion mode
//          * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
//          * of the pid controller in Smart Motion mode
//          * - setSmartMotionAllowedClosedLoopError() will set the max allowed
//          * error for the pid controller in Smart Motion mode
//          */
//     /*
//         m_steeringPIDController.setSmartMotionMaxVelocity(SteeringControllerPIDValues.k_steerMaxVel, SteeringControllerPIDValues.k_smartMotionSlot);
//         m_steeringPIDController.setSmartMotionMinOutputVelocity(SteeringControllerPIDValues.k_steerMinVel, SteeringControllerPIDValues.k_smartMotionSlot);
//         m_steeringPIDController.setSmartMotionMaxAccel(SteeringControllerPIDValues.k_steerMaxAcc, SteeringControllerPIDValues.k_smartMotionSlot);
//         m_steeringPIDController.setSmartMotionAllowedClosedLoopError(SteeringControllerPIDValues.k_steerMaxAcc, SteeringControllerPIDValues.k_smartMotionSlot);
//     */
//     }



//     // Note to self and Team: This is cumbersome, but also very Java-y.

//     // Getters
//     /**
//      * Provides the Steering Motor encoder velocity
//      * @return the encoder velocity
//      */
//     public double getSteeringVelocity()
//     {
//        return m_steeringEncoder.getVelocity();
//     }

//     /**
//      * Provides the Driving Motor encoder velocity
//      * @return the encoder velocity
//      */
//     public double getDrivingVelocity()
//     {
//        return m_driveEncoder.getVelocity();
//     }

//     public double getSteerAppliedOutput()
//     {
//         return m_steeringMotor.getAppliedOutput();
//     }

//     public double getDriveAppliedOutput()
//     {
//             return m_driveMotor.getAppliedOutput();
//     }

//     public double getSteeringPosition()
//     {
//         return m_steeringEncoder.getPosition();
//     }

//     public double getDrivePosition()
//     {
//             return m_driveEncoder.getPosition();
//     }



//     // Setters
//     /**
//      * Sets the Steering Motor PIDController P-value.
//      * @param p The Proportional value
//      */
//     public void setSteerMotor_P(double p)
//     {
//         m_steeringPIDController.setP(p);
//     }

//     public void setDriveMotor_P(double p_Drive)
//     {
//         m_driverPIDController.setP(p_Drive);
//     }

//     /**
//      * Sets the Steering Motor PIDController I-value.
//      * @param i The Integral value.
//      */
//     public void setSteerMotor_I(double i)
//     {
//         m_steeringPIDController.setI(i);
//     }

//     public void setDriveMotor_I(double i_Drive)
//     {
//         m_driverPIDController.setI(i_Drive);
//     }

//     /**
//      * Sets the Steering Motor PIDController D-value.
//      * @param d The Differential value.
//      */
//     public void setSteerMotor_D(double d)
//     {
//         m_steeringPIDController.setD(d);
//     }

//     public void setDriveMotor_D(double d_Drive)
//     {
//         m_driverPIDController.setD(d_Drive);
//     }

//     public void setSteerMotor_Iz(double Iz)
//     {
//         m_steeringPIDController.setIZone(Iz);
//     }

//     /**
//      * Sets the Integral Zone for the Drive Motor PIDController.
//      * @param Iz Limits for Integral PID loop
//      */
//     public void setDriveMotor_Iz(double Iz)
//     {
//         m_driverPIDController.setIZone(Iz);
//     }

//     public void setSteerMotor_FF(double FF){
//         m_steeringPIDController.setFF(FF);
//     }

//     public void setDriveMotor_FF(double FF){
//         m_driverPIDController.setFF(FF);
//     }

//     /**
//      * Sets the Steering Motor PIDCOntroller Output range
//      * @param min The range minimum.
//      * @param max The range maximum. 
//      */
//     public void setSteerMotorOutputRange(double min, double max)
//     {
//         m_steeringPIDController.setOutputRange(min, max);
//     }

//     public void setDriveMotorOutputRange( double min_Drive, double max_Drive)
//     {
//         m_driverPIDController.setOutputRange(min_Drive, max_Drive);
//     }

//     /**
//      * Sets the Steering Motor PIDController Smart Motion maximum velocity (in RPM)
//      * @param maxVel The maximum velocity.
//      */
//     public void setSteerMotorSmartMaxVel(double maxVel)
//     {
//         m_steeringPIDController.setSmartMotionMaxVelocity(maxVel, 0); // we're not using the 'slot' paramter of the motor controller, so the second parameter is just 0 for Slot 0
//     }

//     public void setDriveMotorSmartMaxVel(double maxVel)
//     {
//         m_driverPIDController.setSmartMotionMaxVelocity(maxVel, 0); 
//     }

//     public void setSteerMotorSmartMinVel(double minVel)
//     {
//         m_steeringPIDController.setSmartMotionMinOutputVelocity(minVel, 0);
//     }
    
//     public void setDriveMotorSmartMinVel(double minVel)
//     {
//         m_driverPIDController.setSmartMotionMinOutputVelocity(minVel, 0);
//     }

//     public void setSteerMotorVelocityMode(double setPoint)
//     {
//         m_steeringPIDController.setReference(setPoint, ControlType.kVelocity);
//     }

//     public void setSteerMotorMaxAcc(double maxAcc)
//     {
//         m_steeringPIDController.setSmartMotionMaxAccel(maxAcc, 0);
//     }

//     public void setDriveMotorMaxAcc(double maxAcc)
//     {
//         m_driverPIDController.setSmartMotionMaxAccel(maxAcc, 0);
//     }

//     public void setSteerMotorAllowedErr(double allowedErr)
//     {
//         m_steeringPIDController.setSmartMotionMaxAccel(allowedErr, 0);
//     }

//     public void setDriveMotorAllowedErr(double allowedErr)
//     {
//         m_driverPIDController.setSmartMotionMaxAccel(allowedErr, 0);
//     }

//     public void setSteerMotorSmartMotionMode(double setPoint)
//     {
//         m_steeringPIDController.setReference(setPoint, ControlType.kSmartMotion);
//     }

//     public void setDriveMotorSmartMotionMode(double setPoint)
//     {
//         m_driverPIDController.setReference(setPoint, ControlType.kSmartMotion);
//     }



//     


}



