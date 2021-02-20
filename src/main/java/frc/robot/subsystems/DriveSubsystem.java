// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants;

import frc.robot.subsystems.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//"Swerve Drive Tuning"


@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase 
{        
    private final SwerveModule m_LeftFrontModule = 
        new SwerveModule(
            SwerveDriveModuleConstants.CANBusIDs.k_LeftFront_DriveMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveLeftFront_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_LeftFront_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveLeftFront_Steering,
            SwerveDriveModuleConstants.k_LeftFrontDriveEncoderReversed,
            SwerveDriveModuleConstants.k_LeftFrontSteeringEncoderReversed);


   private final SwerveModule m_RightFrontModule = 
        new SwerveModule(
            SwerveDriveModuleConstants.CANBusIDs.k_RightFront_DriveMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightFront_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_RightFront_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightFront_Steering,
            SwerveDriveModuleConstants.k_RightFrontDriveEncoderReversed,
            SwerveDriveModuleConstants.k_RightFrontSteeringEncoderReversed);

    private final SwerveModule m_LeftRearModule = 
        new SwerveModule(
            SwerveDriveModuleConstants.CANBusIDs.k_LeftRear_DriveMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveLeftRear_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_LeftRear_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveLeftRear_Steering,
            SwerveDriveModuleConstants.k_LeftRearDriveEncoderReversed,
            SwerveDriveModuleConstants.k_LeftRearSteeringEncoderReversed);

    private final SwerveModule m_RightRearModule = 
        new SwerveModule(
            SwerveDriveModuleConstants.CANBusIDs.k_RightRear_DriveMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightRear_Drive,
            SwerveDriveModuleConstants.CANBusIDs.k_RightRear_SteeringMotor,
            SwerveDriveModuleConstants.MotorTypes.k_SwerveRightRear_Steering,
            SwerveDriveModuleConstants.k_RightRearDriveEncoderReversed,
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

    private NetworkTableEntry kP_Steer_widget, kI_Steer_widget, kD_Steer_widget, kIz_Steer_widget, kFF_Steer_widget, kMaxOutput_Steer_widget, kMinOutput_Steer_widget, maxRPM_Steer_widget, maxVel_Steer_widget, minVel_Steer_widget, maxAcc_Steer_widget, allowedErr_Steer_widget, setPos_Steer_widget, setVel_Steer_widget, motorMode_Steer_widget, moduleSel_Steer_widget, processVariable_steer_widget, appliedOutput_steer_widget;
    
    private NetworkTableEntry kP_Drive_widget, kI_Drive_widget, kD_Drive_widget, kIz_Drive_widget, kFF_Drive_widget, kMaxOutput_Drive_widget, kMinOutput_Drive_widget, maxRPM_Drive_widget, maxVel_Drive_widget, minVel_Drive_widget, maxAcc_Drive_widget, allowedErr_Drive_widget, setPos_Drive_widget, setVel_Drive_widget, motorMode_Drive_widget, moduleSel_Drive_widget;

    
    

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
        
        // !! Important! The SmartDashboard uses key/value pairs, so the keys here need to match the keys in periodic()
        // display PID coefficients on SmartDashboard
        kP_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Steer P Gain", kP_Steer).getEntry();
        kI_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Steer I Gain", kI_Steer).getEntry();
        kD_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Steer D Gain", kD_Steer).getEntry();
        kIz_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kIz_Steer Zone", kIz_Steer).getEntry();
        kFF_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kFF_Steer Feed Forward", kFF_Steer).getEntry();
        kMaxOutput_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kMaxOutput_Steer Max Output", kMaxOutput_Steer).getEntry();
        kMinOutput_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kMinOutput_Steer Min Output", kMinOutput_Steer).getEntry();
        
        // display Smart Motion coefficients
        maxVel_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("maxVel_Steer Max Velocity", maxVel_Steer).getEntry();
        minVel_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("minVel_Steer Min Velocity", minVel_Steer).getEntry();
        maxAcc_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("maxAcc_Steer Acceleration", maxAcc_Steer).getEntry();
        allowedErr_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("allowedErr_Steer Allowed Closed Loop Error", allowedErr_Steer).getEntry();
        setPos_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Set Position Steer", 0).getEntry();
        setVel_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Set Velocity Steer", 0).getEntry();

        // button to toggle between velocity and smart motion modes
        motorMode_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Steering Motor Mode", true).getEntry();

        // 1 = m_LeftFrontModule   2 = m_RightFrontModule
        // 3 = m_LeftRearModule    4 = m_RightRearModule
        moduleSel_Steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Select Steer Swerve Module", 1).getEntry();

   
        
        processVariable_steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Process Variable Steer", 0).getEntry();
        appliedOutput_steer_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Output Steer", 0).getEntry();




        // display PID coefficients on SmartDashboard
        kP_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Drive P Gain", kP_Drive).getEntry();
        kI_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Drive I Gain", kI_Drive).getEntry();
        kD_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kD_Drive Gain", kD_Drive).getEntry();
        kIz_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kIz_Drive Zone", kIz_Drive).getEntry();
        kFF_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kFF_Drive Feed Forward", kFF_Drive).getEntry();
        kMaxOutput_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kMaxOutput_Drive Max Output", kMaxOutput_Drive).getEntry();
        kMinOutput_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("kMinOutput_Drive Output", kMinOutput_Drive).getEntry();

        // display Smart Motion coefficients
        maxVel_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("maxVel_Drive Max Velocity", maxVel_Drive).getEntry();
        minVel_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("minVel_Drive Min Velocity", minVel_Drive).getEntry();
        maxAcc_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("maxAcc_Drive Acceleration", maxAcc_Drive).getEntry();
        allowedErr_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("allowedErr_Drive Allowed Closed Loop Error", allowedErr_Drive).getEntry();
        setPos_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Set Position Drive", 0).getEntry();
        setVel_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Set Velocity Drive", 0).getEntry();

        // button to toggle between velocity and smart motion modes
        motorMode_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Driving Motor Mode", true).getEntry();

        // 1 = m_LeftFrontModule   2 = m_RightFrontModule
        // 3 = m_LeftRearModule    4 = m_RightRearModule
        moduleSel_Drive_widget = Shuffleboard.getTab("Swerve Drive Tuning").add("Select Drive Swerve Module", 1).getEntry();
    }

    @Override
    public void periodic()
    {
        MotorControlsValuesUpdate();

        // Update the odeometry in the periodic block
        // (Please provide the states in the same order in which you instantiated your 
        // SwerveDriveKinematics.)
       m_odometry.update(new Rotation2d(getHeading()),
            m_LeftFrontModule.getState(),
            m_RightFrontModule.getState(),
            m_LeftRearModule.getState(),
            m_RightRearModule.getState());

    //    m_Odometry_TEMP.update(new Rotation2d(0.0), m_LeftFrontModule.getState());

    }

    public void MotorControlsValuesUpdate()
    {
        // read PID coefficients from SmartDashboard
        double local_kP_Steer = kP_Steer_widget.getDouble(0);
        double local_kI_Steer = kI_Steer_widget.getDouble(0);
        double local_kD_Steer = kD_Steer_widget.getDouble(0);
        double local_kIz_Steer = kIz_Steer_widget.getDouble(0);
        double local_kFF_Steer = kFF_Steer_widget.getDouble(0);

        double local_kMaxOutput_Steer = kMaxOutput_Steer_widget.getDouble(0);
        double local_kMinOutput_Steer = kMinOutput_Steer_widget.getDouble(0);

        double local_maxVel_Steer = maxVel_Steer_widget.getDouble(0);
        double local_minVel_Steer = minVel_Steer_widget.getDouble(0);
        double local_maxAcc_Steer = maxAcc_Steer_widget.getDouble(0);

        double local_allowedErr_Steer = allowedErr_Steer_widget.getDouble(0);

        double local_kP_Drive = kP_Drive_widget.getDouble(0);
        double local_kI_Drive = kI_Drive_widget.getDouble(0);
        double local_kD_Drive = kD_Drive_widget.getDouble(0);
        double local_kIz_Drive = kIz_Drive_widget.getDouble(0);
        double local_kFF_Drive = kFF_Drive_widget.getDouble(0);

        double local_kMaxOutput_Drive = kMaxOutput_Drive_widget.getDouble(0);
        double local_kMinOutput_Drive = kMinOutput_Drive_widget.getDouble(0);

        double local_maxVel_Drive = maxVel_Drive_widget.getDouble(0);
        double local_minVel_Drive = minVel_Drive_widget.getDouble(0);
        double local_maxAcc_Drive = maxAcc_Drive_widget.getDouble(0);

        double local_allowedErr_Drive = allowedErr_Drive_widget.getDouble(0);



        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if (local_kP_Steer != kP_Steer) 
        {  
            m_LeftFrontModule.setSteerMotor_P(local_kP_Steer);
            m_RightFrontModule.setSteerMotor_P(local_kP_Steer);
            m_LeftRearModule.setSteerMotor_P(local_kP_Steer);
            m_RightRearModule.setSteerMotor_P(local_kP_Steer);

            kP_Steer = local_kP_Steer; 
        }

        if (local_kP_Drive != kP_Drive)
        {
            m_LeftFrontModule.setDriveMotor_P(local_kP_Drive);
            m_RightFrontModule.setDriveMotor_P(local_kP_Drive);
            m_LeftRearModule.setDriveMotor_P(local_kP_Drive);
            m_RightRearModule.setDriveMotor_P(local_kP_Drive);

            kP_Drive = local_kP_Drive;
        }

        if (local_kI_Steer != kI_Steer) 
        {  
            m_LeftFrontModule.setSteerMotor_I(local_kI_Steer);
            m_RightFrontModule.setSteerMotor_I(local_kI_Steer);
            m_LeftRearModule.setSteerMotor_I(local_kI_Steer);
            m_RightRearModule.setSteerMotor_I(local_kI_Steer);

            kI_Steer = local_kI_Steer; 
        }

        if (local_kI_Drive != kI_Drive)
        {
            m_LeftFrontModule.setDriveMotor_I(local_kI_Drive);
            m_RightFrontModule.setDriveMotor_I(local_kI_Drive);
            m_LeftRearModule.setDriveMotor_I(local_kI_Drive);
            m_RightRearModule.setDriveMotor_I(local_kI_Drive);

            kI_Drive = local_kI_Drive;
        }

        if (local_kD_Steer != kD_Steer) 
        {  
            m_LeftFrontModule.setSteerMotor_D(local_kD_Steer);
            m_RightFrontModule.setSteerMotor_D(local_kD_Steer);
            m_LeftRearModule.setSteerMotor_D(local_kD_Steer);
            m_RightRearModule.setSteerMotor_D(local_kD_Steer);

            kD_Steer = local_kD_Steer; 
        }

        if (local_kD_Drive != kD_Drive)
        {
            m_LeftFrontModule.setDriveMotor_D(local_kD_Drive);
            m_RightFrontModule.setDriveMotor_D(local_kD_Drive);
            m_LeftRearModule.setDriveMotor_D(local_kD_Drive);
            m_RightRearModule.setDriveMotor_D(local_kD_Drive);

            kD_Drive = local_kD_Drive;
        }

        if (local_kIz_Steer != kIz_Steer) 
        {  
            m_LeftFrontModule.setSteerMotor_Iz(local_kIz_Steer);
            m_RightFrontModule.setSteerMotor_Iz(local_kIz_Steer);
            m_LeftRearModule.setSteerMotor_Iz(local_kIz_Steer);
            m_RightRearModule.setSteerMotor_Iz(local_kIz_Steer);

            kIz_Steer = local_kIz_Steer; 
        }

        if (local_kIz_Drive != kIz_Drive)
        {
            m_LeftFrontModule.setDriveMotor_Iz(local_kIz_Drive);
            m_RightFrontModule.setDriveMotor_Iz(local_kIz_Drive);
            m_LeftRearModule.setDriveMotor_Iz(local_kIz_Drive);
            m_RightRearModule.setDriveMotor_Iz(local_kIz_Drive);

            kIz_Drive = local_kIz_Drive;
        }

        if (local_kFF_Steer != kFF_Steer) 
        {  
            m_LeftFrontModule.setSteerMotor_FF(local_kFF_Steer);
            m_RightFrontModule.setSteerMotor_FF(local_kFF_Steer);
            m_LeftRearModule.setSteerMotor_FF(local_kFF_Steer);
            m_RightRearModule.setSteerMotor_FF(local_kFF_Steer);

            kFF_Steer = local_kFF_Steer; 
        }

        if (local_kFF_Drive != kFF_Drive)
        {
            m_LeftFrontModule.setDriveMotor_FF(local_kFF_Drive);
            m_RightFrontModule.setDriveMotor_FF(local_kFF_Drive);
            m_LeftRearModule.setDriveMotor_FF(local_kFF_Drive);
            m_RightRearModule.setDriveMotor_FF(local_kFF_Drive);

            kFF_Drive = local_kFF_Drive;
        }

        if ( (local_kMinOutput_Steer != kMinOutput_Steer) || (local_kMaxOutput_Steer != kMaxOutput_Steer) ) 
        {
            m_LeftFrontModule.setSteerMotorOutputRange(local_kMinOutput_Steer, local_kMaxOutput_Steer);
            m_RightFrontModule.setSteerMotorOutputRange(local_kMinOutput_Steer, local_kMaxOutput_Steer);
            m_LeftRearModule.setSteerMotorOutputRange(local_kMinOutput_Steer, local_kMaxOutput_Steer);
            m_RightRearModule.setSteerMotorOutputRange(local_kMinOutput_Steer, local_kMaxOutput_Steer);
         
            kMinOutput_Steer = local_kMinOutput_Steer;
            kMaxOutput_Steer = local_kMaxOutput_Steer; 
        }

        if ( (local_kMinOutput_Drive != kMinOutput_Drive) || (local_kMaxOutput_Drive != kMaxOutput_Drive) ) 
        {
            m_LeftFrontModule.setDriveMotorOutputRange(local_kMinOutput_Drive, local_kMaxOutput_Drive);
            m_RightFrontModule.setDriveMotorOutputRange(local_kMinOutput_Drive, local_kMaxOutput_Drive);
            m_LeftRearModule.setDriveMotorOutputRange(local_kMinOutput_Drive, local_kMaxOutput_Drive);
            m_RightRearModule.setDriveMotorOutputRange(local_kMinOutput_Drive, local_kMaxOutput_Drive);
         
            kMinOutput_Drive = local_kMinOutput_Drive;
            kMaxOutput_Drive = local_kMaxOutput_Drive; 
        }
        
        if ((local_maxVel_Steer != maxVel_Steer)) 
        {
            m_LeftFrontModule.setSteerMotorSmartMaxVel(local_maxVel_Steer);
            m_RightFrontModule.setSteerMotorSmartMaxVel(local_maxVel_Steer);
            m_LeftRearModule.setSteerMotorSmartMaxVel(local_maxVel_Steer);
            m_RightRearModule.setSteerMotorSmartMaxVel(local_maxVel_Steer);
            
            maxVel_Steer = local_maxVel_Steer; 
        }

        if ((local_maxVel_Drive != maxVel_Drive)) 
        {
            m_LeftFrontModule.setDriveMotorSmartMaxVel(local_maxVel_Drive);
            m_RightFrontModule.setDriveMotorSmartMaxVel(local_maxVel_Drive);
            m_LeftRearModule.setDriveMotorSmartMaxVel(local_maxVel_Drive);
            m_RightRearModule.setDriveMotorSmartMaxVel(local_maxVel_Drive);
            
            maxVel_Drive = local_maxVel_Drive; 
        }

        if ((local_minVel_Steer != minVel_Steer)) 
        {
            m_LeftFrontModule.setSteerMotorSmartMinVel(local_minVel_Steer);
            m_RightFrontModule.setSteerMotorSmartMinVel(local_minVel_Steer);
            m_LeftRearModule.setSteerMotorSmartMinVel(local_minVel_Steer);
            m_RightRearModule.setSteerMotorSmartMinVel(local_minVel_Steer);
            
            minVel_Steer = local_minVel_Steer; 
        }

        if ((local_minVel_Drive != minVel_Drive)) 
        {
            m_LeftFrontModule.setDriveMotorSmartMinVel(local_minVel_Drive);
            m_RightFrontModule.setDriveMotorSmartMinVel(local_minVel_Drive);
            m_LeftRearModule.setDriveMotorSmartMinVel(local_minVel_Drive);
            m_RightRearModule.setDriveMotorSmartMinVel(local_minVel_Drive);
            
            minVel_Drive = local_minVel_Drive; 
        }
        

        if ((local_maxAcc_Steer != maxAcc_Steer)) 
        {
            m_LeftFrontModule.setSteerMotorMaxAcc(local_maxAcc_Steer);
            m_RightFrontModule.setSteerMotorMaxAcc(local_maxAcc_Steer);
            m_LeftRearModule.setSteerMotorMaxAcc(local_maxAcc_Steer);
            m_RightRearModule.setSteerMotorMaxAcc(local_maxAcc_Steer);
            
            maxAcc_Steer = local_maxAcc_Steer; 
        } 
        
        if ((local_maxAcc_Drive != maxAcc_Drive)) 
        {
            m_LeftFrontModule.setDriveMotorMaxAcc(local_maxAcc_Drive);
            m_RightFrontModule.setDriveMotorMaxAcc(local_maxAcc_Drive);
            m_LeftRearModule.setDriveMotorMaxAcc(local_maxAcc_Drive);
            m_RightRearModule.setDriveMotorMaxAcc(local_maxAcc_Drive);
            
            maxAcc_Drive = local_maxAcc_Drive; 
        }
        
        if ((local_allowedErr_Steer != allowedErr_Steer)) 
        {
            m_LeftFrontModule.setSteerMotorAllowedErr(local_allowedErr_Steer);
            m_RightFrontModule.setSteerMotorAllowedErr(local_allowedErr_Steer);
            m_LeftRearModule.setSteerMotorAllowedErr(local_allowedErr_Steer);
            m_RightRearModule.setSteerMotorAllowedErr(local_allowedErr_Steer);
            
            allowedErr_Steer = local_allowedErr_Steer; 
        } 
        
        if ((local_allowedErr_Drive != allowedErr_Drive)) 
        {
            m_LeftFrontModule.setDriveMotorAllowedErr(local_allowedErr_Drive);
            m_RightFrontModule.setDriveMotorAllowedErr(local_allowedErr_Drive);
            m_LeftRearModule.setDriveMotorAllowedErr(local_allowedErr_Drive);
            m_RightRearModule.setDriveMotorAllowedErr(local_allowedErr_Drive);
            
            allowedErr_Drive = local_allowedErr_Drive; 
        }


        // Mapping for number selector below
        // // 1 = m_LeftFrontModule   2 = m_RightFrontModule
        // // 3 = m_LeftRearModule    4 = m_RightRearModule
        // Shuffleboard.getTab("Swerve Drive Tuning").add("Select Steer Swerve Module", 1);

        double setPoint_steer, processVariable_steer, appliedOutput_steer;
        boolean mode = motorMode_Steer_widget.getBoolean(false);
        
        if (mode) // true means use Velocity mode 
        {
            setPoint_steer = setVel_Steer_widget.getDouble(0);
            m_LeftFrontModule.setSteerMotorVelocityMode(setPoint_steer);
            m_RightFrontModule.setSteerMotorVelocityMode(setPoint_steer);
            m_LeftRearModule.setSteerMotorVelocityMode(setPoint_steer);
            m_RightRearModule.setSteerMotorVelocityMode(setPoint_steer);

            double module = moduleSel_Steer_widget.getDouble(1);
            switch ((int)module)
            {
                case 1: processVariable_steer = m_LeftFrontModule.getSteeringVelocity();
                        appliedOutput_steer = m_LeftFrontModule.getSteerAppliedOutput();
                        break;

                case 2: processVariable_steer = m_RightFrontModule.getSteeringVelocity();
                        appliedOutput_steer = m_RightFrontModule.getSteerAppliedOutput();
                        break;

                case 3: processVariable_steer = m_LeftRearModule.getSteeringVelocity();
                        appliedOutput_steer = m_LeftRearModule.getSteerAppliedOutput();
                        break;

                case 4: processVariable_steer = m_RightRearModule.getSteeringVelocity();
                        appliedOutput_steer = m_RightRearModule.getSteerAppliedOutput();
                        break;

                default:processVariable_steer = m_LeftFrontModule.getSteeringVelocity();
                        appliedOutput_steer = m_LeftFrontModule.getSteerAppliedOutput();
                        break;
            }
        } 
        else // use SmartMotion mode
        {
            setPoint_steer = setPos_Steer_widget.getDouble(0);
            /**
             * As with other PID modes, Smart Motion is set by calling the
             * setReference method on an existing pid object and setting
             * the control type to kSmartMotion
             */
            m_LeftFrontModule.setSteerMotorSmartMotionMode(setPoint_steer);
            m_RightFrontModule.setSteerMotorSmartMotionMode(setPoint_steer);
            m_LeftRearModule.setSteerMotorSmartMotionMode(setPoint_steer);
            m_RightRearModule.setSteerMotorSmartMotionMode(setPoint_steer);

            double module = moduleSel_Steer_widget.getDouble(1);
            switch ((int)module)
            {
                case 1: processVariable_steer = m_LeftFrontModule.getSteeringPosition();
                        appliedOutput_steer = m_LeftFrontModule.getSteerAppliedOutput();
                        break;

                case 2: processVariable_steer = m_RightFrontModule.getSteeringPosition();
                        appliedOutput_steer = m_RightFrontModule.getSteerAppliedOutput();
                        break;

                case 3: processVariable_steer = m_LeftRearModule.getSteeringPosition();
                        appliedOutput_steer = m_LeftRearModule.getSteerAppliedOutput();
                        break;

                case 4: processVariable_steer = m_RightRearModule.getSteeringPosition();
                        appliedOutput_steer = m_RightRearModule.getSteerAppliedOutput();
                        break;

                default:processVariable_steer = m_LeftFrontModule.getSteeringPosition();
                        appliedOutput_steer = m_LeftFrontModule.getSteerAppliedOutput();
                        break;
            }
        }
        
        setPos_Steer_widget.setDouble(setPoint_steer);
        processVariable_steer_widget.setDouble(processVariable_steer);
        appliedOutput_steer_widget.setDouble(appliedOutput_steer);
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
