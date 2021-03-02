// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

//Imports below are from SwerveMK3 file
import edu.wpi.first.wpilibj.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants 
{

    public static final class HIDConstants 
    {
        public static final int k_DriverControllerPort = 1;
        public static final double kDeadBand = 0.05;
    }

    public static final class SwerveDriveModuleConstants
    {
        //public static final double k_MaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        //public static final double k_MaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
        public static final double kMaxSpeed = Units.feetToMeters(15); // FIX gear ratio calc.
        public static final double kMaxAngularSpeed = 50; // ???? rotation per second
        public static double fieldCalibration = 0;

        //Angle offsets
        public static double frontLeftOffset = 20;//27.333984;
        public static double frontRightOffset = 78.574219;
        public static double backLeftOffset = 160;//155.195313;
        public static double backRightOffset = 3;//357.880859;

        public static final double k_SteeringEncoderCoefficient = (Math.PI * 2) / 4096.0; // this is 2π radians, dividided by the 12-bit encoder precision 

        //TODO Change values to match the robot
        public static final double ks_Volts = 1;
        public static final double kv_VoltSecondsPerMeter = 0.8;
        public static final double ka_VoltSecondsSquaredPerMeter = 0.15;

        public static final boolean k_GyroReversed = false;

        public static final AlternateEncoderType k_AlternateEncoderType = AlternateEncoderType.kQuadrature;
        public static final double kEncoderTicksPerRotation = 4096;


        public static final int k_RevNEOEncoderCtsPerRev = 42; // the NEO's hall-effect encoder is 42 counts/rev
        public static final double k_WheelDiameterMeters = 0.1016;
        public static final double k_MK3SwerveModuleGearRatio = 6.86;
        public static final double k_DriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (k_WheelDiameterMeters * Math.PI) / ( (double) k_RevNEOEncoderCtsPerRev * k_MK3SwerveModuleGearRatio);
    
        public static final double k_TurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kEncoderTicksPerRotation; //in radians

        public static final class CANBusIDs
        {
            public static final int k_LeftFront_DriveMotor = 1; 
            public static final int frontLeftCANCoderId = 9; 
            public static final int k_LeftFront_SteeringMotor = 2;

            public static final int k_RightFront_DriveMotor = 3; 
            public static final int frontRightCANCoderId = 10; 
            public static final int k_RightFront_SteeringMotor = 4; 

            public static final int k_LeftRear_DriveMotor = 5; 
            public static final int backLeftCANCoderId = 11; 
            public static final int k_LeftRear_SteeringMotor = 6;

            public static final int k_RightRear_DriveMotor = 7; 
            public static final int backRightCANCoderId = 12; 
            public static final int k_RightRear_SteeringMotor = 8;   
          }
        
        public static final class MotorTypes
        {
            public static final MotorType k_SwerveRightFront_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveRightFront_Steering = MotorType.kBrushless;
            
            public static final MotorType k_SwerveLeftFront_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveLeftFront_Steering = MotorType.kBrushless;
    
            public static final MotorType k_SwerveRightRear_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveRightRear_Steering = MotorType.kBrushless;
    
            public static final MotorType k_SwerveLeftRear_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveLeftRear_Steering = MotorType.kBrushless;
        }

        // Distance between centers of right and left wheels on robot; unit is meters
        public static final double k_TrackWidth = 0.5842;

        // Distance between front and back wheels on robot; unit is meters
        public static final double k_WheelBase = 0.5842;
        
     
        public static SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                new Translation2d(k_WheelBase / 2, k_TrackWidth / 2),    // Left Front
                new Translation2d(k_WheelBase / 2, -k_TrackWidth / 2),   // Right Front
                new Translation2d(-k_WheelBase / 2, k_TrackWidth / 2),   // Left Rear
                new Translation2d(-k_WheelBase / 2, -k_TrackWidth / 2)); // Right Rear


        public static final class DriveModulePIDValues
        {
            public static final double k_driveP = 15.0;
            public static final double k_driveI = 0.01;
            public static final double k_driveD = 0.1;

            public static final double k_driveIz = 0; 
            public static final double k_driveFF = .2; // feedforward
            
            public static final double k_driveMaxOutput = 1; 
            public static final double k_driveMinOutput = -1;
            public static final double k_driveMaxRPM = 5600;

            public static final double k_driveMaxVel = 5600; // rpm
            public static final double k_driveMinVel = 0;
            public static final double k_driveMaxAcc = 1500;

            public static final double k_driveAllowedError = 0.05; // allowedErr - The allowed deviation for your setpoint vs actual position in rotations
            public static final int k_driveSmartMotionSlot = 0; // Is the gain schedule slot, the value is a number between 0 and 3. Each slot has its own set of gain values and can be changed in each control frame using SetReference().
        }

        public static final class SteeringControllerPIDValues
        {
            public static final double k_steerP = 0.0005;
            public static final double k_steerI= 0;
            public static final double k_steerD = 0;

            public static final double k_steerIz = 0; 
            public static final double k_steerFF = 0; // feedforward

            public static final double k_steerMaxOutput = 1; 
            public static final double k_steerMinOutput = -1;
            public static final double k_steerMaxRPM = 5600;

            public static final double k_steerMaxVel = 2000; // rpm
            public static final double k_steerMinVel = 0;
            public static final double k_steerMaxAcc = 1500;

            public static final double k_steerAllowedError = 0.1; // allowedErr - The allowed deviation for your setpoint vs actual position in rotations
            public static final int k_steerSmartMotionSlot = 0; // Is the gain schedule slot, the value is a number between 0 and 3. Each slot has its own set of gain values and can be changed in each control frame using SetReference().

        }
    }
}
