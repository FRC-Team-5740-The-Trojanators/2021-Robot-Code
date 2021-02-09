// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

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
    }
    




    public static final class SwerveDriveModuleConstants
    {
        public static final double k_MaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double k_MaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        public static final double k_SteeringEncoderCoefficient = (Math.PI * 2) / 4096.0; // this is 2π radians, dividided by the 12-bit encoder precision 


        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        // !!!!!!!!!!!!!!!!!!!!!!!
        public static final double ks_Volts = 1;
        public static final double kv_VoltSecondsPerMeter = 0.8;
        public static final double ka_VoltSecondsSquaredPerMeter = 0.15;

        public static final double k_MaxSpeedMetersPerSecond = 3;
        // !!!!!!!!!!!!!!!!!!!!!!!

        public static final boolean k_GyroReversed = false;


        public static final AlternateEncoderType k_AlternateEncoderType = AlternateEncoderType.kQuadrature;
        public static final int k_AltEnc_CountPerRev = 4096;


        public static final int k_RevNEOEncoderCtsPerRev = 42; // the NEO's hall-effect encoder is 42 counts/rev
        public static final double k_WheelDiameterMeters = 0.15;
        public static final double k_MK3SwerveModuleGearRatio = 6.86;
        public static final double k_DriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (k_WheelDiameterMeters * Math.PI) / ( (double) k_RevNEOEncoderCtsPerRev * 6.86);
    
        public static final double k_TurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) k_RevNEOEncoderCtsPerRev;

        public static final class CANBusIDs
        {
            public static final int k_SteeringEncoder = 30;

            public static final int k_RightFront_DriveMotor = 1;
            public static final int k_RightFront_SteeringMotor = 2;
    
            public static final int k_LeftFront_DriveMotor = 3;
            public static final int k_LeftFront_SteeringMotor = 4;

            public static final int k_RightRear_DriveMotor  = 5;
            public static final int k_RightRear_SteeringMotor = 6;
    
            public static final int k_LeftRear_DriveMotor  = 7;
            public static final int k_LeftRear_SteeringMotor = 8;
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

        // Distance between centers of right and left wheels on robot; unit is meters, I think
        public static final double k_TrackWidth = 0.5;

        // Distance between front and back wheels on robot; unit is meters, I think
        public static final double k_WheelBase = 0.7;
        
        public static final SwerveDriveKinematics k_DriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(k_WheelBase / 2, k_TrackWidth / 2),    // Left Front
                new Translation2d(k_WheelBase / 2, -k_TrackWidth / 2),   // Right Front
                new Translation2d(-k_WheelBase / 2, k_TrackWidth / 2),   // Left Rear
                new Translation2d(-k_WheelBase / 2, -k_TrackWidth / 2)); // Right Rear
        

        public static final class TurningController
        {
            public static final double k_Prop = 1;
            public static final double k_Inter = 0;
            public static final double k_Diff = 0;
        }

        public static final class DriveModule
        {
            public static final double k_Proportional = 1;
            public static final double k_Intergral = 0;
            public static final double k_Differential = 0;
        }

        // Unsure if we need these constants, including them just-in-case
        public static final boolean k_LeftFrontSteeringEncoderReversed = false;
        public static final boolean k_LeftRearSteeringEncoderReversed = false;
        public static final boolean k_RightFrontSteeringEncoderReversed = false;
        public static final boolean k_RightRearSteeringEncoderReversed = false;

        public static final boolean k_LeftFrontDriveEncoderReversed = false;
        public static final boolean k_LeftRearDriveEncoderReversed = false;
        public static final boolean k_RightFrontDriveEncoderReversed = false;
        public static final boolean k_RightRearDriveEncoderReversed = false;

    }
}
