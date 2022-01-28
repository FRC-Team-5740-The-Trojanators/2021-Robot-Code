// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.talonPID;
import frc.robot.Constants.SwerveDriveModuleConstants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants.SteeringControllerPIDValues;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;


public class TalonTesting extends SubsystemBase {
  /** Creates a new TalonTesting. */

  //private PIDController m_steeringPIDController = new PIDController(0, 0, 0);
  private TalonFX m_angleMotor;
  private CANCoder m_moduleSteeringEncoder;
  private Rotation2d m_offset;


  public TalonTesting(/*TalonFX angleMotor, CANCoder moduleSteeringEncoder, Rotation2d offset*/) 
  {
    // m_angleMotor = angleMotor;
     m_moduleSteeringEncoder = new CANCoder(CANBusIDs.backRightCANCoderId);
    // m_offset = offset;

    m_offset = new Rotation2d(0);
    m_moduleSteeringEncoder.configFactoryDefault();
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = m_offset.getDegrees();
    //canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    m_moduleSteeringEncoder.configAllSettings(canCoderConfiguration);
    TalonFXConfiguration angleTalonConfig = new TalonFXConfiguration();
    
    
    m_angleMotor = new TalonFX(CANBusIDs.k_TalonTestID);
    m_angleMotor.configFactoryDefault();
    m_angleMotor.setNeutralMode(NeutralMode.Brake);

    angleTalonConfig.slot0.allowableClosedloopError = 10;
    angleTalonConfig.remoteFilter0.remoteSensorDeviceID = m_moduleSteeringEncoder.getDeviceID();
    angleTalonConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    angleTalonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    angleTalonConfig.slot0.kP = talonPID.k_talonP;
    m_angleMotor.configAllSettings(angleTalonConfig);
  }

  public void setSetpoint(int setpoint)
  {
    m_angleMotor.set(TalonFXControlMode.Position, setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("canPos", m_moduleSteeringEncoder.getPosition());
    SmartDashboard.putNumber("canAbsPos", m_moduleSteeringEncoder.getAbsolutePosition());

  }

  public void runTalon(double speed)
  {
    m_angleMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void stopTalon()
  {
    m_angleMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
}
