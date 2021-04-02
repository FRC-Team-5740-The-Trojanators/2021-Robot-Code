// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveModuleConstants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.FlywheelPIDValues;

public class FlyWheelSubsystem extends SubsystemBase {

  
  private CANEncoder m_flywheelEncoder;

  private CANSparkMax FlywheelMotorOne = new CANSparkMax(CANBusIDs.k_FlywheelMotorOne, MotorType.kBrushless);
  private CANSparkMax FlywheelMotorTwo = new CANSparkMax(CANBusIDs.k_FlywheelMotorTwo, MotorType.kBrushless);
  
  private CANPIDController m_flywheelMotorOnePID;

  /** Creates a new FlyWheelSubsystem. */
  public FlyWheelSubsystem() 
  {
    m_flywheelEncoder = FlywheelMotorOne.getEncoder();
    m_flywheelMotorOnePID = FlywheelMotorOne.getPIDController();
    configShooterMotors();
  }

  public void configShooterMotors()
    {
        FlywheelMotorOne.restoreFactoryDefaults();
        FlywheelMotorTwo.restoreFactoryDefaults();

        FlywheelMotorOne.setClosedLoopRampRate(FlywheelPIDValues.k_rampTime);
        FlywheelMotorTwo.setClosedLoopRampRate(FlywheelPIDValues.k_rampTime);

        FlywheelMotorTwo.follow(FlywheelMotorOne, true);

        m_flywheelMotorOnePID.setP(FlywheelPIDValues.k_flywheelP);
        m_flywheelMotorOnePID.setI(FlywheelPIDValues.k_flywheelI);
        m_flywheelMotorOnePID.setD(FlywheelPIDValues.k_flywheelD);
        m_flywheelMotorOnePID.setFF(FlywheelPIDValues.k_flywheelFF);
        m_flywheelMotorOnePID.setOutputRange(FlywheelPIDValues.k_minFlywheelOutput, FlywheelPIDValues.k_maxFlywheelOutput);

        FlywheelMotorOne.setIdleMode(IdleMode.kCoast);
        FlywheelMotorTwo.setIdleMode(IdleMode.kCoast);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void setFlywheelSpeed(double speed)
  {
    FlywheelMotorOne.set(speed);
  }

  public void stopFlyWheel()
  {
    FlywheelMotorOne.set(0);
  }
  
  public void setFlywheelRPM(int rpm)
  {
    m_flywheelMotorOnePID.setReference(rpm, ControlType.kVelocity);
  }

  public double getFlywheelVelocity()
    {
      return m_flywheelEncoder.getVelocity();
    }

  /*public void runFlyWheel()
    {
      FlywheelMotorOne.set(ShooterConstants.shooterMaxSpeed);
    }*/
}
