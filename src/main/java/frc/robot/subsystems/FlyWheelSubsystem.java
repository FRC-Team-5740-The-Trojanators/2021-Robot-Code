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
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterPIDValues;

public class FlyWheelSubsystem extends SubsystemBase {

  
  private CANEncoder m_shooterEncoder;

  private CANSparkMax ShooterMotorOne = new CANSparkMax(CANBusIDs.k_ShooterMotorOne, MotorType.kBrushless);
  private CANSparkMax ShooterMotorTwo = new CANSparkMax(CANBusIDs.k_ShooterMotorTwo, MotorType.kBrushless);
  
  private CANPIDController m_ShooterMotorOnePID;

  /** Creates a new FlyWheelSubsystem. */
  public FlyWheelSubsystem() 
  {
    m_shooterEncoder = ShooterMotorOne.getEncoder();
    m_ShooterMotorOnePID = ShooterMotorOne.getPIDController();
    configShooterMotors();
  }

  public void configShooterMotors()
    {
        ShooterMotorOne.restoreFactoryDefaults();
        ShooterMotorTwo.restoreFactoryDefaults();

        ShooterMotorTwo.follow(ShooterMotorOne, true);

        m_ShooterMotorOnePID.setP(ShooterPIDValues.k_shooterP);
        m_ShooterMotorOnePID.setI(ShooterPIDValues.k_shooterI);
        m_ShooterMotorOnePID.setD(ShooterPIDValues.k_shooterD);
        m_ShooterMotorOnePID.setOutputRange(ShooterPIDValues.k_minShooterOutput, ShooterPIDValues.k_maxShooterOutput);

        ShooterMotorOne.setIdleMode(IdleMode.kCoast);
        ShooterMotorTwo.setIdleMode(IdleMode.kCoast);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void setShooterSpeed(double speed)
  {
    ShooterMotorOne.set(speed);
  }

  public void stopShooter()
  {
    ShooterMotorOne.set(0);
  }
  
  public void setShooterRPM(int rpm)
  {
    m_ShooterMotorOnePID.setReference(rpm, ControlType.kVelocity);
  }

  public double getShooterVelocity()
    {
      return m_shooterEncoder.getVelocity();
    }

  public void runFlyWheel()
    {
      ShooterMotorOne.set(ShooterConstants.shooterMaxSpeed);
    }

    public void stopFlyWheel()
    {
      ShooterMotorOne.set(0);
    }
}
