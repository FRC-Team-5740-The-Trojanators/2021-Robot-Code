// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveModuleConstants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterPIDValues;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  public double measuredX, tlong, thor, skewOffsetDegrees, actualXx;
  public final double pixelsToDegrees = .1419047619;

  private CANSparkMax flyWheelMotor;
  private TalonSRX hoodMotor;

  private CANSparkMax lowerShooterWheel = new CANSparkMax(CANBusIDs.k_lowerShooterWheel, MotorType.kBrushless);
  private CANSparkMax upperShooterWheel = new CANSparkMax(CANBusIDs.k_upperShooterWheel, MotorType.kBrushless);

  private NetworkTableEntry shuffleDistance;
  private NetworkTableEntry abs, quad, kp, ki, kd, kff, period, pos, setPoint, height;
  
  public ShooterSubsystem() {
    ledOff();
  }

  public void configShooterMotors()
  {
    lowerShooterWheel.restoreFactoryDefaults();
    upperShooterWheel.restoreFactoryDefaults();

    lowerShooterWheel.getPIDController().setP(ShooterPIDValues.k_shooterP);
    lowerShooterWheel.getPIDController().setI(ShooterPIDValues.k_shooterI);
    lowerShooterWheel.getPIDController().setD(ShooterPIDValues.k_shooterD);
    lowerShooterWheel.getPIDController().setFF(ShooterPIDValues.k_shooterFF);
    lowerShooterWheel.getPIDController().setOutputRange(ShooterPIDValues.k_minShooterOutput, ShooterPIDValues.k_maxShooterOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean seesTarget() {
    double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (seesTarget == 1.0) {
      return true;
    } else {
      return false;
    }
  }

  public double getHeadingToTarget() {
    double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (seesTarget == 1.0) {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)
          + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    } else {
      return 0.0;
    }
  }

  public double getSkew() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
  }

  public double getHeight() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getY() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }
 
  public void ledOn() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public void ledOff() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void setShooterSpeed(double speed)
    {
      lowerShooterWheel.set(speed);
    }

    public void stopShooter()
    {
      lowerShooterWheel.set(0);
    }
    
    public void setShooterRPM(int rpm)
    {
      lowerShooterWheel.getPIDController().setReference(rpm, ControlType.kVelocity);
    }
}
