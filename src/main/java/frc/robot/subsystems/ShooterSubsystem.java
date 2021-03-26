// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants.HoodConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterPIDValues;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  public double measuredX, tlong, thor, skewOffsetDegrees, actualXx;
  public final double pixelsToDegrees = .1419047619;

  private TalonSRX hoodMotor;

  private CANSparkMax ShooterMotorOne = new CANSparkMax(CANBusIDs.k_ShooterMotorOne, MotorType.kBrushless);
  private CANSparkMax ShooterMotorTwo = new CANSparkMax(CANBusIDs.k_ShooterMotorTwo, MotorType.kBrushless);

  private VictorSPX indexerWheel;

  private NetworkTableEntry shuffleDistance;
  private NetworkTableEntry abs, quad, kp, ki, kd, kff, period, pos, setPoint, height;
  
  private PIDController m_hoodPID;
  private PIDController m_aimPID;
  private CANPIDController m_ShooterMotorOnePID = ShooterMotorOne.getPIDController();


  private double m_txRad;
  private double m_ty;

  public ShooterSubsystem()
  {
    ledOff();
    m_hoodPID = new PIDController(HoodConstants.k_hoodP, HoodConstants.k_hoodI, HoodConstants.k_hoodD, HoodConstants.k_hoodFF);
    m_aimPID = new PIDController(ShooterPIDValues.k_aimingP, ShooterPIDValues.k_aimingI, ShooterPIDValues.k_aimingD);
  }

  public void configShooterMotors()
  {
    ShooterMotorOne.restoreFactoryDefaults();
    ShooterMotorTwo.restoreFactoryDefaults();

    ShooterMotorTwo.follow(ShooterMotorOne, true);


    m_ShooterMotorOnePID.setP(ShooterPIDValues.k_shooterP);
    m_ShooterMotorOnePID.setI(ShooterPIDValues.k_shooterI);
    m_ShooterMotorOnePID.setD(ShooterPIDValues.k_shooterD);
    m_ShooterMotorOnePID.setFF(ShooterPIDValues.k_shooterFF);
    m_ShooterMotorOnePID.setOutputRange(ShooterPIDValues.k_minShooterOutput, ShooterPIDValues.k_maxShooterOutput);

    ShooterMotorOne.setIdleMode(IdleMode.kCoast);
    ShooterMotorTwo.setIdleMode(IdleMode.kCoast);

    
   
    m_aimPID.setTolerance(ShooterPIDValues.k_aimTolerance);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    
    m_ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    m_txRad = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) * (Math.PI/180);
  }

  public double getAimPID(){
    return m_aimPID.calculate(m_txRad, 0);
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

  public void hoodSetSetpoint(double setpoint)
  {
    m_hoodPID.setSetpoint(setpoint);
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
      ShooterMotorOne.set(speed);
    }

    public void stopShooter()
    {
      ShooterMotorOne.set(0);
    }
    
    public void setShooterRPM(int rpm)
    {
      ShooterMotorOne.getPIDController().setReference(rpm, ControlType.kVelocity);
    }

    public void runIndexerWheel(){
      indexerWheel.set(ControlMode.PercentOutput, 1);
    }

    public double turnShooter()
    {
      double rot = SwerveDriveModuleConstants.kMaxAngularSpeed * SwerveDriveModuleConstants.kRotCoefficent *  getAimPID();
      return rot;
    }

    public void actuateHood()
    {
      double angle = m_ty + HoodConstants.limelightAngle;
      var distance = HoodConstants.heightDifference / Math.tan(angle);

      if(distance >= HoodConstants.k_maxDistance)
      {
       hoodSetSetpoint(HoodConstants.k_retractSetpoint);
      }
      else if( distance < HoodConstants.k_maxDistance && distance >= 20)
      {
        hoodSetSetpoint(HoodConstants.k_retractSetpoint + 50);
      } 
      else if(distance < 20 && distance >= 10)
      {
        hoodSetSetpoint(HoodConstants.k_retractSetpoint + 100);
      } 
      else if(distance < 10)
      {
        hoodSetSetpoint(HoodConstants.k_retractSetpoint + 150);
      }
    }
    public void runFlyWheel(){
      m_ShooterMotorOnePID.setReference(ShooterPIDValues.k_speedRPM, ControlType.kVelocity);
    }
}
