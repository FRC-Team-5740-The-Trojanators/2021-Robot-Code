// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
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
import frc.robot.Constants.SwerveDriveModuleConstants.HexEncoderInputs;
import frc.robot.Constants.SwerveDriveModuleConstants.HoodConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterPIDValues;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  public double measuredX, tlong, thor, skewOffsetDegrees, actualXx;
  public final double pixelsToDegrees = .1419047619;

  private TalonSRX hoodMotor;

  private CANEncoder m_shooterEncoder;

  private CANSparkMax ShooterMotorOne = new CANSparkMax(CANBusIDs.k_ShooterMotorOne, MotorType.kBrushless);
  private CANSparkMax ShooterMotorTwo = new CANSparkMax(CANBusIDs.k_ShooterMotorTwo, MotorType.kBrushless);

  private VictorSPX indexerWheel = new VictorSPX(CANBusIDs.k_indexerID);

  private NetworkTableEntry shuffleDistance;
  private NetworkTableEntry abs, quad, kp, ki, kd, kff, period, pos, setPoint, height;
  
  private PIDController m_hoodPID;
  private PIDController m_aimPID;
  private CANPIDController m_ShooterMotorOnePID = ShooterMotorOne.getPIDController();

  private double m_txRad;
  private double m_ty;
  private int m_hoodTicks;

  private DutyCycleEncoder m_hexAbsoluteEncoder = new DutyCycleEncoder(HexEncoderInputs.k_absoluteInput);
  private Encoder m_hexQuadEncoder = new Encoder(HexEncoderInputs.k_quadratureA, HexEncoderInputs.k_quadratureB, HexEncoderInputs.k_indexInput);
  
  public ShooterSubsystem()
  {
    ledOff();
    m_hoodPID = new PIDController(HoodConstants.k_hoodP, HoodConstants.k_hoodI, HoodConstants.k_hoodD, HoodConstants.k_hoodFF);
    m_aimPID = new PIDController(ShooterPIDValues.k_aimingP, ShooterPIDValues.k_aimingI, ShooterPIDValues.k_aimingD);
    configShooterMotors();
    m_shooterEncoder = ShooterMotorOne.getEncoder();

    hoodMotor.configClosedloopRamp(ShooterConstants.k_rampRate); //Needs actual value
    hoodMotor.configOpenloopRamp(ShooterConstants.k_rampRate); 
    hoodMotor.setNeutralMode(NeutralMode.Brake); 

    m_hexQuadEncoder.reset();
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
    m_hoodTicks = m_hexQuadEncoder.get();
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

  public double hoodSetSetpoint(double setpoint)
  {
    //m_hoodPID.setSetpoint(setpoint);
   return m_hoodPID.calculate(m_hoodTicks, setpoint);
  }

  public void setHoodMotor(double demand)
  {
    hoodMotor.set(TalonSRXControlMode.PercentOutput, demand);
  }

  public void forceRunHoodMotorExtend()
  {
    hoodMotor.set(TalonSRXControlMode.PercentOutput, 0.1);
  }

  public void forceRunHoodMotorRetract()
  {
    hoodMotor.set(TalonSRXControlMode.PercentOutput, -0.1);
  }

  public void forceStopHoodMotor()
  {
    hoodMotor.set(TalonSRXControlMode.PercentOutput, 0);
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
      indexerWheel.set(ControlMode.PercentOutput, ShooterConstants.indexerMaxSpeed);
    }

    public void stopIndexerWheel(){
      indexerWheel.set(ControlMode.PercentOutput, 0);
    }

    public double getIndexerVelocity()
    {
      return indexerWheel.getMotorOutputPercent();
    }

    public double getShooterVelocity()
    {
      return m_shooterEncoder.getVelocity();
    }

    public double turnShooter()
    {
      double rot = SwerveDriveModuleConstants.kMaxAngularSpeed * SwerveDriveModuleConstants.kRotCoefficent * getAimPID();
      return rot;
    }

    public double actuateHood()
    {
      double angle = m_ty + HoodConstants.limelightAngle;
      var distance = HoodConstants.heightDifference / Math.tan(angle);

      if(distance >= HoodConstants.k_maxDistance)
      {
       return hoodSetSetpoint(HoodConstants.k_retractSetpoint);
      }
      else if( distance < HoodConstants.k_maxDistance && distance >= 20)
      {
        return hoodSetSetpoint(HoodConstants.k_retractSetpoint + 50);
      } 
      else if(distance < 20 && distance >= 10)
      {
        return hoodSetSetpoint(HoodConstants.k_retractSetpoint + 100);
      } 
      else if(distance < 10)
      {
        return hoodSetSetpoint(HoodConstants.k_retractSetpoint + 150);
      }
      else
      {
        return -1; //error indication, it should never get here
      }
    }

    public void runFlyWheel(){
      ShooterMotorOne.set(ShooterConstants.shooterMaxSpeed);
    }
    public void stopFlyWheel(){
      ShooterMotorOne.set(0);
    }

  }
