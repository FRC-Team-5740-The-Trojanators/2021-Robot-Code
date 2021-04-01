// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveModuleConstants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants.HexEncoderInputs;
import frc.robot.Constants.SwerveDriveModuleConstants.HoodConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase {

  private TalonSRX hoodMotor = new TalonSRX(CANBusIDs.k_hoodID);
  private PIDController m_hoodPID;
  private DutyCycleEncoder m_hexAbsoluteEncoder = new DutyCycleEncoder(HexEncoderInputs.k_absoluteInput);
  private Encoder m_hexQuadEncoder = new Encoder(HexEncoderInputs.k_quadratureA, HexEncoderInputs.k_quadratureB, HexEncoderInputs.k_indexInput);

  private double m_hoodRotations;

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() 
  {
    m_hoodPID = new PIDController(HoodConstants.k_hoodP, HoodConstants.k_hoodI, HoodConstants.k_hoodD);

    hoodMotor.configOpenloopRamp(ShooterConstants.k_rampRate); 
    hoodMotor.setNeutralMode(NeutralMode.Brake); 

    m_hexAbsoluteEncoder.reset();
    m_hoodPID.disableContinuousInput();
    adjustHood();
    m_hoodPID.enableContinuousInput(HoodConstants.k_retractSetpoint, HoodConstants.k_extendSetpoint);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_hoodRotations = m_hexAbsoluteEncoder.get();
    SmartDashboard.putNumber("Hood Encoder", getAbsEncoder());
  }

  public double hoodSetSetpoint(double setpoint)
    {
        return m_hoodPID.calculate(m_hoodRotations, setpoint);
    }

    public void setHoodMotor(double demand)
    {
        if(demand > HoodConstants.k_hoodExtendSpeed)
        {
            demand = HoodConstants.k_hoodExtendSpeed;
        } 
        if(demand < HoodConstants.k_hoodRetractSpeed)
        {
            demand = HoodConstants.k_hoodRetractSpeed;
        }
            
        hoodMotor.set(TalonSRXControlMode.PercentOutput, demand);    
    }

    public void adjustHood()
    {
        if(m_hexAbsoluteEncoder.get() < HoodConstants.k_retractSetpoint)
        {
            double softLimit = m_hoodPID.calculate(m_hoodRotations, HoodConstants.k_retractSetpoint);
           // hoodMotor.set(TalonSRXControlMode.PercentOutput, softLimit);
        }
        
    }

    public void forceRunHoodMotorExtend()
    {
        hoodMotor.setInverted(false);
        hoodMotor.set(TalonSRXControlMode.PercentOutput, HoodConstants.k_hoodExtendSpeed);
    }

    public void forceRunHoodMotorRetract()
    {
        hoodMotor.setInverted(true);
        hoodMotor.set(TalonSRXControlMode.PercentOutput, HoodConstants.k_hoodExtendSpeed);
    }

    public void forceStopHoodMotor()
    {
        hoodMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public double getAbsEncoder()
    {
        return m_hexAbsoluteEncoder.get();
    }

    public double hoodAngleFinder(double limelight_ty)
    {
        double angle = limelight_ty + HoodConstants.limelightAngle;
        var distance = HoodConstants.heightDifference / Math.tan(angle);

        if(distance >= HoodConstants.k_maxDistance)
        {
        return hoodSetSetpoint(HoodConstants.k_retractSetpoint + .5);
        }
        else if( distance < HoodConstants.k_maxDistance && distance >= HoodConstants.k_redZoneDistance)
        {
            return hoodSetSetpoint(HoodConstants.k_redEncoder);
        } 
        else if(distance < HoodConstants.k_redZoneDistance && distance >= HoodConstants.k_blueZoneDistance)
        {
            return hoodSetSetpoint(HoodConstants.k_blueEncoder);
        } 
        else if(distance < HoodConstants.k_blueZoneDistance && distance >= HoodConstants.k_yellowZoneDistance)
        {
            return hoodSetSetpoint(HoodConstants.k_yellowEncoder);
        } 
        else if(distance < HoodConstants.k_yellowZoneDistance && distance >= HoodConstants.k_greenZoneDistance)
        {
            return hoodSetSetpoint(HoodConstants.k_greenEncoder);
        } 
        else if(distance < HoodConstants.k_yellowZoneDistance && distance >= HoodConstants.k_greenZoneDistance)
        {
            return hoodSetSetpoint(HoodConstants.k_closestEncoder);
        } 
        else
        {
            return -1; //error indication, it should never get here
        }
    }
}
