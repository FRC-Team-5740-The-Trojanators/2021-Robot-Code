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

  private TalonSRX hoodMotor = new TalonSRX(CANBusIDs.k_HoodID);
  private PIDController m_hoodPID;
  private DutyCycleEncoder m_hexAbsoluteEncoder = new DutyCycleEncoder(HexEncoderInputs.k_absoluteInput);
  private Encoder m_hexQuadEncoder = new Encoder(HexEncoderInputs.k_quadratureA, HexEncoderInputs.k_quadratureB);

  private double m_hoodRotations;
  private static double m_quadOffset;

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() 
  {
    m_hoodPID = new PIDController(HoodConstants.k_hoodP, HoodConstants.k_hoodI, HoodConstants.k_hoodD);
    m_hoodPID.setTolerance(HoodConstants.k_hoodTolerance);

    hoodMotor.configOpenloopRamp(ShooterConstants.k_rampRate);
    hoodMotor.setNeutralMode(NeutralMode.Brake); 

    m_hexQuadEncoder.setReverseDirection(true);
    m_hexQuadEncoder.reset();
    m_hoodPID.disableContinuousInput();
    adjustHood();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_hoodRotations = m_hexAbsoluteEncoder.get();
    SmartDashboard.putNumber("Hood Abs Encoder", getAbsEncoder());
    SmartDashboard.putNumber("Hood Quad Encoder", (double) getQuadEncoder() + m_quadOffset);
    SmartDashboard.putNumber("QuadOffset", m_quadOffset);
  }

  public double hoodSetSetpoint(double setpoint)
  {
    return m_hoodPID.calculate((double) getQuadEncoder() + m_quadOffset, setpoint);
  }

  public boolean hoodMoveEnd()
  {
    return m_hoodPID.atSetpoint();
  }

    public void setHoodMotor(double demand)
    {
        if(demand > HoodConstants.k_hoodAutoSpeed)
        {
            demand = HoodConstants.k_hoodAutoSpeed;
        } 
        if(demand < -HoodConstants.k_hoodAutoSpeed)
        {
            demand = -HoodConstants.k_hoodAutoSpeed;
        }
            
        hoodMotor.set(TalonSRXControlMode.PercentOutput, demand);    
    }

    public void adjustHood()
    {
        m_hoodRotations = m_hexAbsoluteEncoder.get();
        m_quadOffset = (m_hoodRotations - HoodConstants.k_retractAbsSetpoint) * HoodConstants.k_quadTicksPerRotation;
    }

    public void forceRunHoodMotorExtend()
    {
        hoodMotor.set(TalonSRXControlMode.PercentOutput, HoodConstants.k_hoodExtendSpeed);
    }

    public void forceRunHoodMotorRetract()
    {
        hoodMotor.set(TalonSRXControlMode.PercentOutput, HoodConstants.k_hoodRetractSpeed);
    }

    public void forceStopHoodMotor()
    {
        hoodMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public double getAbsEncoder()
    {
        return m_hexAbsoluteEncoder.get();
    }

    public int getQuadEncoder()
    {
        return m_hexQuadEncoder.get();
    }

    public double hoodAngleFinder(double limelight_ty)
    {
        /*    d = (h1-h2) / tan(a1+a2)    */
        //angle is in degrees
        double angle = limelight_ty + HoodConstants.limelightAngle;
        angle *= (Math.PI / 180); // convert to radians
        //distance is in inches, angle radians
        var distance = HoodConstants.heightDifference / Math.tan(angle);

        SmartDashboard.putNumber("hood angle", angle);
        SmartDashboard.putNumber("distance inches", distance);

        if (distance > 283)
        {
            distance = 283;
        }
        
        if (distance < 50)
        {
            distance = 50;
        }

        double hoodPosition;

        //hoodPosition = 0.000002775*(Math.pow(distance, 4)) - 0.0013*(Math.pow(distance, 3)) + 0.081*(Math.pow(distance, 2)) + 28.303*distance - 60.17;
        hoodPosition = 0.0006*(Math.pow(distance, 3)) - 0.3463*(Math.pow(distance, 2)) + 64.618*distance - 1144.3;

      // hoodPosition = 2760;
        SmartDashboard.putNumber("Hood Commanded Position", hoodPosition);

        return hoodPosition;
    }
}
