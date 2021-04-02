// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.FlywheelPIDValues;

public class ShooterSubsystem extends SubsystemBase 
{
    /** Creates a new ShooterSubsystem. */
    public double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    public double measuredX, tlong, thor, skewOffsetDegrees, actualXx;
    public final double pixelsToDegrees = .1419047619;

    private PIDController m_aimPID;

    private double m_txRad;
    private double m_ty;

    public ShooterSubsystem()
    {
        ledOff();
        m_aimPID = new PIDController(FlywheelPIDValues.k_aimingP, FlywheelPIDValues.k_aimingI, FlywheelPIDValues.k_aimingD);
        m_aimPID.setTolerance(FlywheelPIDValues.k_aimTolerance);
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        m_ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        m_txRad = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) * (Math.PI/180);
    }

    public double getAimPID()
    {
        return m_aimPID.calculate(m_txRad, 0);
    }

    public double getLimelightTY()
    {
        return m_ty;
    }

  public boolean seesTarget() 
  {
    double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (seesTarget == 1.0) 
    {
        return true;
    } 
    else 
    {
        return false;
    }
  }

    public double getHeadingToTarget() 
    {
        double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        if (seesTarget == 1.0) 
        {
            return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)
                + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
        } 
        else 
        {
            return 0.0;
        }
    }

    public boolean aimEnd()
    {
        return m_aimPID.atSetpoint();
    }

    public double getSkew() 
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    }

    public double getHeight()
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    public double getX()
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getY()
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }
    
    public void ledOn()
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void ledOff()
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public double turnShooter()
    {
      double rot = SwerveDriveModuleConstants.kMaxAngularSpeed * getAimPID();
      return rot;
    }
}
