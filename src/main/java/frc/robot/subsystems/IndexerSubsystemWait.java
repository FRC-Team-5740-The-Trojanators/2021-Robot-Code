// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveModuleConstants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants.ShooterConstants;

public class IndexerSubsystemWait extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */
  private VictorSPX indexerWheel;

  public IndexerSubsystemWait() 
  {
    indexerWheel = new VictorSPX(CANBusIDs.k_IndexerID);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
