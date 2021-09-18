// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.CANBusIDs;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final VictorSPX m_intakeMotor;
  private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(1, 0);

  public IntakeSubsystem() {
     m_intakeMotor = new VictorSPX(CANBusIDs.k_IntakeMotors);
     m_intakeSolenoid.set(kOff);
     //stopIntakeMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isIntakeOut()
  {
    if(m_intakeSolenoid.get() == kForward)
    {
      return true;
    } else
    {
      return false;
    }
  }

  public void extendIntake()
  {
      m_intakeSolenoid.set(kReverse);
  }

  public void retractIntake()
  {
    m_intakeSolenoid.set(kForward);
  }

  public void startIntakeMotors()
  {
    m_intakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeSubsystemConstants.k_intakeMotorSpeed);
  }

  public void reverseIntakeMotors()
  {
    m_intakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeSubsystemConstants.k_intakeReverseMotorSpeed);
  }

  public void stopIntakeMotors()
  {
    m_intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }
}
