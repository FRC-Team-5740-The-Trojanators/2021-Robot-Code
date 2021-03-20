// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.VelocityPeriod;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final VictorSPX m_intakeMotor;
  private final Compressor m_intakeCompressor = new Compressor(0); //fake address
  private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(1, 3); //fake address

  public IntakeSubsystem() {
     m_intakeMotor = new VictorSPX(99); //fake DeviceID
     m_intakeSolenoid.set(kOff);

     m_intakeCompressor.setClosedLoopControl(true);

     boolean enabled = m_intakeCompressor.enabled();
     boolean pressureSwitch = m_intakeCompressor.getPressureSwitchValue();
     double current = m_intakeCompressor.getCompressorCurrent();

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
      m_intakeSolenoid.set(kForward);
  }

  public void retractIntake()
  {
    m_intakeSolenoid.set(kReverse);
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
