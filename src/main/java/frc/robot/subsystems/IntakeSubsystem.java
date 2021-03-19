// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonSRX m_intakeMotorLeft;
  private final TalonSRX m_intakeMotorRight;
  private final Compressor intakeCompressor = new Compressor(0); //fake address
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(1, 3); //fake address



  public IntakeSubsystem() {
     m_intakeMotorLeft = new TalonSRX(1); //fake DeviceID
     m_intakeMotorRight = new TalonSRX(5); //fake DeviceID
     intakeSolenoid.set(kOff);


     intakeCompressor.setClosedLoopControl(true);

     boolean enabled = intakeCompressor.enabled();
     boolean pressureSwitch = intakeCompressor.getPressureSwitchValue();
     double current = intakeCompressor.getCompressorCurrent();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isIntakeOut()
  {
    if(intakeSolenoid.get() == kForward)
    {
      return true;
    } else
    {
      return false;
    }
  }

  public void flipIntakeOut()
  {
      intakeSolenoid.set(kForward);
  }

  public void flipIntakeIn()
  {
    intakeSolenoid.set(kReverse);
  }

  
}
