// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestShooter extends SubsystemBase 
{
    private final Spark shooterMotor1 = new Spark(Constants.shooterMotor1PWMChannel);
    private final Spark shooterMotor2 = new Spark(Constants.shooterMotor2PWMChannel);

    private final Encoder motor1Feedback = new Encoder(0, 1);

    /** Creates a new TestShooter. */
    public TestShooter() 
    {
        Spark.checkMotors();
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
    }
}
