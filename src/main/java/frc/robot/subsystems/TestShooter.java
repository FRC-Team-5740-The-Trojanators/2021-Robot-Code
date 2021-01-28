// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



public class TestShooter extends SubsystemBase 
{
    private final Spark shooterMotor1 = new Spark();

    /** Creates a new TestShooter. */
    public TestShooter() 
    {

    }

    @Override
    public void periodic() 
    {
        System.out.println("x");
        // This method will be called once per scheduler run
    }
}

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;