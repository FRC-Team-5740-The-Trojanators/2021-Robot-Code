package subsystems;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.internal.progress.MockingProgressImpl;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.mockito.Mockito.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.Constants.SwerveDriveModuleConstants.SteeringControllerPIDValues;
import frc.robot.subsystems.SwerveModule;

public class SwerveModuleSubsystemTests
{
    CANSparkMax m_driveMotor;
    CANSparkMax m_angleMotor;
    CANCoder m_canCoder; 
    Rotation2d m_offset;

    SwerveModule m_sm;

    private void setup()
    {
        System.out.println("start here");

        m_driveMotor = mock(CANSparkMax.class);
        m_angleMotor = mock(CANSparkMax.class);
        m_canCoder = mock(CANCoder.class);
        m_offset = new Rotation2d();

        when(m_driveMotor.getPIDController()).thenReturn(mock(CANPIDController.class));
        when(m_driveMotor.getEncoder()).thenReturn(mock(CANEncoder.class));

        m_sm = new SwerveModule(m_driveMotor, m_angleMotor, m_canCoder, m_offset);
    }

    @Test
    public void GetState_Returns_Valid_States()
    {
        // System.out.println("start here");

        // m_driveMotor = mock(CANSparkMax.class);
        // m_angleMotor = mock(CANSparkMax.class);
        // m_canCoder = mock(CANCoder.class);
        // m_offset = new Rotation2d();

        // when(m_driveMotor.getPIDController()).thenReturn(mock(CANPIDController.class));
        // when(m_driveMotor.getEncoder()).thenReturn(mock(CANEncoder.class));

        // SwerveModule m_sm = new SwerveModule(m_driveMotor, m_angleMotor, m_canCoder, m_offset);
        setup();
        
        var enc = m_driveMotor.getEncoder();

        when(enc.getVelocity()).thenReturn(100.0);
        when(m_canCoder.getPosition()).thenReturn(45.0);


        var x = m_sm.getState();
        // System.out.println(x.speedMetersPerSecond);
         System.out.println(x.angle);

        assertEquals(100.0, x.speedMetersPerSecond);
        assertEquals(45.0, x.angle.getDegrees());
    }

    @Test
    public void GetAngle_Provides_Valid_Data()
    {
        // Arrange
        setup();

        // Act
        when(m_canCoder.getAbsolutePosition()).thenReturn(280.0);
        var pos = m_canCoder.getAbsolutePosition();

        // Assert
        assertEquals(280.0, pos);
    }

    @Test
    public void setDesiredState_Provides_Valid_Data()
    {
        // Arrange
        setup();
                                            // This might need tweaking
        var ds = new SwerveModuleState(1.0, new Rotation2d(0.5, 0.5));
        var curRot = new Rotation2d(280.0);

        //Mocking static classes is a code smell, FYI
        
        MockedStatic<SwerveModuleState> sms = Mockito.mockStatic(SwerveModuleState.class);

        when(m_canCoder.getAbsolutePosition()).thenReturn(280.0);
        

        //var newSMS = new SwerveModuleState(1.0, new Rotation2d(280.0));
        //when(sms.optimize()).thenReturn(newSMS);
        sms.when(() -> 
            SwerveModuleState.optimize(new SwerveModuleState(1.0, new Rotation2d(280.0)), new Rotation2d(180.0)))
                .thenReturn(new SwerveModuleState(-1.0, new Rotation2d(95.0)));


        // Act
        
        //var pos = m_canCoder.getAbsolutePosition();

        // Assert
       // assertEquals(280.0, pos);
    }

    @Test
    public void calculateDeltaTicks_At0deg_ReturnsCorrectValue()
    {
        // Arrange
        setup();
        var rD = new Rotation2d(0.0);

        // Act
        var result = m_sm.calculateDeltaTicks(rD);


        // Assert (expected, equals)
        assertEquals(0.0, result);
    }

    @Test
    public void calcuateDeltaTicks_At180deg_ReturnsCorrectValue()
    {
        // Arrange
        setup();
        var rD = new Rotation2d(Math.PI);

        // Act
        var result = m_sm.calculateDeltaTicks(rD);
        var expected = (180.0 / 360.0) * SwerveDriveModuleConstants.kEncoderTicksPerRotation;

        //Assert
        assertEquals(expected, result);
    }


    @Test
    public void calcuateDeltaTicks_At270deg_ReturnsCorrectValue()
    {
        // Arrange
        setup();
        var rD = new Rotation2d(Math.PI * 1.5); // 270 is 180 + half again as much

        // Act
        var result = m_sm.calculateDeltaTicks(rD);
        var expected = (270.0 / 360.0) * SwerveDriveModuleConstants.kEncoderTicksPerRotation;

        //Assert
        assertEquals(expected, result);
    }

    @Test
    public void filterAngleMotorDeadband_LTDeadband_Returns0()
    {
        // Arrange
        setup();
        double angle = 0.0;

        // Act
        var result = m_sm.filterAngleMotorDeadband(angle);

        // Assert
        assertEquals(angle, result);
    }

    @Test
    public void filterAngleMotorDeadband_GTDeadband_ReturnsAngle()
    {
        setup();
        double angle = 20;

        // Act
        var result = m_sm.filterAngleMotorDeadband(angle);

        // Assert
        assertEquals(angle, result);
    }

    @Test
    public void filterAngleMotorDeadband_EQDeadband_Returns0()
    {
        setup();
        double angle = SteeringControllerPIDValues.k_steerDeadband;

        // Act
        var result = m_sm.filterAngleMotorDeadband(angle);
        var expected = 0.0;

        // Assert
        assertEquals(expected, result);
    }

    @Test
    public void calculateCurrentTicks_ValidInput_ValidOutput()
    {
        //Arrange
        setup();
        when(m_canCoder.getPosition()).thenReturn(10.0);
        when(m_canCoder.configGetFeedbackCoefficient()).thenReturn(10.0);
//        when(m_driveMotor.getPIDController()).thenReturn(mock(CANPIDController.class));

        //Act
        var result = m_sm.calculateCurrentTicks();
        var expected = 1.0;

        //Assert
        assertEquals(expected, result);

    }

    @Test
    public void validateDesiredTicks()
    {
        //Arrange
        setup();
        when(m_canCoder.getPosition()).thenReturn(10.0);
        when(m_canCoder.configGetFeedbackCoefficient()).thenReturn(10.0);
        //Act
        var result = m_sm.calculateCurrentTicks() + m_sm.calculateDeltaTicks(Rotation2d.fromDegrees(180));
        var expected = 2049.0;
        //Assert
        assertEquals(expected, result);
    }

}
