// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.HIDConstants;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.AutonomousDrive2;
import frc.robot.commands.IntakeFlip;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.RotateRobot;
import frc.robot.commands.RunTalon;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.SwerveDriveCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.TalonTesting;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ForceExtendHood;
import frc.robot.commands.ForceRetractHood;
import frc.robot.commands.HoodDefaultCommand;
import frc.robot.commands.HoodMoveCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TargetCommand;
import frc.robot.pathsOLD.BarrelRacePath;
import frc.robot.pathsOLD.BlueAPath;
import frc.robot.pathsOLD.BlueBPath;
import frc.robot.pathsOLD.BouncePath;
import frc.robot.pathsOLD.BouncePath1;
import frc.robot.pathsOLD.FiveMeterPath;
import frc.robot.pathsOLD.RedAPath;
import frc.robot.pathsOLD.RedBPath;
import frc.robot.pathsOLD.SlalomPath;
import frc.robot.pathsOLD.TrajectoriesExporter;
import frc.robot.commands.FlyWheelCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here... 
    public final DriveSubsystem m_robotDrive = new DriveSubsystem(false);
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final AutonomousDrive m_autonomousDrive = new AutonomousDrive(m_robotDrive);
  //  private final AutonomousDrive2 m_autonomousDrive2 = new AutonomousDrive2(m_robotDrive);
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final HoodSubsystem m_hood = new HoodSubsystem();
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    private final FlyWheelSubsystem m_flywheel = new FlyWheelSubsystem();
    private final TalonTesting m_talon = new TalonTesting();

    // The driver's controller
    XboxController m_driverController = new XboxController(HIDConstants.k_DriverControllerPort);
    XboxController m_operatorController = new XboxController(HIDConstants.k_OperatorControllerPort);


    //Commands
    private final IntakeRun m_intakeRun = new IntakeRun(m_intake);
    private final IntakeReverse m_intakeReverse = new IntakeReverse(m_intake);
    private final TargetCommand m_target = new TargetCommand(m_shooter, m_robotDrive);
    private final IndexerCommand m_index = new IndexerCommand(m_indexer);
    private final HoodMoveCommand m_moveHood = new HoodMoveCommand(m_hood);
    //private final ForceExtendHood m_forceExtend = new ForceExtendHood(m_hood);
    //private final ForceRetractHood m_forceRetract = new ForceRetractHood(m_hood);
    private final FlyWheelCommand m_flyWheelCommand = new FlyWheelCommand(m_flywheel);
    private final RotateRobot m_rotateRobot = new RotateRobot(m_robotDrive, m_intake, m_shooter);
    //private final RunTalon m_runTalon = new RunTalon(m_talon, m_operatorController);
    
    //Command Groups
    private final ParallelCommandGroup spinupCommandGroup = new ParallelCommandGroup(m_flyWheelCommand, m_moveHood);
    //private final SequentialCommandGroup galacticSearchCommandGroup = new SequentialCommandGroup(m_rotateRobot, m_autonomousDrive);
   // private final SequentialCommandGroup autonomousCommandTestGroup = new SequentialCommandGroup(m_autonomousDrive, m_autonomousDrive2);

    //The Button Binding Names
    public static JoystickButton intakeFlip, intakeRun, intakeReverse, indexerRun, prepareShooter, TestButton;
    //POVButton  forceExtendHood, forceRetractHood;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    { 
        //  var traj = BouncePath1.getTrajectory();
        //  TrajectoriesExporter.exportTrajectoryToCSV(traj, BouncePath1.getTrajectoryName());
        //  TrajectoriesExporter.exportTrajectoryToHumanReadable(traj, BouncePath1.getTrajectoryName());

        configureButtonBindings();
        //m_robotDrive.setDefaultCommand(new SwerveDriveCommand(m_robotDrive, m_driverController));
        m_hood.setDefaultCommand(new HoodDefaultCommand(m_hood));
        m_talon.setDefaultCommand(new RunTalon(m_talon));

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        m_robotDrive.resetIMU();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        intakeFlip = new JoystickButton(m_driverController , HIDConstants.kA);
        intakeRun = new JoystickButton(m_driverController , HIDConstants.kLB);
        intakeReverse = new JoystickButton(m_driverController, HIDConstants.kStart);
        indexerRun = new JoystickButton(m_driverController, HIDConstants.kX);
        prepareShooter = new JoystickButton(m_driverController, HIDConstants.kRB);
        //forceExtendHood = new POVButton(m_driverController, HIDConstants.kDL);
        //forceRetractHood = new POVButton(m_driverController, HIDConstants.kDR);
        //TestButton = new JoystickButton(m_driverController, HIDConstants.kLB);

        intakeFlip.toggleWhenPressed(new StartEndCommand(m_intake::retractIntake, m_intake::extendIntake, m_intake));
        intakeRun.whileHeld(m_intakeRun);
        intakeReverse.whileHeld(m_intakeReverse);
        //forceExtendHood.whileHeld(m_forceExtend);
        //forceRetractHood.whileHeld(m_forceRetract);
        prepareShooter.whileHeld(spinupCommandGroup);
        //prepareShooter.whileHeld(m_flyWheelCommand);
        prepareShooter.whenActive(m_target);
        indexerRun.whileHeld(m_index);
        //TestButton.whileHeld(m_moveHood);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        //return galacticSearchCommandGroup;
       // return autonomousCommandTestGroup;
        return m_autonomousDrive;

    }
}