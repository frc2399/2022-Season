// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveForwardGivenTime;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SetShootPowerCmd;
import frc.robot.commands.Shift;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shooter;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  
// The robot's subsystems
    public final DriveTrain m_driveTrain = new DriveTrain();
    public final static Shifter m_shifter = new Shifter();
    public final static Intake m_intake = new Intake();
    public final static Shooter m_shooter = new Shooter();
    public final static Indexer m_indexer = new Indexer();

// Joysticks
  public static Joystick JOYSTICK = new Joystick(JoystickConstants.JOYSTICK_PORT);
  public static Joystick XBOX = new Joystick(JoystickConstants.XBOX_PORT);

  //Shift
  private static Shift shiftToDangerous = new Shift(!DriveConstants.SHIFTER_SOLENOID_HOT, DriveConstants.SHIFTER_SOLENOID_DANGEROUS);
  private static Shift shiftToHot = new Shift(DriveConstants.SHIFTER_SOLENOID_HOT,!DriveConstants.SHIFTER_SOLENOID_DANGEROUS);

  //Intake
  private static IntakeCmd intakeCmd = new IntakeCmd(m_intake, IntakeConstants.INTAKESPEED);

  //Indexer
  private static IndexerCmd indexerCmd = new IndexerCmd(m_indexer, IndexerConstants.INDEXERSPEED);



  //Buttons
  // private static Button shiftToHotButt = new JoystickButton(XBOX, 2);
	// private static Button shiftToDangerousButt = new Button(XBOX, 3);



  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private static RobotContainer m_robotContainer = new RobotContainer();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
    // Smartdashboard Subsystems
    SmartDashboard.putData(m_driveTrain);

    CameraServer.startAutomaticCapture();

    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("DriveForwardGivenTime: time", new DriveForwardGivenTime(1, m_driveTrain));

    // Configure the button bindings
    configureButtonBindings();
    
    System.out.println("Hello, I am in RobotContainer");

    m_driveTrain.setDefaultCommand(new ArcadeDriveCmd(m_driveTrain, //
    () -> -XBOX.getRawAxis(JoystickConstants.kArcadeDriveSpeedAxis),
    () -> XBOX.getRawAxis(JoystickConstants.kArcadeDriveTurnAxis))//
);

    m_intake.setDefaultCommand(new IntakeCmd(m_intake, 0));
    m_shooter.setDefaultCommand(new SetShootPowerCmd(m_shooter, 0, 0));
    m_indexer.setDefaultCommand(new IndexerCmd(m_indexer, 0));

    // Configure default commands


    // Configure autonomous sendable chooser

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
// Create some buttons

    

      // shiftToHotButt.whenPressed(shiftToHot);
		  // shiftToDangerousButt.whenPressed(shiftToDangerous);
      System.out.println (JOYSTICK);
      System.out.println ("************************************");
      new JoystickButton(JOYSTICK, JoystickConstants.kShiftHot)
      .whenPressed(shiftToHot);
      new JoystickButton(JOYSTICK, JoystickConstants.kShiftDangerous)
      .whenPressed(shiftToDangerous);

      new JoystickButton(JOYSTICK, JoystickConstants.INTAKE).whileHeld(intakeCmd);

      new JoystickButton(JOYSTICK, JoystickConstants.SHOOTER_BTN).whileHeld(
        //new SetShootPowerCmd(m_shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED)
        new InstantCommand(() -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT), m_shooter) 
      );
      new JoystickButton(JOYSTICK, JoystickConstants.INDEXER).whileHeld(indexerCmd);
    }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    //return m_chooser.getSelected();

    // return new ShootForNSeconds(m_shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, 5);
    return new SequentialCommandGroup(

      new InstantCommand(m_shooter::runShooter, m_shooter),
      new WaitCommand(5)
      ); 
 
    // return new SequentialCommandGroup(
    //   new IntakeCmdForGivenTime(m_intake, 1.0, 3),
    //   new IndexerCmdForGivenTime(m_indexer, 1.0, 3)

    // );
  }

}

