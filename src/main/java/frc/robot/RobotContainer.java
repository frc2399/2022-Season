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

import frc.robot.commands.*;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.subsystems.*;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



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

// Joysticks
  public static Joystick JOYSTICK = new Joystick(JoystickConstants.JOYSTICK_PORT);
  public static Joystick XBOX = new Joystick(JoystickConstants.XBOX_PORT);

  //Shift
  private static Shift shiftToDangerous = new Shift(!DriveConstants.SHIFTER_SOLENOID_HOT, DriveConstants.SHIFTER_SOLENOID_DANGEROUS);
  private static Shift shiftToHot = new Shift(DriveConstants.SHIFTER_SOLENOID_HOT,!DriveConstants.SHIFTER_SOLENOID_DANGEROUS);

  //Buttons
  // private static Button shiftToHotButt = new JoystickButton(XBOX, 2);
	// private static Button shiftToDangerousButt = new Button(XBOX, 3);

  private static RobotContainer m_robotContainer = new RobotContainer();


  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
    // Smartdashboard Subsystems
    SmartDashboard.putData(m_driveTrain);


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
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
  

}

