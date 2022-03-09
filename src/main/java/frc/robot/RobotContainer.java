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

import java.util.Map;

//import edu.wpi.first.cameraserver.CameraServer;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.IntakeBallShootBothP1;
import frc.robot.commands.autonomous.JellyStrawberryAuton;
import frc.robot.commands.autonomous.Position2Auton;
import frc.robot.commands.autonomous.Position3Auton;
import frc.robot.commands.autonomous.Position4AutonStale;
import frc.robot.commands.autonomous.Position5AutonPB;
import frc.robot.subsystems.*;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.XboxConstants;
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
    public final static DriveTrain m_driveTrain = new DriveTrain();
    //public final DriveTrain m_driveTrain = new DriveTrain();
    public final static Shifter m_shifter = new Shifter();
    public final static Intake m_intake = new Intake();
    public final static Shooter m_shooter = new Shooter();
    public final static Indexer m_indexer = new Indexer();
    public static final Climber m_climber = new Climber();
    public final static Limelight m_limelight = new Limelight();
    public final static PhotonLimelight m_photonLimelight = new PhotonLimelight();

// Joysticks
  public static Joystick JOYSTICK = new Joystick(JoystickConstants.JOYSTICK_PORT);
  public static Joystick XBOX = new Joystick(XboxConstants.XBOX_PORT);

  //Shift
  private static Shift shiftToDangerous = new Shift(!DriveConstants.SHIFTER_SOLENOID_HOT, DriveConstants.SHIFTER_SOLENOID_DANGEROUS);
  private static Shift shiftToHot = new Shift(DriveConstants.SHIFTER_SOLENOID_HOT,!DriveConstants.SHIFTER_SOLENOID_DANGEROUS);

  //Intake
  private static IntakeCmd intakeCmd = new IntakeCmd(m_intake, IntakeConstants.INTAKESPEED);

  //Indexer
  private static IndexerCmd indexerFwdCmd = new IndexerCmd(m_indexer, IndexerConstants.INDEXERSPEED);
  private static IndexerCmd indexerBackCmd = new IndexerCmd(m_indexer, -IndexerConstants.INDEXERSPEED);

  //Climber
  private static ExtendClimber extendClimberCmd = new ExtendClimber(m_climber, 0.5);


  public static TurnToNAngle m_turnToNAngle = new TurnToNAngle(0, m_driveTrain);

  public static TurnNAngle m_turnRight = new TurnNAngle(90, m_driveTrain);
  public static TurnNAngle m_turnLeft = new TurnNAngle(-90, m_driveTrain);
  public static TurnNAngle m_turn180 = new TurnNAngle(180, m_driveTrain);

  //Buttons
  // private static Button shiftToHotButt = new JoystickButton(XBOX, 2);
	// private static Button shiftToDangerousButt = new Button(XBOX, 3);

  // A simple auto routine that drives forward a specified distance, and then stops.
  private static Command m_turnAuto =
      new SequentialCommandGroup(
        new TurnToNAngle(90, m_driveTrain),
        new WaitCommand(2),
        new TurnToNAngle(0, m_driveTrain)
      );

  private static Command m_driveStraightAuto = new DriveForwardGivenTime(0.3, 1, m_driveTrain);
  private static Command m_bread = new IntakeBallShootBothP1(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_PB = new Position2Auton(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_jellyStrawberryAuton = new JellyStrawberryAuton(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_jelly = new Position3Auton(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_stale = new Position4AutonStale(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_crunchy = new Position5AutonPB(m_driveTrain, m_intake, m_shooter, m_indexer);


  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private static RobotContainer m_robotContainer = new RobotContainer();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
    // Smartdashboard Subsystems
    // SmartDashboard.putData(m_driveTrain);
    // SmartDashboard.putData(m_intake);
    // SmartDashboard.putData(m_shooter);
    // SmartDashboard.putData(m_indexer);

    //CameraServer.startAutomaticCapture();

    // Add commands to the autonomous command chooser
    m_chooser.addOption("Index and Shoot Both", m_bread);
    m_chooser.addOption("position 2 auton", m_PB);
    m_chooser.addOption("Jelly Strawberry auton", m_jellyStrawberryAuton);
    m_chooser.addOption("Turn Auto", m_turnAuto);
    m_chooser.addOption("Drive Straight Auto", m_driveStraightAuto);
    m_chooser.addOption("Position 3 (jelly)", m_jelly);
    m_chooser.addOption("Position 4 (stale bread)", m_stale);
    m_chooser.addOption("Position 5 (crunchy PB)", m_crunchy);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);

    // SmartDashboard Buttons
    Shuffleboard.getTab("DriveTrain").add("DriveForwardGivenTime: time", new DriveForwardGivenTime(0.3, 0.5, m_driveTrain));
    Shuffleboard.getTab("DriveTrain").add("DriveForwardGivenDistance", new DriveForwardGivenDistance(0.3, 40, m_driveTrain));

    Shuffleboard.getTab("DriveTrain").add("TurnNAngle", new TurnNAngle(90, m_driveTrain));

    Shuffleboard.getTab("DriveTrain").add("Turn to N Angle", new TurnToNAngle(90, m_driveTrain));
    Shuffleboard.getTab("Robot").add("Index and Shoot",  new SequentialCommandGroup(
          new InstantCommand(() -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT), m_shooter),
          new WaitUntilCommand(() -> m_shooter.correctSpeed()),
          new IndexerCmdForGivenTime(m_indexer, 0.5, 2)
    ));
    Shuffleboard.getTab("SmartDashboard").add("Follow Target", new FollowTarget(m_driveTrain));
    Shuffleboard.getTab("SmartDashboard").add("Buttery Follow Target", new ButterySmoothFollowTarget(m_driveTrain));
    

    // Changing the "a" value on shuffleboard to alter joystick drive sensitivity
    // Shuffleboard.getTab("Drive")
    // .add("a value", 1) 
    // .withWidget(BuiltInWidgets.kNumberSlider)
    // .withProperties(Map.of("min", 0, "max", 1))
    // .getEntry();

    SmartDashboard.putNumber("a value", XboxConstants.JOYSTICK_SENSITIVITY);


    // Configure the button bindings
    configureButtonBindings();
    
    System.out.println("Hello, I am in RobotContainer");

    m_driveTrain.setDefaultCommand(new ArcadeDriveCmd(m_driveTrain, //
    () -> -XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_SPEED_AXIS),
    // TODO WHY IS 0.25 HERE
    () -> XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_TURN_AXIS) * 0.25)//
);

    m_intake.setDefaultCommand(new IntakeCmd(m_intake, 0));
    m_shooter.setDefaultCommand(new SetShootPowerCmd(m_shooter, 0, 0));
    m_indexer.setDefaultCommand(new IndexerCmd(m_indexer, 0));
    m_climber.setDefaultCommand(new StopClimber(m_climber));

    // Configure default commands


    // Configure autonomous sendable chooser

    // m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());


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

    

      System.out.println (JOYSTICK);
      System.out.println ("************************************");
      new JoystickButton(JOYSTICK, JoystickConstants.SHIFT_HIGH_SPEED)
      .whenPressed(shiftToHot);
      new JoystickButton(JOYSTICK, JoystickConstants.SHIFT_HIGH_GEAR)
      .whenPressed(shiftToDangerous);

      new JoystickButton(JOYSTICK, JoystickConstants.INTAKE).whileHeld(intakeCmd);

      new JoystickButton(JOYSTICK, JoystickConstants.CLIMBER_UP).whenPressed(extendClimberCmd);
      
      // Indexer runs for 2 seconds when the shooter gets to the correct speed
      // Shooter stays at the correct speed 
      new JoystickButton(JOYSTICK, JoystickConstants.SHOOTER_BTN).whenPressed(
        new SequentialCommandGroup(
          new InstantCommand(() -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT), m_shooter),
          new WaitUntilCommand(() -> m_shooter.correctSpeed()),
          new IndexerCmdForGivenTime(m_indexer, 0.5, 2)
        )
      );

      new JoystickButton(JOYSTICK, JoystickConstants.INDEXER_FWD).whileHeld(indexerFwdCmd);
      new JoystickButton(JOYSTICK, JoystickConstants.INDEXER_BACK).whileHeld(indexerBackCmd);
      
      new JoystickButton(JOYSTICK, JoystickConstants.TURN_TO_N).whenPressed(m_turnToNAngle);

      new JoystickButton(XBOX, XboxConstants.TURN_RIGHT).whenPressed(m_turnRight);
      new JoystickButton(XBOX, XboxConstants.TURN_LEFT).whenPressed(m_turnLeft);
      new JoystickButton(XBOX, XboxConstants.TURN_180).whenPressed(m_turn180);

    }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    System.out.println("Autonomous command!" + m_chooser.getSelected());
    return m_chooser.getSelected();
    // return new SequentialCommandGroup(
    //   //new DriveForwardGivenTime(2, 0.5, m_driveTrain),
    //   new TurnToNAngle(90, m_driveTrain),
    //   new WaitCommand(2),
    //   //new DriveForwardGivenTime(2, 0.5, m_driveTrain),
    //   new TurnToNAngle(180, m_driveTrain),
    //   new WaitCommand(2),
    //   new TurnToNAngle(-180, m_driveTrain),
    //   new WaitCommand(2),
    //   //new DriveForwardGivenTime(2, 0.5, m_driveTrain),
    //   new TurnToNAngle(270, m_driveTrain),
    //   new WaitCommand(2),
    //   //new DriveForwardGivenTime(2, 0.5, m_driveTrain),
    //   new TurnToNAngle(360, m_driveTrain),
    //   new WaitCommand(2),
    //   new TurnToNAngle(360, m_driveTrain)
    // );
  }

}

