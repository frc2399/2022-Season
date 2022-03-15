package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
//import edu.wpi.first.wpilibj.XboxController.Button;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.ButterySmoothFollowTarget;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.indexer.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */

public class RobotContainer {

  // The robot's subsystems
  public final static DriveTrain m_driveTrain = new DriveTrain();
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

  // Shift
  // private static Shift shiftToDangerous = new
  // Shift(!DriveConstants.SHIFTER_SOLENOID_HOT,
  // DriveConstants.SHIFTER_SOLENOID_DANGEROUS);
  // private static Shift shiftToHot = new
  // Shift(DriveConstants.SHIFTER_SOLENOID_HOT,!DriveConstants.SHIFTER_SOLENOID_DANGEROUS);

  private static InstantCommand shiftHighGear = new InstantCommand(() -> m_shifter.setShifterDangerous(), m_shifter);
  private static InstantCommand shiftHighSpeed = new InstantCommand(() -> m_shifter.setShifterHot(), m_shifter);

  private static InstantCommand extendIntakeArm = new InstantCommand(() -> m_intake.extendArm(), m_intake);
  private static InstantCommand retractIntakeArm = new InstantCommand(() -> m_intake.retractArm(), m_intake);

  // private static InstantCommand new InstantCommand(() ->
  // m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT,
  // ShooterConstants.BOTTOM_SETPOINT), m_shooter);

  // Intake
  private static IntakeCmd intakeCmd = new IntakeCmd(m_intake, IntakeConstants.INTAKESPEED);

  // Indexer
  private static IndexerCmd indexerFwdCmd = new IndexerCmd(m_indexer, IndexerConstants.INDEXERSPEED);
  private static IndexerCmd indexerBackCmd = new IndexerCmd(m_indexer, -IndexerConstants.INDEXERSPEED);

  // Climber
  private static ExtendClimber extendClimberCmd = new ExtendClimber(m_climber, 0.5);

  // Drive Train
  public static TurnToNAngle m_turnToNAngle = new TurnToNAngle(0, m_driveTrain);
  public static TurnNAngle m_turnRight = new TurnNAngle(90, m_driveTrain);
  public static TurnNAngle m_turnLeft = new TurnNAngle(-90, m_driveTrain);
  public static TurnNAngle m_turn180 = new TurnNAngle(180, m_driveTrain);
  private static Command m_driveStraightAuto = new DriveForwardGivenTime(0.3, 1, m_driveTrain);

  // Autonomous
  // A simple auto routine that drives forward a specified distance, and then
  // stops.
  private static Command m_turnAuto = new SequentialCommandGroup(
      new TurnToNAngle(90, m_driveTrain),
      new WaitCommand(2),
      new TurnToNAngle(0, m_driveTrain));

  private static Command m_bread = new IntakeBallShootBothP1(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_PB = new Position2Auton(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_jellyStrawberryAuton = new JellyStrawberryAuton(m_driveTrain, m_intake, m_shooter);
  private static Command m_jelly = new Position3Auton(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_stale = new Position4AutonStale(m_driveTrain, m_intake, m_shooter, m_indexer);
  private static Command m_crunchy = new Position5AutonPB(m_driveTrain, m_intake, m_shooter, m_indexer);

  public static NetworkTableEntry a_value = Shuffleboard.getTab("Params").addPersistent("a value", 1.0).getEntry();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  // ALWAYS put this last!!!!
  private static RobotContainer m_robotContainer = new RobotContainer();

  private RobotContainer() {
    // Smartdashboard Subsystems
    // SmartDashboard.putData(m_driveTrain);
    // SmartDashboard.putData(m_intake);
    // SmartDashboard.putData(m_shooter);
    // SmartDashboard.putData(m_indexer);

    // CameraServer.startAutomaticCapture();

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
    // Drive Train
    Shuffleboard.getTab("DriveTrain").add("DriveForwardGivenTime: time",
        new DriveForwardGivenTime(0.3, 0.5, m_driveTrain));
    Shuffleboard.getTab("DriveTrain").add("DriveForwardGivenDistance",
        new DriveForwardGivenDistance(0.3, 40, m_driveTrain));
    Shuffleboard.getTab("DriveTrain").add("TurnNAngle", new TurnNAngle(90, m_driveTrain));
    Shuffleboard.getTab("DriveTrain").add("Turn to N Angle", new TurnToNAngle(90, m_driveTrain));

    // Robot
    Shuffleboard.getTab("Robot").add("Index and Shoot", new SequentialCommandGroup(
        new InstantCommand(
            () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT),
            m_shooter),
        new WaitUntilCommand(() -> m_shooter.correctSpeed()),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 2)));

    // Smart Dashboard
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

    // Configure default commands
    m_driveTrain.setDefaultCommand(
        new ArcadeDriveCmd(m_driveTrain,
            () -> -XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_SPEED_AXIS),
            () -> XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_TURN_AXIS)));

    //
    // m_driveTrain.setDefaultCommand(new ArcadeDriveCmd(m_driveTrain,
    // () -> -XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_SPEED_AXIS),
    // () -> XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_TURN_AXIS) * 0.25)
    // );
    m_intake.setDefaultCommand(new IntakeCmd(m_intake, 0));
    m_shooter.setDefaultCommand(new SetShootPowerCmd(m_shooter, 0, 0));
    m_indexer.setDefaultCommand(new IndexerCmd(m_indexer, 0));
    m_climber.setDefaultCommand(new StopClimber(m_climber));

    m_shifter.setShifterDangerous();

    m_intake.retractArm();

    // Configure default commands

    // Configure autonomous sendable chooser

    // m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    // Create some buttons
    new JoystickButton(JOYSTICK, JoystickConstants.SHIFT_HIGH_GEAR)
        .whenPressed(shiftHighGear);
    new JoystickButton(JOYSTICK, JoystickConstants.SHIFT_HIGH_SPEED)
        .whenPressed(shiftHighSpeed);

    new JoystickButton(JOYSTICK, JoystickConstants.INTAKE).whileHeld(intakeCmd);

    new JoystickButton(JOYSTICK, JoystickConstants.CLIMBER_UP).whenPressed(extendClimberCmd);

    // Indexer runs for 2 seconds when the shooter gets to the correct speed
    // Shooter stays at the correct speed
    new JoystickButton(JOYSTICK, JoystickConstants.SHOOTER_BTN).whenPressed(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT),
                m_shooter),
            new WaitUntilCommand(() -> m_shooter.correctSpeed()),
            new IndexerCmdForGivenTime(m_indexer, 0.5, 2)));

    new JoystickButton(JOYSTICK, JoystickConstants.INDEXER_FWD).whileHeld(indexerFwdCmd);
    new JoystickButton(JOYSTICK, JoystickConstants.INDEXER_BACK).whileHeld(indexerBackCmd);

    new JoystickButton(JOYSTICK, JoystickConstants.TURN_TO_N).whenPressed(m_turnToNAngle);

    new JoystickButton(JOYSTICK, JoystickConstants.INTAKE).whileHeld(intakeCmd);

    new JoystickButton(JOYSTICK, JoystickConstants.CLIMBER_UP).whenPressed(extendClimberCmd);

    // Indexer runs for 2 seconds when the shooter gets to the correct speed
    // Shooter stays at the correct speed
    new JoystickButton(JOYSTICK, JoystickConstants.SHOOTER_BTN).whenPressed(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT),
                m_shooter),
            new WaitUntilCommand(() -> m_shooter.correctSpeed()),
            new IndexerCmdForGivenTime(m_indexer, 0.5, 2)));

    new JoystickButton(JOYSTICK, JoystickConstants.INDEXER_FWD).whileHeld(indexerFwdCmd);
    new JoystickButton(JOYSTICK, JoystickConstants.INDEXER_BACK).whileHeld(indexerBackCmd);
    new JoystickButton(JOYSTICK, JoystickConstants.TURN_TO_N).whenPressed(m_turnToNAngle);

    new JoystickButton(XBOX, XboxConstants.TURN_RIGHT).whenPressed(m_turnRight);
    new JoystickButton(XBOX, XboxConstants.TURN_LEFT).whenPressed(m_turnLeft);
    new JoystickButton(XBOX, XboxConstants.TURN_180).whenPressed(m_turn180);
    new JoystickButton(JOYSTICK, JoystickConstants.INTAKE_ARM_EXTEND).whenPressed(extendIntakeArm);
    new JoystickButton(JOYSTICK, JoystickConstants.INTAKE_ARM_RETRACT).whenPressed(retractIntakeArm);

  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    System.out.println("Autonomous command!" + m_chooser.getSelected());
    return m_chooser.getSelected();
  }

}
