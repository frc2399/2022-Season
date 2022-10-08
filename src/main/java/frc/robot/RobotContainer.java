package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.TurnToHub;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.indexer.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.intakearm.ExtendIntakeArm;
import frc.robot.commands.intakearm.RetractIntakeArm;
import frc.robot.commands.robot.PointAndShoot;
import frc.robot.commands.robot.UpperShootFromFender;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;
import frc.robot.util.DPadButton;
import frc.robot.util.DriveTurnControls;

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
    public final static PhotonLimelight m_photonLimelight = new PhotonLimelight();

    // Joysticks
    public static Joystick JOYSTICK = new Joystick(JoystickConstants.JOYSTICK_PORT);
    public static Joystick XBOX = new Joystick(XboxConstants.XBOX_PORT);

    private DriveTurnControls driveTurnControls = new DriveTurnControls(XBOX);

    // Defining commands
    private static InstantCommand shiftHighTorque = new InstantCommand(() -> m_shifter.setShifterHighTorque(),
        m_shifter);
    private static InstantCommand shiftHighSpeed = new InstantCommand(() -> m_shifter.setShifterHighSpeed(), m_shifter);

    //private static InstantCommand extendIntakeArm = new InstantCommand(() -> m_intake.extendArm(), m_intake);
    //private static InstantCommand retractIntakeArm = new InstantCommand(() -> m_intake.retractArm(), m_intake);

    // Robot
    private static InstantCommand killCommand = new InstantCommand(() -> CommandScheduler.getInstance().cancelAll());

    private static InstantCommand toggleSpeedCommand = new InstantCommand(() -> m_driveTrain.isSlow = !m_driveTrain.isSlow);


    // Shooter 
    private static InstantCommand startShooter = new InstantCommand(() -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT), m_shooter);
    private static SetShootPowerCmd stopShooter =  new SetShootPowerCmd(m_shooter, 0, 0);

    //Turn To Hub
    private static Command turnToHub = new TurnToHub(m_driveTrain);

    // private  static Command shoot = new SequentialCommandGroup(
    //     new IndexerCmdForGivenTime(m_indexer, -0.5, 0.1),
    //     new InstantCommand(
    //         () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT,ShooterConstants.BOTTOM_SETPOINT), m_shooter),
    //     new WaitUntilCommand(() -> m_shooter.correctSpeed()),
    //     new IndexerCmdForGivenTime(m_indexer, 0.5, 2));

    private  static Command maxShoot = new SequentialCommandGroup(
        new IndexerCmdForGivenTime(m_indexer, -0.5, 0.1),
        new InstantCommand(
            () -> m_shooter.setSpeedWithPID(ShooterConstants.MAX_SETPOINT,ShooterConstants.MAX_SETPOINT), m_shooter),
        new WaitUntilCommand(() -> m_shooter.correctSpeed()),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 2));

    public static Command lowerShootFromFender = new LowerShootFromFender(m_indexer, m_shooter);

    public static Command upperShootFromFender = new UpperShootFromFender(m_indexer, m_shooter, m_driveTrain);

        
    private static Command spitOutBall = new ParallelCommandGroup(
        new ExtendIntakeArm(m_intake),
        new IndexerBackCmd(m_indexer),
        new IntakeCmd(m_intake, -IntakeConstants.INTAKESPEED));
    

    public static Command upperShootFromTarmac = new SequentialCommandGroup(
        new IndexerCmdForGivenTime(m_indexer, -0.5, 0.1),
        new InstantCommand(
            () -> m_shooter.setSpeedWithPID(ShooterConstants.TARMAC_UPPER_SHOOTER_TOP_SPEED,ShooterConstants.TARMAC_UPPER_SHOOTER_BOTTOM_SPEED), m_shooter),
        new WaitUntilCommand(() -> m_shooter.correctSpeed()),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 2));
    

    
    

    // Intake
    // private static IntakeCmd intakeCmd = new IntakeCmd(m_intake, IntakeConstants.INTAKESPEED);
    // private static IntakeFwdCmd intakeFwdCmd = new IntakeFwdCmd(m_intake);
    // private static IntakeBackCmd intakeBackCmd = new IntakeBackCmd(m_intake);

    private static Command defaultIntake = new IntakeCmd(m_intake, 0);

    // Indexer
    private static IndexerFwdCmd indexerFwdCmd = new IndexerFwdCmd(m_indexer);
    private static IndexerBackCmd indexerBackCmd = new IndexerBackCmd(m_indexer);

    // Ultimate command
    private static PointAndShoot pointAndShootCmd = new PointAndShoot(m_driveTrain, m_shooter, m_indexer);

    // Collect balls
    private static ParallelCommandGroup collectBall = new ParallelCommandGroup(
        new ExtendIntakeArm(m_intake),
        new IntakeCmd(m_intake, IntakeConstants.INTAKESPEED),
        new IndexerCmd(m_indexer, IndexerConstants.INDEXERSPEED)); 

    private static ParallelCommandGroup noCollectBall = new ParallelCommandGroup(
        new RetractIntakeArm(m_intake),
        new IntakeCmd(m_intake, 0),
        new IndexerCmd(m_indexer, 0));

    // Climber
    private static ExtendClimber extendClimberCmd = new ExtendClimber(m_climber, 1.0);
    private static RetractClimber retractClimberCmd = new RetractClimber(m_climber, 1.0);

    private static ExtendLeftClimber extendLeftClimberCmd = new ExtendLeftClimber(m_climber, 1.0);
    private static ExtendRightClimber extendRightClimberCmd = new ExtendRightClimber(m_climber, 1.0);

    // Drive Train
    public static TurnToNAngle m_turnToNAngle = new TurnToNAngle(0, m_driveTrain);
    public static TurnNAngle m_turnRight = new TurnNAngle(.5, 90, m_driveTrain);
    public static TurnNAngle m_turnLeft = new TurnNAngle(.5, -90, m_driveTrain);
    public static TurnNAngle m_turn180 = new TurnNAngle(.5, 180, m_driveTrain);
    private static Command m_driveStraightAuto = new DriveForwardGivenTime(0.3, 1, m_driveTrain);

    // Autonomous
    // A simple auto routine that drives forward a specified distance, and then
    // stops.
    private static Command m_turnAuto = new SequentialCommandGroup(
        new TurnToNAngle(90, m_driveTrain),
        new WaitCommand(2),
        new TurnToNAngle(0, m_driveTrain));

    private static Command m_bread = new Position1AutonBREAD(m_driveTrain, m_intake, m_shooter, m_indexer);
    private static Command m_PB = new Position2AutonPB(m_driveTrain, m_intake, m_shooter, m_indexer);
    private static Command m_jellyStrawberryAuton = new Position6AutonSTRAWBERRY(m_driveTrain, m_intake, m_shooter, m_indexer);
    private static Command m_jelly = new Position3AutonJELLY(m_driveTrain, m_intake, m_shooter, m_indexer);
    private static Command m_stale = new Position4AutonStale(m_driveTrain, m_intake, m_shooter, m_indexer);
    private static Command m_crunchy = new Position5AutonCRUNCHY(m_driveTrain, m_intake, m_shooter, m_indexer);
    private static Command m_air = new DoNothingAuton();
    private static Command m_banana = new DriveOutAuton(m_driveTrain);
    private static Command m_ploop = new PloopAndDriveOutAuton(m_driveTrain, m_shooter, m_indexer);
    private static Command m_pop = new PopAndDriveOutAuton(m_driveTrain, m_shooter, m_indexer);


    private static Command m_pointAndShoot = new PointAndShoot(m_driveTrain, m_shooter, m_indexer);

   // private static Command m_driveStraightGivenDistance = new DriveStraightGivenDistance(0.5, 10, m_driveTrain);

    public static NetworkTableEntry a_value = Shuffleboard.getTab("Params")
        .addPersistent("Stick Sensitivity", 0.0).getEntry();

    // A chooser for autonomous commands
    static SendableChooser < Command > m_chooser = new SendableChooser < > ();
    public final static ComplexWidget autonChooser = Shuffleboard.getTab("Driver")
    .add("Choose Auton", m_chooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(4, 4).withSize(9, 1);

    PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

    // ALWAYS put this last!!!!
    private static RobotContainer m_robotContainer = new RobotContainer();

    private RobotContainer() {

        
        // camera not in simulator to make it not crash
        if (RobotBase.isReal())
        {
           // CameraServer.startAutomaticCapture();
        }

        // Add commands to the autonomous command chooser
        m_chooser.addOption("Bread - position 1 (2 ball)", m_bread);
        m_chooser.addOption("Peanut Butter - position 2 (2 ball)", m_PB);
        m_chooser.addOption("Strawberry Jelly - position 3 (3 ball)", m_jellyStrawberryAuton);
        m_chooser.addOption("Turn Auto", m_turnAuto);
        m_chooser.addOption("Drive Straight Auto", m_driveStraightAuto);
        m_chooser.addOption("Jelly - position 3 (2 ball)", m_jelly);
        m_chooser.addOption("Stale Bread - position 1 (2 ball)", m_stale);
        m_chooser.addOption("Crunchy Peanut Butter - position 2 (3 ball)", m_crunchy);
        m_chooser.addOption("Do nothing (air)", m_air);
        m_chooser.addOption("Drive out tarmac (banana)", m_banana);
        m_chooser.addOption("Ploop and drive out", m_ploop);
        m_chooser.addOption("Pop and drive out", m_pop);

        // Put the chooser on the dashboard

        // Smart Dashboard
        // Smartdashboard Subsystems
        // SmartDashboard.putData(m_driveTrain);
        // SmartDashboard.putData(m_intake);
        // SmartDashboard.putData(m_shooter);
        // SmartDashboard.putData(m_indexer);

        // Shuffleboard.getTab("SmartDashboard").add("Follow Target", new
        // FollowTarget(m_driveTrain));
        // Shuffleboard.getTab("SmartDashboard").add("Buttery Follow Target", new
        // ButterySmoothFollowTarget(m_driveTrain));

        Shuffleboard.getTab("Shooter").add(
            "Start Shooter", startShooter.perpetually()
            );
        Shuffleboard.getTab("Shooter").add(
            "Stop Shooter", stopShooter
        );

        // playing around hehe
        Shuffleboard.getTab("Testing").add("DriveGivenTime",
            new DriveForwardGivenTime(0.3, 0.5, m_driveTrain));
        Shuffleboard.getTab("Testing").add("DriveGivenDistance",
            new DriveForwardGivenDistance(0.3, 40, m_driveTrain));

        Shuffleboard.getTab("Testing").add("Turn to N Angle", new TurnToNAngle(90, m_driveTrain));
        Shuffleboard.getTab("Testing").add("TurnNAngle", new TurnNAngle(.5, 90, m_driveTrain));

        Shuffleboard.getTab("Testing").add("DriveStraightGivenDistance", new DriveStraightGivenDistance(.5, 40, m_driveTrain));

        Shuffleboard.getTab("Testing").add("Turn to hub", new TurnToHub(m_driveTrain));


        SmartDashboard.putNumber("drive slew", XboxConstants.DRIVE_SLEW_RATE);
        SmartDashboard.putNumber("turn slew", XboxConstants.TURN_SLEW_RATE);

        // SmartDashboard.putNumber("a value", XboxConstants.JOYSTICK_SENSITIVITY);

        // Changing the "a" value on shuffleboard to alter joystick drive sensitivity
        // Shuffleboard.getTab("Drive")
        // .add("a value", 1)
        // .withWidget(BuiltInWidgets.kNumberSlider)
        // .withProperties(Map.of("min", 0, "max", 1))
        // .getEntry();

        // SmartDashboard.putNumber("Joystick Sensitivity",
        // XboxConstants.JOYSTICK_SENSITIVITY);

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // m_driveTrain.setDefaultCommand(
        // new ArcadeDriveCmd(m_driveTrain,
        // () -> -XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_SPEED_AXIS),
        // () -> XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_TURN_AXIS)));
        m_driveTrain.setDefaultCommand(
            new ArcadeDriveCmd(m_driveTrain,
                () -> -driveTurnControls.getDrive(),
                () -> driveTurnControls.getTurn()));

        m_intake.setDefaultCommand(defaultIntake.perpetually());
        m_intake.setDefaultCommand(new IntakeCmd(m_intake, 0));
        m_shooter.setDefaultCommand(new SetShootPowerCmd(m_shooter, 0, 0));
        m_indexer.setDefaultCommand(new IndexerDefaultCmd(m_indexer));
        m_climber.setDefaultCommand(new StopClimber(m_climber));

        m_shifter.setShifterHighTorque();

        // Shuffleboard.getTab("Robot").add("Index and Shoot", new
        // SequentialCommandGroup(
        // new InstantCommand(
        // () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT,
        // ShooterConstants.BOTTOM_SETPOINT),
        // m_shooter),
        // new WaitUntilCommand(() -> m_shooter.correctSpeed()),
        // new IndexerCmdForGivenTime(m_indexer, 0.5, 2)));

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
        // Robot
       // new JoystickButton(XBOX, XboxMappingToJoystick.Y_BUTTON).whenPressed(killCommand);
        new DPadButton(XBOX, DPadButton.Direction.DOWN).whenPressed(killCommand);
        new JoystickButton(XBOX, XboxMappingToJoystick.Y_BUTTON).whenPressed(toggleSpeedCommand);

        // Drive train
        new JoystickButton(XBOX, XboxConstants.SHIFT_HIGH_TORQUE).whenPressed(shiftHighTorque);
        new JoystickButton(XBOX, XboxConstants.SHIFT_HIGH_SPEED).whenPressed(shiftHighSpeed);

        Trigger upperShootFromTarmacTrigger = new Trigger(() -> XBOX.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.05);
             upperShootFromTarmacTrigger.whileActiveContinuous(upperShootFromFender);

        
        Trigger intakeTrigger = new Trigger(() -> XBOX.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.1);
             intakeTrigger.whileActiveContinuous(collectBall);
             intakeTrigger.whenInactive(noCollectBall);

        

 

        // new JoystickButton(XBOX, XboxMappingToJoystick.A_BUTTON).whenPressed(pointAndShootCmd);

      
        new JoystickButton(XBOX, XboxConstants.POINT_AND_SHOOT).whenPressed(m_pointAndShoot);

     

        // Climber
        new JoystickButton(JOYSTICK, JoystickConstants.CLIMBER_DOWN).whileHeld(retractClimberCmd);
        new JoystickButton(JOYSTICK, JoystickConstants.CLIMBER_UP).whileHeld(extendClimberCmd);

        new JoystickButton(JOYSTICK, JoystickConstants.LEFT_CLIMBER_UP).whileHeld(extendRightClimberCmd);
        new JoystickButton(JOYSTICK, JoystickConstants.RIGHT_CLIMBER_UP).whileHeld(extendLeftClimberCmd);
        // Shooter
        // new JoystickButton(JOYSTICK, JoystickConstants.SHOOTER_BTN).whenPressed(shoot);
        new JoystickButton(XBOX, XboxMappingToJoystick.B_BUTTON).whenPressed(lowerShootFromFender);

        new JoystickButton(JOYSTICK, JoystickConstants.MAX_SHOOT).whenPressed(maxShoot);

        new JoystickButton(XBOX, XboxMappingToJoystick.X_BUTTON).whileHeld(spitOutBall).whenReleased(noCollectBall);
        
        new JoystickButton(JOYSTICK, JoystickConstants.TURN_TO_HUB).whenPressed(turnToHub);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        System.out.println("Autonomous command!" + m_chooser.getSelected());
        return m_chooser.getSelected();
    }

}
