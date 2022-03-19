package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
        public final static Limelight m_limelight = new Limelight();
        public final static PhotonLimelight m_photonLimelight = new PhotonLimelight();

        // Joysticks
        public static Joystick JOYSTICK = new Joystick(JoystickConstants.JOYSTICK_PORT);
        public static Joystick XBOX = new Joystick(XboxConstants.XBOX_PORT);

        private DriveTurnControls driveTurnControls = new DriveTurnControls(XBOX);

        // Defining commands
        private static InstantCommand shiftHighGear = new InstantCommand(() -> m_shifter.setShifterDangerous(),
                        m_shifter);
        private static InstantCommand shiftHighSpeed = new InstantCommand(() -> m_shifter.setShifterHot(), m_shifter);

        private static InstantCommand extendIntakeArm = new InstantCommand(() -> m_intake.extendArm(), m_intake);
        private static InstantCommand retractIntakeArm = new InstantCommand(() -> m_intake.retractArm(), m_intake);

        // Intake
        private static IntakeCmd intakeCmd = new IntakeCmd(m_intake, IntakeConstants.INTAKESPEED);

        // Indexer
        private static IndexerCmd indexerFwdCmd = new IndexerCmd(m_indexer, IndexerConstants.INDEXERSPEED);
        private static IndexerCmd indexerBackCmd = new IndexerCmd(m_indexer, -IndexerConstants.INDEXERSPEED);

        // Climber
        private static ExtendClimber extendClimberCmd = new ExtendClimber(m_climber, 0.5);
        private static RetractClimber retractClimberCmd = new RetractClimber(m_climber, 0.5);

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

        public static NetworkTableEntry a_value = Shuffleboard.getTab("Params")
                        .addPersistent("Stick Sensitivity", 0.0).getEntry();


        // A chooser for autonomous commands
        static SendableChooser<Command> m_chooser = new SendableChooser<>();
        public final static ComplexWidget autonChooser = Shuffleboard.getTab("Driver").add("Choose Auton", m_chooser);

        PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

        // ALWAYS put this last!!!!
        private static RobotContainer m_robotContainer = new RobotContainer();

        private RobotContainer() {

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

                // //Shuffleboard.getTab("Robot").add("Index and Shoot", new
                // SequentialCommandGroup(
                // new InstantCommand(
                // () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT,
                // ShooterConstants.BOTTOM_SETPOINT),
                // m_shooter),
                // new WaitUntilCommand(() -> m_shooter.correctSpeed()),
                // new IndexerCmdForGivenTime(m_indexer, 0.5, 2)));

                Shuffleboard.getTab("Testing").add("DriveGivenTime",
                                new DriveForwardGivenTime(0.3, 0.5, m_driveTrain));
                Shuffleboard.getTab("Testing").add("DriveGivenDistance",
                                new DriveForwardGivenDistance(0.3, 40, m_driveTrain));

                Shuffleboard.getTab("Testing").add("Turn to N Angle", new TurnToNAngle(90, m_driveTrain));
                Shuffleboard.getTab("Testing").add("TurnNAngle", new TurnNAngle(90, m_driveTrain));

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

                m_intake.setDefaultCommand(new IntakeCmd(m_intake, 0));
                m_shooter.setDefaultCommand(new SetShootPowerCmd(m_shooter, 0, 0));
                m_indexer.setDefaultCommand(new IndexerCmd(m_indexer, 0));
                m_climber.setDefaultCommand(new StopClimber(m_climber));

                m_shifter.setShifterDangerous();

                m_intake.retractArm();

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
                // Drive train
                new JoystickButton(XBOX, XboxConstants.SHIFT_HIGH_TORQUE).whenPressed(shiftHighGear);
                new JoystickButton(XBOX, XboxConstants.SHIFT_HIGH_SPEED).whenPressed(shiftHighSpeed);
                // new JoystickButton(JOYSTICK,
                // JoystickConstants.TURN_TO_N).whenPressed(m_turnToNAngle);
                new JoystickButton(XBOX, XboxConstants.TURN_RIGHT).whenPressed(m_turnRight);
                new JoystickButton(XBOX, XboxConstants.TURN_LEFT).whenPressed(m_turnLeft);
                new JoystickButton(XBOX, XboxConstants.TURN_180).whenPressed(m_turn180);

                // Intake
                new JoystickButton(XBOX, XboxConstants.INTAKE).whileHeld(intakeCmd);
                new JoystickButton(JOYSTICK, JoystickConstants.INTAKE_ARM_EXTEND).whenPressed(extendIntakeArm);
                new JoystickButton(JOYSTICK, JoystickConstants.INTAKE_ARM_RETRACT).whenPressed(retractIntakeArm);

                // Indexer
                new JoystickButton(JOYSTICK, JoystickConstants.INDEXER_FWD).whileHeld(indexerFwdCmd);
                new JoystickButton(JOYSTICK, JoystickConstants.INDEXER_BACK).whileHeld(indexerBackCmd);
                // TODO: Need to create these commands
                // new JoystickButton(XBOX, XboxConstants.INDEXER_AND_SHOOT).whileHeld();
                // new JoystickButton(XBOX, XboxConstants.LIMELIGHT_SHOOT).whileHeld();
                // new JoystickButton(XBOX, XboxConstants.TURN_LEFT_90_CCW).whileHeld();
                // new JoystickButton(XBOX, XboxConstants.TURN_LEFT_90_CW).whileHeld();
                // new JoystickButton(XBOX, XboxConstants.TURN_180).whileHeld();

                // Climber
                new JoystickButton(JOYSTICK, JoystickConstants.CLIMBER_UP).whenPressed(retractClimberCmd);
                new JoystickButton(JOYSTICK, JoystickConstants.CLIMBER_DOWN).whenPressed(extendClimberCmd);

                // Shooter
                new JoystickButton(JOYSTICK, JoystickConstants.SHOOTER_BTN).whenPressed(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> m_shooter.setSpeedWithPID(
                                                                                ShooterConstants.TOP_SETPOINT,
                                                                                ShooterConstants.BOTTOM_SETPOINT),
                                                                m_shooter),
                                                new WaitUntilCommand(() -> m_shooter.correctSpeed()),
                                                new IndexerCmdForGivenTime(m_indexer, 0.5, 2)));

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
