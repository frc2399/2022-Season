//Clare was here!!!!
//Herb was here !!!!
//Alison was here :)
//Maisie was here :))))))))))))))
//Rachel was here!!!! :DDDDDDD
//Alice was hwew :33333333333333
//Ethan was here (O-O)

package frc.robot.subsystems;

// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
//import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SimEncoder;
import frc.robot.util.SimGyro;

//import edu.wpi.first.wpilibj.Joystick;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.util.NavX;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;





/**
 *
 */
public class DriveTrain extends SubsystemBase {

    private static CANSparkMax leftFrontMotorController;
    public static CANSparkMax rightFrontMotorController;
    private static CANSparkMax leftMiddleMotorController;
    private static CANSparkMax rightMiddleMotorController;
    private static CANSparkMax leftBackMotorController;
    private static CANSparkMax rightBackMotorController;

    public RelativeEncoder leftEncoder, rightEncoder;

    public static AHRS ahrs;
    public static PIDController turnController;
    //public static DifferentialDrive myRobot;
    //double rotateToAngleRate;


    public final double kP = 0.06;
    public final double kPSim = 0.5;
    static final double kI = 0;
    static final double kD = 0;
    static final double kF = 0;

    //private static double currentAngle = 0;
    //private static double targetAngle = 0;
    public static double outputSpeed;

    
    //static double kToleranceDegrees = 2.0f;
    /**
    *
    */

    private DifferentialDriveOdometry odometry;

    public SimEncoder leftEncoderSim;
    public SimEncoder rightEncoderSim;
    public SimGyro gyroSim;
    private DifferentialDrivetrainSim driveSim;
    private Field2d field = new Field2d();


    final ShuffleboardTab tab = Shuffleboard.getTab("Motor Diag");
    public static final NetworkTableEntry angleErrorTolerance = Shuffleboard.getTab("Params").addPersistent("Angle Err Tol", 2).getEntry();
    public static final NetworkTableEntry distanceErrorTolerance = Shuffleboard.getTab("Params").addPersistent("Distance Err Tol", 5).getEntry();
    public static final NetworkTableEntry robotAngle = Shuffleboard.getTab("Driver").add("Angle of Robot", 0).getEntry();
    public static final NetworkTableEntry angleErrorPValue = Shuffleboard.getTab("Params").add("angle err p", 0.01).getEntry();
    public static final NetworkTableEntry encoderTickLeft = Shuffleboard.getTab("Testing").add("tick left", 0).getEntry();
    public static final NetworkTableEntry encoderTickRight = Shuffleboard.getTab("Testing").add("tick right", 0).getEntry();




    public DriveTrain() {

        leftFrontMotorController = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, MotorType.kBrushless);
        rightFrontMotorController = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, MotorType.kBrushless);
        leftMiddleMotorController = new CANSparkMax(DriveConstants.LEFT_MIDDLE_MOTOR_ID, MotorType.kBrushless);
        rightMiddleMotorController = new CANSparkMax(DriveConstants.RIGHT_MIDDLE_MOTOR_ID, MotorType.kBrushless);
        leftBackMotorController = new CANSparkMax(DriveConstants.LEFT_BACK_MOTOR_ID, MotorType.kBrushless);
        rightBackMotorController = new CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_ID, MotorType.kBrushless);

        // Set motors to coast mode
        teleopInit();

        // Make wheels go in same direction
        leftFrontMotorController.setInverted(true);
        rightFrontMotorController.setInverted(false);

        //sets motor controllers following leaders
        leftMiddleMotorController.follow(leftFrontMotorController);
        rightMiddleMotorController.follow(rightFrontMotorController);
        leftBackMotorController.follow(leftFrontMotorController);
        rightBackMotorController.follow(rightFrontMotorController);

        //myRobot = new DifferentialDrive(leftFrontMotorController, rightFrontMotorControllefr);

        turnController = new PIDController(kP, kI, kD);
        turnController.enableContinuousInput(-180.0f, 180.0f);

       //initialize motor encoder
        leftEncoder = leftFrontMotorController.getEncoder();
        rightEncoder = rightFrontMotorController.getEncoder();

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        leftEncoder.setPositionConversionFactor(Constants.DriveConstants.REVOLUTION_TO_INCH_CONVERSION);
        rightEncoder.setPositionConversionFactor(Constants.DriveConstants.REVOLUTION_TO_INCH_CONVERSION);

        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();

        // this code is instantiating the simulated sensors and actuators when the robot is in simulation
        if (RobotBase.isSimulation()) {

            leftEncoderSim = new SimEncoder("Left Drive");
            rightEncoderSim = new SimEncoder("Right Drive");
            gyroSim = new SimGyro("NavX");
            odometry = new DifferentialDriveOdometry(gyroSim.getAngle(), new Pose2d(9, 6.5, new Rotation2d(3.14/2)));
            // Create the simulation model of our drivetrain.
            driveSim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(3),       // 3 NEO motors on each side of the drivetrain.
                8,                       // 8:1 gearing reduction. for now
                6,                       // MOI of 6 kg m^2 (from CAD model). for now
                Units.lbsToKilograms(140), // The mass of the robot is 140 lbs (with battery) which is 63 kg
                Units.inchesToMeters(2.1), // The robot uses 2.1" radius wheels.
                Units.inchesToMeters(27.811), // The track width is 27.811 inches.

                // The standard deviations for measurement noise:
                // x and y:          0 m
                // heading:          0 rad
                // l and r velocity: 0  m/s
                // l and r position: 0 m
                VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
            );

            field = new Field2d();

            // Ethan is suspicious and thinks we need to re-enable this but it doesn't matter
            // SmartDashboard.putData("Field", field);

            field.setRobotPose(new Pose2d(9, 6.5, new Rotation2d(3.14/2)));
        }

    

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // currentAngle = ahrs.getAngle();
        // targetAngle = Robot.targetAngle;

        // double error = targetAngle - currentAngle;

        // outputSpeed = kP * error;

        // robotAngle.setDouble(ahrs.getAngle());
        // SmartDashboard.putNumber("Angle", ahrs.getAngle());

        SmartDashboard.putNumber("hub angle", PhotonLimelight.angleToHub());
        double error = PhotonLimelight.angleToHub();
        SmartDashboard.putNumber("hub error!!!!!!", error);

// 
        // System.out.println("drive train periodic");

        //SmartDashboard.putNumber("target angle", RobotContainer.m_turnToNAngle.targetAngle);

        // outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);
        // setMotors(-outputSpeed, outputSpeed);


        // This will get the simulated sensor readings that we set
        // in the previous article while in simulation, but will use
        // real values on the robot itself.
        // finds the position and angle of the robot given gyro and encoders

        encoderTickLeft.setDouble(leftEncoder.getPosition());
        encoderTickRight.setDouble(rightEncoder.getPosition());

    }



    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.

        odometry.update(
            // changes the sign of input for turning because the simulator would invert it otherwise (we want CW is positive, CCW is negative)
            new Rotation2d(-gyroSim.getAngle().getRadians()),
            leftEncoderSim.getDistance(),
            rightEncoderSim.getDistance()
        );
        field.setRobotPose(odometry.getPoseMeters());

        driveSim.setInputs(
            leftFrontMotorController.get() * RobotController.getInputVoltage(),
            rightFrontMotorController.get() * RobotController.getInputVoltage()
        );
    

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        driveSim.update(0.02);

        // Update all of our sensors.
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setSpeed(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setSpeed(driveSim.getRightVelocityMetersPerSecond());
        // inverts so the code reads it as positive after the simulator (we want CW is positive, CCW is negative)
        gyroSim.setAngle(new Rotation2d(-driveSim.getHeading().getRadians()));


    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {

        leftFrontMotorController.set(leftSpeed);
        rightFrontMotorController.set(rightSpeed);

        // SmartDashboard.putNumber("outputSpeed", leftSpeed);
    }

    public double getLeftEncoderPosition()
    {
        if (RobotBase.isSimulation())
        {
            return Units.metersToInches(leftEncoderSim.getDistance());
        }
        else
        {
            //gets position in inches
            return leftEncoder.getPosition();
        }
    }

    public double getRightEncoderPosition()
    {
        if (RobotBase.isSimulation())
        {
            return Units.metersToInches(rightEncoderSim.getDistance());
        }
        else
        {
            //gets position in inches
            return rightEncoder.getPosition();
        }
    }

    public double getAngle()
    { 
        return ahrs.getAngle();
    }

    public static void teleopInit()
    {
        leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public static void autonomousInit()
    {
        leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }


}

