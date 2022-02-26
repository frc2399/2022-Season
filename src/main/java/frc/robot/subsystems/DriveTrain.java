// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.


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

//import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SimEncoder;
import frc.robot.util.SimGyro;

//import edu.wpi.first.wpilibj.Joystick;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.util.NavX;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;





/**
 *
 */
public class DriveTrain extends SubsystemBase {


    private static CANSparkMax leftFrontMotorController;
    private static CANSparkMax rightFrontMotorController;
    private CANSparkMax leftMiddleMotorController;
    private CANSparkMax rightMiddleMotorController;
    private CANSparkMax leftBackMotorController;
    private CANSparkMax rightBackMotorController;

    public static AHRS ahrs;
    public static PIDController turnController;
    //public static DifferentialDrive myRobot;
    //double rotateToAngleRate;


    public final double kP = 0.06;
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

    private SimEncoder leftEncoderSim;
    private SimEncoder rightEncoderSim;
    private SimGyro gyroSim;
    private DifferentialDrivetrainSim driveSim;
    private Field2d field;

    public DriveTrain() {

        leftFrontMotorController = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, MotorType.kBrushed);
        rightFrontMotorController = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, MotorType.kBrushed);
        leftMiddleMotorController = new CANSparkMax(DriveConstants.LEFT_MIDDLE_MOTOR_ID, MotorType.kBrushed);
        rightMiddleMotorController = new CANSparkMax(DriveConstants.RIGHT_MIDDLE_MOTOR_ID, MotorType.kBrushed);
        leftBackMotorController = new CANSparkMax(DriveConstants.BACK_LEFT_MOTOR_ID, MotorType.kBrushed);
        rightBackMotorController = new CANSparkMax(DriveConstants.BACK_RIGHT_MOTOR_ID, MotorType.kBrushed);

        // Set motors to brake mode 
        leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Make wheels go in same direction
        leftFrontMotorController.setInverted(false);
        rightFrontMotorController.setInverted(true);

        //sets motor controllers following leaders
        leftMiddleMotorController.follow(leftFrontMotorController);
        rightMiddleMotorController.follow(rightFrontMotorController);
        leftBackMotorController.follow(leftFrontMotorController);
        rightBackMotorController.follow(rightFrontMotorController);

        //myRobot = new DifferentialDrive(leftFrontMotorController, rightFrontMotorController);

        turnController = new PIDController(kP, kI, kD);
        turnController.enableContinuousInput(-180.0f, 180.0f);

        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();

        // this code is instantiating the simulated sensors and actuators when the robot is in simulation
        if (RobotBase.isSimulation()) {
            leftEncoderSim = new SimEncoder("Left Drive");
            rightEncoderSim = new SimEncoder("Right Drive");
            gyroSim = new SimGyro("NavX");
            odometry = new DifferentialDriveOdometry(gyroSim.getAngle());
            // Create the simulation model of our drivetrain.
            driveSim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(3),       // 3 NEO motors on each side of the drivetrain.
                8,                       // 8:1 gearing reduction. for now
                5,                       // MOI of 5 kg m^2 (from CAD model). for now
                Units.lbsToKilograms(140), // The mass of the robot is 140 lbs (with battery) which is 63 kg
                Units.inchesToMeters(4.2), // The robot uses 4.2" radius wheels.
                Units.inchesToMeters(27.811), // The track width is 27.811 inches.

                // The standard deviations for measurement noise:
                // x and y:          0 m
                // heading:          0 rad
                // l and r velocity: 0  m/s
                // l and r position: 0 m
                VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
            );

            field = new Field2d();
        }

    

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // currentAngle = ahrs.getAngle();
        // targetAngle = Robot.targetAngle;

        // double error = targetAngle - currentAngle;

        // outputSpeed = kP * error;
        SmartDashboard.putNumber("Angle", ahrs.getAngle());
        //SmartDashboard.putNumber("target angle", RobotContainer.m_turnToNAngle.targetAngle);

        // outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);
        // setMotors(-outputSpeed, outputSpeed);

        double fahrenheit = ahrs.getTempC() * (9/5) + 32;
        SmartDashboard.putNumber("temperature", fahrenheit);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        driveSim.setInputs(leftFrontMotorController.get() * RobotController.getInputVoltage(),
        rightFrontMotorController.get() * RobotController.getInputVoltage());
    

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        driveSim.update(0.02);

        // Update all of our sensors.
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setSpeed(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setSpeed(driveSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(driveSim.getHeading());

        System.out.println(leftFrontMotorController.get());
        System.out.println(rightFrontMotorController.get());
        System.out.println(leftMiddleMotorController.get());
        System.out.println(driveSim.getHeading());
        System.out.println(leftEncoderSim.getSpeed());
        System.out.println(rightEncoderSim.getSpeed());

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {
        leftFrontMotorController.set(leftSpeed);
        rightFrontMotorController.set(rightSpeed);
        SmartDashboard.putNumber("outputSpeed", leftSpeed);
    }

    public double getAngle()
    {
        return ahrs.getAngle();
    }

}

