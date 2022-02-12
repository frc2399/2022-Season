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
//import edu.wpi.first.wpilibj.Joystick;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

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
    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        leftFrontMotorController = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, MotorType.kBrushed);
        rightFrontMotorController = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, MotorType.kBrushed);
        leftMiddleMotorController = new CANSparkMax(DriveConstants.LEFT_MIDDLE_MOTOR_ID, MotorType.kBrushed);
        rightMiddleMotorController = new CANSparkMax(DriveConstants.RIGHT_MIDDLE_MOTOR_ID, MotorType.kBrushed);
        leftBackMotorController = new CANSparkMax(DriveConstants.BACK_LEFT_MOTOR_ID, MotorType.kBrushed);
        rightBackMotorController = new CANSparkMax(DriveConstants.BACK_RIGHT_MOTOR_ID, MotorType.kBrushed);

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

