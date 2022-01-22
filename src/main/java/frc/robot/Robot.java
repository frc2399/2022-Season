/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Import Statements for Talon Controllers and 775 motors
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

// Import Statements for Spark Max Controllers and Neo 550 Motors
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static DriverStation DS = DriverStation.getInstance();
  
  // public static Joystick pilot_stick = new Joystick(0);
  // public static Joystick copilot_stick = new Joystick(1);

  public static Joystick shooter_stick_a = new Joystick(1);  //Shooter controller - Logitech Attack 3
  public static Joystick shooter_stick_b = new Joystick(2);  //Second Shooter controller - Logitech Attack 3

  // setting up the SparkMax values
  private static final int deviceA = 3; // device ID for shooter A
  private static final int deviceB = 4; // device ID for shooter B

  private CANSparkMax a_motor, b_motor;
  private SparkMaxPIDController a_pidController, b_pidController;
  private RelativeEncoder a_encoder, b_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  
  // intake motor
  public static TalonSRX intake = new TalonSRX(1);
  // public static TalonSRX shootbottom = new TalonSRX(2);
  // public static TalonSRX winch = new TalonSRX(3);
  

  /* // Unused Inputs 
   * public static TalonSRX intake_1 = new TalonSRX(21);
   * public static TalonSRX intake_2 = new TalonSRX(22);

   * public static Compressor air1 = new Compressor();
   * public static Solenoid intakepiston = new Solenoid(0);
   * public static Solenoid liftpiston = new Solenoid(1); */
  
  @Override
  public void robotInit() {

    System.out.println("BEGIN robotInit()");

    // intake code
    intake.configFactoryDefault();
    intake.setNeutralMode(NeutralMode.Coast);
    intake.configVoltageCompSaturation(11);
    intake.enableVoltageCompensation(true);
 
    /* // current limit drive falcons
     * // TODO: fix for Spark Max
     * // factory default
     * shoottop.configFactoryDefault();
     * shootbottom.configFactoryDefault();
     * 
     * SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
     * falconlimit.enable = true;
     * falconlimit.currentLimit =  10;
     * falconlimit.triggerThresholdCurrent = 10;
     * falconlimit.triggerThresholdTime = 0.1;
     * 
     * shoottop.configSupplyCurrentLimit(falconlimit);
     * shootbottom.configSupplyCurrentLimit(falconlimit);
     * 
     * shoottop.setNeutralMode(NeutralMode.Coast);
     * shootbottom.setNeutralMode(NeutralMode.Coast);
     * 
     * shoottop.configVoltageCompSaturation(11);
     * shootbottom.configVoltageCompSaturation(11);
     * 
     * shoottop.enableVoltageCompensation(true);
     * shootbottom.enableVoltageCompensation(true); */
    
    // Begin Spark Max Code
    // https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java/Velocity%20Closed%20Loop%20Control

    // initialize NEO 550 motors
    a_motor = new CANSparkMax(deviceA, MotorType.kBrushless);
    b_motor = new CANSparkMax(deviceB, MotorType.kBrushless);
    
    // reset to defaults
    a_motor.restoreFactoryDefaults();
    b_motor.restoreFactoryDefaults();

    // build SparkMaxPIDController object to use PID functionality
    a_pidController = a_motor.getPIDController();
    b_pidController = b_motor.getPIDController();

    // encoder object created to display position values
    a_encoder = a_motor.getEncoder();
    b_encoder = b_motor.getEncoder();

    // PID Coefficients
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = 0;
    maxRPM = 5700; // TODO: Figure out why lol

    // set PID Coefficeints
    a_pidController.setP(kP);
    a_pidController.setI(kI);
    a_pidController.setD(kD);
    a_pidController.setIZone(kIz);
    a_pidController.setFF(kFF);
    a_pidController.setOutputRange(kMinOutput, kMaxOutput);

    b_pidController.setP(kP);
    b_pidController.setI(kI);
    b_pidController.setD(kD);
    b_pidController.setIZone(kIz);
    b_pidController.setFF(kFF);
    b_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID Coeffiicients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    System.out.println("END robotInit()");
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    
    double shooterStickValue;
    shooterStickValue = shooter_stick_a.getRawAxis(2);    //Gets Slider Value (Axis 3)
    shooterStickValue = ((shooterStickValue + 1)/2);
    //System.out.println(shooterStickValue);              //Prints Slider Value (From 0 to 1)

    double bottomShooterStickValue;
    bottomShooterStickValue = shooter_stick_b.getRawAxis(2); //Gets Slider Value (Axis 3) from secondary joystick
    bottomShooterStickValue = ((bottomShooterStickValue + 1)/2);
    //System.out.println(bottomShooterStickValue); // Prints Slider Value (From 0 to 1)

    System.out.printf("%-30.30s  %-30.30s%n",shooterStickValue,bottomShooterStickValue);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    
    //ThreadShoot th_shoot = new ThreadShoot();
    //th_shoot.start();

  }

  @Override
  public void teleopPeriodic() {

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { a_pidController.setP(p); b_pidController.setP(p); kP = p; }
    if((i != kI)) { a_pidController.setI(i); b_pidController.setI(i); kI = i; }
    if((d != kD)) { a_pidController.setD(d); b_pidController.setD(d); kD = d; }
    if((iz != kIz)) { a_pidController.setIZone(iz); b_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { a_pidController.setFF(ff); b_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      a_pidController.setOutputRange(min, max); 
      b_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    double shooterAStickValue, shooterARPM;
    shooterAStickValue = shooter_stick_a.getRawAxis(2);    //Gets Slider Value (Axis 3)
    shooterARPM = shooterAStickValue * maxRPM;
    // shooterAStickValue = ((shooterStickValue + 1)/2);
    a_pidController.setReference(shooterARPM, CANSparkMax.ControlType.kVelocity);

    double shooterBStickValue, shooterBRPM;
    shooterBStickValue = shooter_stick_b.getRawAxis(2); //Gets Slider Value (Axis 3) from secondary joystick
    shooterBRPM = shooterBStickValue * maxRPM;
    // bottomShooterStickValue = ((shooterBStickValue + 1)/2);
    b_pidController.setReference(shooterBRPM, CANSparkMax.ControlType.kVelocity);

    // System.out.println(shooterAStickValue);  //Prints Slider Value (From 0 to 1)
    // System.out.println(shooterBStickValue); // Prints Slider Value (From 0 to 1)
    System.out.printf("%-30.30s  %-30.30s%n", shooterAStickValue, shooterBStickValue);
    SmartDashboard.putNumber("Shooter A RPM", shooterARPM);
    SmartDashboard.putNumber("Shooter B RPM", shooterBRPM);

    // intake control
    if (Robot.shooter_stick_a.getRawButton(2) == true)
      { Robot.intake.set(ControlMode.PercentOutput, 1);}
    else 
      { Robot.intake.set(ControlMode.PercentOutput, 0);}


    // Joystick Slider Control -- TALONS
    // Robot.a_motor.set(ControlMode.PercentOutput, shooterAStickValue);
    // Robot.b_motor.set(ControlMode.PercentOutput, -shooterBStickValue);

    /* // Winch Control System
     * double shooterStickY;
     * shooterStickY = shooter_stick.getY();
     *
     * Robot.winch.set(ControlMode.PercentOutput, shooterStickY);
     *
     * //Intake Control System
     * double intakeStick;
     * intakeStick = shooter_stick.getRawAxis(3);    //Gets Slider Value (Axis 3)
     * intakeStick = ((intakeStick + 1)/2)*0.4;
     * Robot.intake_1.set(ControlMode.PercentOutput, intakeStick);
     * Robot.intake_2.set(ControlMode.PercentOutput, intakeStick);
     */
  

//Previous Code
/*
    if (Robot.pilot_stick.getRawButton(2) == true)
    {
      Robot.shoottop.set(ControlMode.PercentOutput, 0.7);
      Robot.shootbottom.set(ControlMode.PercentOutput, -0.5);
    }
    else
    {
      Robot.shoottop.set(ControlMode.PercentOutput, 0);
      Robot.shootbottom.set(ControlMode.PercentOutput, 0);
    }*/

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
