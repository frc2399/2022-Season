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

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;



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
  //public static Joystick copilot_stick = new Joystick(1);
  public static Joystick shooter_stick = new Joystick(1);     //Shooter controller - Logitech Extreme 3D Pro
  public static Joystick shooter_stick_bottom = new Joystick(2);     //Second Shooter controller - Logitech Extreme 3D Pro
  
  public static TalonSRX shoottop = new TalonSRX(1);
  public static TalonSRX shootbottom = new TalonSRX(2);
  // public static TalonSRX winch = new TalonSRX(3);

  // public static TalonSRX intake_1 = new TalonSRX(21);
  // public static TalonSRX intake_2 = new TalonSRX(22);

//  public static Compressor air1 = new Compressor();
//  public static Solenoid intakepiston = new Solenoid(0);
//  public static Solenoid liftpiston = new Solenoid(1);
  
  @Override
  public void robotInit() {

    System.out.println("BEGIN robotInit()");
    
    // factory default all falcons
    shoottop.configFactoryDefault();
    shootbottom.configFactoryDefault();
 
    // current limit drive falcons
    // SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
    // falconlimit.enable = true;
    // falconlimit.currentLimit =  10;
    // falconlimit.triggerThresholdCurrent = 10;
    // falconlimit.triggerThresholdTime = 0.1;
    
    //shoottop.configSupplyCurrentLimit(falconlimit);
    //shootbottom.configSupplyCurrentLimit(falconlimit);

    shoottop.setNeutralMode(NeutralMode.Coast);
    shootbottom.setNeutralMode(NeutralMode.Coast);
    
    shoottop.configVoltageCompSaturation(11);
    shootbottom.configVoltageCompSaturation(11);

    shoottop.enableVoltageCompensation(true);
    shootbottom.enableVoltageCompensation(true);

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
    shooterStickValue = shooter_stick.getRawAxis(2);    //Gets Slider Value (Axis 3)
    shooterStickValue = ((shooterStickValue + 1)/2);

    //System.out.println(shooterStickValue);              //Prints Slider Value (From 0 to 1)


    double bottomShooterStickValue;
    bottomShooterStickValue = shooter_stick_bottom.getRawAxis(2); //Gets Slider Value (Axis 3) from secondary joystick
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

    double shooterStickValue;
    shooterStickValue = shooter_stick.getRawAxis(2);    //Gets Slider Value (Axis 3)
    shooterStickValue = ((shooterStickValue + 1)/2);

    //System.out.println(shooterStickValue);              //Prints Slider Value (From 0 to 1)


    double bottomShooterStickValue;
    bottomShooterStickValue = shooter_stick_bottom.getRawAxis(2); //Gets Slider Value (Axis 3) from secondary joystick
    bottomShooterStickValue = ((bottomShooterStickValue + 1)/2);

    //System.out.println(bottomShooterStickValue); // Prints Slider Value (From 0 to 1)
    System.out.printf("%-30.30s  %-30.30s%n",shooterStickValue,bottomShooterStickValue);

    //Joystick Slider Control
    
    Robot.shoottop.set(ControlMode.PercentOutput, shooterStickValue);
    Robot.shootbottom.set(ControlMode.PercentOutput, -bottomShooterStickValue);


    //Winch Control System
    // double shooterStickY;
    // shooterStickY = shooter_stick.getY();

    // Robot.winch.set(ControlMode.PercentOutput, shooterStickY);
    
    //Intake Control System
    // double intakeStick;
    // intakeStick = shooter_stick.getRawAxis(3);    //Gets Slider Value (Axis 3)
    // intakeStick = ((intakeStick + 1)/2)*0.4;
    // Robot.intake_1.set(ControlMode.PercentOutput, intakeStick);
    // Robot.intake_2.set(ControlMode.PercentOutput, intakeStick);
  

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
