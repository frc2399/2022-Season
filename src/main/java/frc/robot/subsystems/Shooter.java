// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import edu.wpi.first.wpilibj.smartdashboard.*;

// // import com.ctre.phoenix.motorcontrol.ControlMode;
// // import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// // Import Statements for Spark Max Controllers and Neo 550 Motors
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;


// /**
//  * Drivetrain subsystem.
//  */
// public class Shooter extends SubsystemBase {
  
//   //instantiate motor controllers
//   private CANSparkMax bottom;
//   private CANSparkMax top;

//   //? - don't know if these are right/go with sparks
//   private static final int PID_IDX = 0;
//   private static final int CAN_TIMEOUT = 10;
//   private static final int ENCODER_TICKS_PER_REVOLUTION = 4096;
//   private static final double GEAR_RATIO = 84.0 / 54.0;
//   private static final double TALON_100MS_IN_1S = 10.0;
//   public double topSpeed = 0.1;
//   public double bottomSpeed = 0.1;

//   public static final double SHOOTER_KP = 1.875;
// 	public static final double SHOOTER_KI = 0.006;
// 	public static final double SHOOTER_KD = 52.5;
//   public static final double SHOOTER_KF = 0.15;

//   //constructor
//   public Shooter() {
      
//     //initialize motor controllers
//     bottom = RobotMap.Shooter.PRIMARY_SHOOTER_LOWER;
//     top = RobotMap.Shooter.PRIMARY_SHOOTER_UPPER;

//     bottom.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_IDX, CAN_TIMEOUT);
//     top.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_IDX, CAN_TIMEOUT);
//     // the function to flip the direction of an encoder reading 
//     bottom.setSensorPhase(false);
//     top.setSensorPhase(true);    
    
//   }

//   //ready ball
//   public void increaseTopSpeed()
//   {
//     topSpeed += RobotMap.Shooter.SHOOTER_SPEED_INCREMENT;
//     if (topSpeed >= 1)
//     {
//       topSpeed = 1;
//     }
//     System.out.println("top speed: " + topSpeed);
//     SmartDashboard.putNumber("topSpeed ", topSpeed);
//   }

//   public void decreaseTopSpeed()
//   {
//     topSpeed -= RobotMap.Shooter.SHOOTER_SPEED_INCREMENT;
//     if (topSpeed <= -1)
//     {
//       topSpeed = -1;
//     }
//     System.out.println("top speed: " + topSpeed);
//     SmartDashboard.putNumber("topSpeed ", topSpeed);
//   }

//   public void increaseBottomSpeed()
//   {
//     bottomSpeed += RobotMap.Shooter.SHOOTER_SPEED_INCREMENT;
//     if (bottomSpeed >= 1)
//     {
//       bottomSpeed = 1;
//     }
//     System.out.println("bottom speed: " + bottomSpeed);
//     SmartDashboard.putNumber("bottomSpeed ", bottomSpeed);
//   }

//   public void decreaseBottomSpeed()
//   {
//     bottomSpeed -= RobotMap.Shooter.SHOOTER_SPEED_INCREMENT;
//     if (bottomSpeed <= -1)
//     {
//       bottomSpeed = -1;
//     }
//     System.out.println("bottom speed: " + bottomSpeed);
//     SmartDashboard.putNumber("bottomSpeed ", bottomSpeed);
//   }

//   //set bottom primary motor speed
//   public void setLowerPrimaryShooterSpeed(double speed){
//     bottom.set(ControlMode.PercentOutput, speed * - 1.0);
//   }

//   //set top secondary motor speed
//   public void setUpperPrimaryShooterSpeed(double speed){
//     top.set(ControlMode.PercentOutput, -0.25 *speed * 1.0);
//   }

//   public double toRotationsPerSecondFromNativeTalon(double talonNative) {
//     return talonNative * ((Math.PI / ENCODER_TICKS_PER_REVOLUTION)) * GEAR_RATIO * TALON_100MS_IN_1S;
//   }

//   public double getTopRotationsPerSecond() {
//     return toRotationsPerSecondFromNativeTalon(top.getSelectedSensorVelocity());
//   }

//   public double getBottomRotationsPerSecond() {
//     return toRotationsPerSecondFromNativeTalon(bottom.getSelectedSensorVelocity());
//   }

//   //set default command
//   public void initDefaultCommand(Command c) {
//     setDefaultCommand(c);
//   }

//   @Override
//   public void initDefaultCommand() {
//   }
// }
