package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// Import Statements for Spark Max Controllers and Neo 550 Motors
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Drivetrain subsystem.
 */
public class Shooter extends SubsystemBase {
  
  public static final NetworkTableEntry topVelocity = Shuffleboard.getTab("Driver").add("Top Velocity", 0).getEntry();
  public static final NetworkTableEntry bottomVelocity = Shuffleboard.getTab("Driver").add("Bottom Velocity", 0).getEntry();

  //instantiate motor controllers
  private CANSparkMax bottomMotorController;
  private CANSparkMax topMotorController;
  private SparkMaxPIDController topPIDController, bottomPIDController;
  private RelativeEncoder topEncoder, bottomEncoder;

  //? - don't know if these are right/go with sparks
  private double topSetpoint;
  private double bottomSetpoint;
  private static final double RANGE = 50;

  //PID constants 
 
  //constructor
  public Shooter() {
      
    //initialize motor controllers
    bottomMotorController = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    topMotorController = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);

    //restore factory settings to reset to a known state
    bottomMotorController.restoreFactoryDefaults();
    topMotorController.restoreFactoryDefaults();

    //initialize motor encoders
    bottomEncoder = bottomMotorController.getEncoder();
    topEncoder = topMotorController.getEncoder();

    //initialize motor PID controllers
    bottomPIDController = bottomMotorController.getPIDController();
    topPIDController = topMotorController.getPIDController();

    //assigns values to PID controllers
    bottomPIDController.setP(ShooterConstants.SHOOTER_KP);
    bottomPIDController.setI(ShooterConstants.SHOOTER_KI);
    bottomPIDController.setD(ShooterConstants.SHOOTER_KD);
    bottomPIDController.setIZone(ShooterConstants.SHOOTER_KIZ);
    bottomPIDController.setFF(ShooterConstants.SHOOTER_KF);
    bottomPIDController.setOutputRange(ShooterConstants.SHOOTER_K_MIN_OUTPUT, ShooterConstants.SHOOTER_K_MAX_OUTPUT);

    topPIDController.setP(ShooterConstants.SHOOTER_KP);
    topPIDController.setI(ShooterConstants.SHOOTER_KI);
    topPIDController.setD(ShooterConstants.SHOOTER_KD);
    topPIDController.setIZone(ShooterConstants.SHOOTER_KIZ);
    topPIDController.setFF(ShooterConstants.SHOOTER_KF);
    topPIDController.setOutputRange(ShooterConstants.SHOOTER_K_MIN_OUTPUT, ShooterConstants.SHOOTER_K_MAX_OUTPUT);
    
    //invert the bottom motor controller so shooter wheels spin in the right directions
    bottomMotorController.setInverted(true);
    topMotorController.setInverted(false);
    
    //bottomMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ShooterConstants.PID_IDX, ShooterConstants.CAN_TIMEOUT);
    //topMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ShooterConstants.PID_IDX, ShooterConstants.CAN_TIMEOUT);

    // the function to flip the direction of an encoder reading

    //bottomMotorController.setSensorPhase(false);
    //topMotorController.setSensorPhase(true);    
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //Set speed of top and bottom motors
  public void setMotors(double topSpeed, double bottomSpeed) {
    bottomMotorController.set(bottomSpeed);
    topMotorController.set(topSpeed); 
  }

  @Override 
  public void periodic()
  {
    topVelocity.setNumber(topEncoder.getVelocity());
    bottomVelocity.setNumber(bottomEncoder.getVelocity());
    //SmartDashboard.putBoolean("Top Speed in Range", checkWithinRange(topSetpoint, topEncoder.getVelocity(), RANGE));
    //SmartDashboard.putBoolean("Bottom Speed in Range", checkWithinRange(bottomSetpoint, bottomEncoder.getVelocity(), RANGE));

    //SmartDashboard.putNumber("P Gain", SHOOTER_KP);
    //SmartDashboard.putNumber("I Gain", SHOOTER_KI);
    //SmartDashboard.putNumber("D Gain", SHOOTER_KD);
    //SmartDashboard.putNumber("I Zone", SHOOTER_KIZ);
    //SmartDashboard.putNumber("Feed Forward", SHOOTER_KF);
    //SmartDashboard.putNumber("Max Output", SHOOTER_K_MAX_OUTPUT);
    //SmartDashboard.putNumber("Min Output", SHOOTER_K_MIN_OUTPUT);

  }

  public void runShooter ()
  {
    setMotors(ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED);
  }

  //Sets the RPM using PID
  public void setSpeedWithPID(double topRPM, double bottomRPM)
  {
    bottomPIDController.setReference(bottomRPM, CANSparkMax.ControlType.kVelocity); 
    topPIDController.setReference(topRPM, CANSparkMax.ControlType.kVelocity);
    topSetpoint = topRPM;
    bottomSetpoint = bottomRPM;
  }

  // Checks if the current value is within the range of the setpoint being passed
  public boolean checkWithinRange(double setpoint, double currentValue, double range)
  {
    return (Math.abs(currentValue-setpoint) < range);
  }

  // Checks to see if both motors are within range of the setpoints
  public boolean correctSpeed()
  {
    return (checkWithinRange(topSetpoint, topEncoder.getVelocity(), RANGE) && 
            checkWithinRange(bottomSetpoint, bottomEncoder.getVelocity(), RANGE));
  }
  //THIS NEEDS TO BE FIXED BUT ITS A WAY TO GET THE PID VALUES FROM SMART DASHBOARD
  // @Override
  // public void teleopPeriodic() {
  //   double p = SmartDashboard.getNumber("P Gain", 0);
  //   double i = SmartDashboard.getNumber("I Gain", 0);
  //   double d = SmartDashboard.getNumber("D Gain", 0);
  //   double iz = SmartDashboard.getNumber("I Zone", 0);
  //   double ff = SmartDashboard.getNumber("Feed Forward", 0);
  //   double max = SmartDashboard.getNumber("Max Output", 0);
  //   double min = SmartDashboard.getNumber("Min Output", 0);

  //   // if PID coefficients on SmartDashboard have changed, write new values to controller
  //   if((p != kP)) { a_pidController.setP(p); b_pidController.setP(p); kP = p; }
  //   if((i != kI)) { a_pidController.setI(i); b_pidController.setI(i); kI = i; }
  //   if((d != kD)) { a_pidController.setD(d); b_pidController.setD(d); kD = d; }
  //   if((iz != kIz)) { a_pidController.setIZone(iz); b_pidController.setIZone(iz); kIz = iz; }
  //   if((ff != kFF)) { a_pidController.setFF(ff); b_pidController.setFF(ff); kFF = ff; }
  //   if((max != kMaxOutput) || (min != kMinOutput)) { 
  //     a_pidController.setOutputRange(min, max); 
  //     b_pidController.setOutputRange(min, max); 
  //     kMinOutput = min; kMaxOutput = max; 
  // }


}
