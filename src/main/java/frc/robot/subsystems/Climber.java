package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  
  private CANSparkMax leftMotorController;
  private CANSparkMax rightMotorController;
  private RelativeEncoder leftEncoder, rightEncoder;
  private SparkMaxPIDController leftPIDController, rightPIDController;

  public static final NetworkTableEntry slewRate = Shuffleboard.getTab("Params").addPersistent("Climber Slew Rate", 5.0).getEntry();


  SlewRateLimiter filter;

  //private double climberSetpoint;

  public static final double CLIMBER_KP = 0;//1.875;
  public static final double CLIMBER_KI = 0;//0.006;
  public static final double CLIMBER_KD = 0;//52.5;
  public static final double CLIMBER_KF = 0.000086; //0.15;
  public static final double CLIMBER_KIZ = 0;
  public static final double CLIMBER_K_MAX_OUTPUT = 1;
  public static final double CLIMBER_K_MIN_OUTPUT = 0;
  public static final double CLIMBER_MAX_RPM = 5700;

  public Climber() {
    //initialize motor controllers
    leftMotorController = new CANSparkMax(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    rightMotorController = new CANSparkMax(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);

    //restore factory settings to reset to a known state
    leftMotorController.restoreFactoryDefaults();
    rightMotorController.restoreFactoryDefaults();

    //set climber motors to coast mode
    leftMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //initialize motor encoder
    leftEncoder = leftMotorController.getEncoder();
    rightEncoder = rightMotorController.getEncoder();

    //initialize motor pid controllers
    leftPIDController = leftMotorController.getPIDController();
    rightPIDController = rightMotorController.getPIDController();

    //assigns values to PID controllers
    leftPIDController.setP(CLIMBER_KP);
    leftPIDController.setI(CLIMBER_KI);
    leftPIDController.setD(CLIMBER_KD);
    leftPIDController.setIZone(CLIMBER_KIZ);
    leftPIDController.setFF(CLIMBER_KF);
    leftPIDController.setOutputRange(CLIMBER_K_MIN_OUTPUT, CLIMBER_K_MAX_OUTPUT);

    rightPIDController.setP(CLIMBER_KP);
    rightPIDController.setI(CLIMBER_KI);
    rightPIDController.setD(CLIMBER_KD);
    rightPIDController.setIZone(CLIMBER_KIZ);
    rightPIDController.setFF(CLIMBER_KF);
    rightPIDController.setOutputRange(CLIMBER_K_MIN_OUTPUT, CLIMBER_K_MAX_OUTPUT);

    //invert the motor controllers so climber climbs right
    leftMotorController.setInverted(false);
    rightMotorController.setInverted(false);
    
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    SmartDashboard.putNumber("Climber Slew Rate", SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));
    filter = new SlewRateLimiter(SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));
    System.out.println ("Climber SlewRateLimiter " + SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Left Climber Extended", this.isLeftExtended());
    // SmartDashboard.putBoolean("Right Climber Extended", this.isRightExtended());

  }

  public void setLeftSpeed(double speed)
  {
    speed = filter.calculate(speed);
    leftMotorController.set(speed);
  }

  public void setRightSpeed(double speed)
  {
    speed = filter.calculate(speed);
    rightMotorController.set(speed);
    // SmartDashboard.putNumber("Climber speed ", speed);
  }

  public boolean isLeftExtended()
  {
    return (leftEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
  }

  public boolean isRightExtended()
  {
    return (rightEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
  }


  public boolean isLeftRetracted()
  {
    return (leftEncoder.getPosition() < ClimberConstants.MIN_HEIGHT);
  }

  public boolean isRightRetracted()
  {
    return (rightEncoder.getPosition() < ClimberConstants.MIN_HEIGHT);
  }
}
