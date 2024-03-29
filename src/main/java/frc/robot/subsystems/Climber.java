package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SimEncoder;

public class Climber extends SubsystemBase {
  
  private CANSparkMax leftMotorController;
  private CANSparkMax rightMotorController;
  private RelativeEncoder leftEncoder, rightEncoder;
  private SparkMaxPIDController leftPIDController, rightPIDController;
  private DoubleSolenoid piston;

  private static double climberDrumRadius = 0.01; //in meters
  private SimEncoder climberEncoderSim;
  private ElevatorSim climberSim; 


  public static final GenericEntry slewRate = Shuffleboard.getTab("Params").addPersistent("Climber Slew Rate", 5.0).getEntry();
  public static final GenericEntry leftClimberMotor = Shuffleboard.getTab("Driver").addPersistent("Left Climber Motor", 0).getEntry();
  public static final GenericEntry rightClimberMotor = Shuffleboard.getTab("Driver").addPersistent("Right Climber Motor", 0).getEntry();



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

    piston = new DoubleSolenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, ClimberConstants.EXTEND_PISTON, ClimberConstants.RETRACT_PISTON);


    //this code is instantiating the simulator stuff for climber
    if(RobotBase.isSimulation()) {
        climberEncoderSim = new SimEncoder("Climber");
        climberSim = new ElevatorSim(
          DCMotor.getNEO(1), //1 NEO motor on the climber
          10, //10:1 gearing ratio - this was an estimate
          0.01, //carriage mass in kg
          climberDrumRadius, //drum radius in meter
          0, //minimum height in meters
          Units.inchesToMeters(25), //maximum height in meters of climber
          true, // simulate gravity
          VecBuilder.fill(0.01) //standard deviation of the measurements, adds noise to the simulation
        ); 
    }
    this.tiltForward();

    leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
    rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Left Climber Extended", this.isLeftExtended());
    // SmartDashboard.putBoolean("Right Climber Extended", this.isRightExtended());
    SmartDashboard.putNumber("Left Climber Height", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Climber Hieght", getRightEncoderPosition());

  }

  @Override
  public void simulationPeriodic() {
    //sets input for climber motor in simulation
    climberSim.setInput(leftMotorController.get() * RobotController.getInputVoltage());
    // Next, we update it. The standard loop time is 20ms.
    climberSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    climberEncoderSim.setDistance(climberSim.getPositionMeters());
    //sets our simulated encoder speeds
    climberEncoderSim.setSpeed(climberSim.getVelocityMetersPerSecond());

    
  }

  public void setLeftSpeed(double speed)
  {
    speed = filter.calculate(speed);
    leftMotorController.set(speed);
    leftClimberMotor.setDouble(speed);
  }

  public void setRightSpeed(double speed)
  {
    speed = filter.calculate(speed);
    rightMotorController.set(speed);
    // SmartDashboard.putNumber("Climber speed ", speed);
    rightClimberMotor.setDouble(speed);

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

  public double getClimberHeight()
  {
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
  }

  public void tiltBack()
  {
    piston.set(Value.kForward);
  }

  public void tiltForward()
  {
    piston.set(Value.kReverse);
  }

  public double getLeftEncoderPosition()
  {
    if (RobotBase.isSimulation())
    {
      // simulator output is in meters, needs to be converted to inches to work with the rest of the code. encoders are already in inches
        return Units.metersToInches(climberEncoderSim.getDistance());
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
      // simulator output is in meters, needs to be converted to inches to work with the rest of the code. encoders are already in inches
    {
        return Units.metersToInches(climberEncoderSim.getDistance());
    }
    else
    {
        //gets position in inches
        return rightEncoder.getPosition();
    }
  }

}

