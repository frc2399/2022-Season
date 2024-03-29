package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// Import Statements for Spark Max Controllers and Neo 550 Motors
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

// import java.io.BufferedReader;
// import java.io.FileNotFoundException;
// import java.io.FileReader;
// import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Drivetrain subsystem.
 */
public class Shooter extends SubsystemBase {

  public static final GenericEntry topVelocity = Shuffleboard.getTab("Shooter").add("Top Velocity", 0).getEntry();
  public static final GenericEntry bottomVelocity = Shuffleboard.getTab("Shooter").add("Bottom Velocity", 0).getEntry();

  public static final GenericEntry p = Shuffleboard.getTab("Shooter").add("P Gain", 0).getEntry();
  public static final GenericEntry d = Shuffleboard.getTab("Shooter").add("D Gain", 0).getEntry();
  public static final GenericEntry i = Shuffleboard.getTab("Shooter").add("I Gain", 0).getEntry();
  public static final GenericEntry iz = Shuffleboard.getTab("Shooter").add("I Zone", 0).getEntry();
  public static final GenericEntry ff = Shuffleboard.getTab("Shooter").add("Feed Forward", 0).getEntry();
  public static final GenericEntry max = Shuffleboard.getTab("Shooter").add("Max Output", 0).getEntry();
  public static final GenericEntry min = Shuffleboard.getTab("Shooter").add("Min OUtput", 0).getEntry();
  public static final GenericEntry topSpeedInRange = Shuffleboard.getTab("Shooter").add("Top Speed in Range", false).getEntry();
  public static final GenericEntry bottomSpeedInRange = Shuffleboard.getTab("Shooter").add("Bottom Speed in Range", false).getEntry();
  public static final GenericEntry topSetpointEntry = Shuffleboard.getTab("Shooter").add("Top Setpoint", 0).getEntry();
  public static final GenericEntry bottomSetpointEntry = Shuffleboard.getTab("Shooter").add("Bottom Setpoint", 0).getEntry();
  public static final GenericEntry maxAccel = Shuffleboard.getTab("Shooter").add("Maximum Acceleration", 0).getEntry();

  double kP;
  double kI;
  double kD;
  double kIz;
  double kFF;
  double kMaxOutput;
  double kMinOutput;
  double kMaxAccel;

  Double[][] arr = 
  {
    {4.,2875.,0.2},
    {5.,2925.,0.2},
    {6.,3025.,0.3},
    {8.,2995.,0.45},
    {9.,2995.,0.4},
    {10.,2050.,1.},
    {11.,1225.,3.},
    {12.,1700.,2.},
    {13.,1700.,2.},
    {14.,1725.,2.},
    {15.,1500.,3.},
    {16.,1550.,3.5},
    {17.,2125.,2.5}    
  };

  // instantiate motor controllers
  private CANSparkMax bottomMotorController;
  private CANSparkMax topMotorController;
  private SparkMaxPIDController topPIDController, bottomPIDController;
  private RelativeEncoder topEncoder, bottomEncoder;

  // ? - don't know if these are right/go with sparks
  private double topSetpoint;
  private double bottomSetpoint;
  // private static final double RANGE = 650;
  private static final double RANGE_PERCENT = .02;

  public static double calculatedOptimalTopSpeed;
  public static double calculatedOptimalBottomSpeed;

  static List<List<Double>> shooterTable;

  // PID constants

  // constructor
  public Shooter() {

    shooterTable = new ArrayList<List<Double>>();
   for(Double[] row : arr) {
      shooterTable.add(Arrays.asList(row));
   }

   // Shuffleboard.getTab("Shooter").add("Stop Shooter", new DriveForwardGivenTime(0.3, 0.5, m_driveTrain));

    // initialize motor controllers
    bottomMotorController = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    topMotorController = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);

    // restore factory settings to reset to a known state
    bottomMotorController.restoreFactoryDefaults();
    topMotorController.restoreFactoryDefaults();

    //set shooter motors to coast mode
    bottomMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
    topMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);

    //initialize motor encoders
    bottomEncoder = bottomMotorController.getEncoder();
    topEncoder = topMotorController.getEncoder();

    // initialize motor PID controllers
    bottomPIDController = bottomMotorController.getPIDController();
    topPIDController = topMotorController.getPIDController();

    // assigns values to PID controllers
    bottomPIDController.setP(ShooterConstants.SHOOTER_KP);
    bottomPIDController.setI(ShooterConstants.SHOOTER_KI);
    bottomPIDController.setD(ShooterConstants.SHOOTER_KD);
    bottomPIDController.setIZone(ShooterConstants.SHOOTER_KIZ);
    bottomPIDController.setFF(ShooterConstants.SHOOTER_KF);
    bottomPIDController.setOutputRange(ShooterConstants.SHOOTER_K_MIN_OUTPUT, ShooterConstants.SHOOTER_K_MAX_OUTPUT);
    bottomPIDController.setSmartMotionMaxAccel(ShooterConstants.SHOOTER_MAX_ACCEL, 0);

    topPIDController.setP(ShooterConstants.SHOOTER_KP);
    topPIDController.setI(ShooterConstants.SHOOTER_KI);
    topPIDController.setD(ShooterConstants.SHOOTER_KD);
    topPIDController.setIZone(ShooterConstants.SHOOTER_KIZ);
    topPIDController.setFF(ShooterConstants.SHOOTER_KF);
    topPIDController.setOutputRange(ShooterConstants.SHOOTER_K_MIN_OUTPUT, ShooterConstants.SHOOTER_K_MAX_OUTPUT);
    topPIDController.setSmartMotionMaxAccel(ShooterConstants.SHOOTER_MAX_ACCEL, 0);

    // invert the bottom motor controller so shooter wheels spin in the right
    // directions
    bottomMotorController.setInverted(false);
    topMotorController.setInverted(false);

    // bottomMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
    // ShooterConstants.PID_IDX, ShooterConstants.CAN_TIMEOUT);
    // topMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
    // ShooterConstants.PID_IDX, ShooterConstants.CAN_TIMEOUT);

    // the function to flip the direction of an encoder reading

    // bottomMotorController.setSensorPhase(false);
    // topMotorController.setSensorPhase(true);

    System.out.println("Working Directory = " + System.getProperty("user.dir"));

    // try {
      double[] speedsBottomTop = shootingInterpolation(5.2, shooterTable);
      System.out.println("Bottom Motor Speed: " + speedsBottomTop[0]);
      System.out.println("Top Motor Speed: " + speedsBottomTop[1]);
    // }
    // catch (IOException e) {
    //   System.out.println("Can't read shooter table");
    // }

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Set speed of top and bottom motors
  public void setMotors(double topSpeed, double bottomSpeed) {
    bottomMotorController.set(bottomSpeed);
    topMotorController.set(topSpeed);
  }

  @Override
  public void periodic() {
    updatePIDGains();
    topVelocity.setDouble(topEncoder.getVelocity());
    bottomVelocity.setDouble(bottomEncoder.getVelocity());
    topSpeedInRange.setBoolean(checkWithinRange(topSetpoint, topEncoder.getVelocity(), (topSetpoint*RANGE_PERCENT)));
    bottomSpeedInRange.setBoolean(checkWithinRange(bottomSetpoint, bottomEncoder.getVelocity(), (bottomSetpoint*RANGE_PERCENT)));
    topSetpointEntry.setDouble(topSetpoint);
    bottomSetpointEntry.setDouble(bottomSetpoint);
  }

  public void runShooter() {
    setMotors(ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED);
  }

  // Sets the RPM using PID
  public void setSpeedWithPID(double topRPM, double bottomRPM) {
    bottomPIDController.setReference(bottomRPM, CANSparkMax.ControlType.kVelocity);
    topPIDController.setReference(topRPM, CANSparkMax.ControlType.kVelocity);
    topSetpoint = topRPM;
    bottomSetpoint = bottomRPM;
  }

  // Checks if the current value is within the range of the setpoint being passed
  public boolean checkWithinRange(double setpoint, double currentValue, double range) {
    return (Math.abs(currentValue - setpoint) < range);
  }

  // Checks to see if both motors are within range of the setpoints
  public boolean correctSpeed() {
    SmartDashboard.putNumber("topSetpoint", topSetpoint);
    double topEncoderVelocity = topEncoder.getVelocity();
    double bottomEncoderVelocity = bottomEncoder.getVelocity();
    SmartDashboard.putNumber("top encoder velocity", topEncoderVelocity);
    SmartDashboard.putNumber("bottom encoder velocity", bottomEncoderVelocity);
    SmartDashboard.putNumber("bottomSetpoint", bottomSetpoint);
    return (checkWithinRange(topSetpoint, topEncoderVelocity, (topSetpoint*RANGE_PERCENT)) &&
        checkWithinRange(bottomSetpoint, bottomEncoderVelocity, (bottomSetpoint*RANGE_PERCENT)));
  }

  // THIS NEEDS TO BE FIXED BUT ITS A WAY TO GET THE PID VALUES FROM SMART
  // DASHBOARD
  public void updatePIDGains() {
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p.getDouble(0) != kP)) { bottomPIDController.setP(p.getDouble(0)); topPIDController.setP(p.getDouble(0)); kP = p.getDouble(0); }
    if((i.getDouble(0) != kI)) { bottomPIDController.setI(i.getDouble(0)); topPIDController.setI(i.getDouble(0)); kI = i.getDouble(0); }
    if((d.getDouble(0) != kD)) { bottomPIDController.setD(d.getDouble(0)); topPIDController.setD(d.getDouble(0)); kD = d.getDouble(0); }
    if((iz.getDouble(0) != kIz)) { bottomPIDController.setIZone(iz.getDouble(0)); topPIDController.setIZone(iz.getDouble(0)); kIz = iz.getDouble(0); }
    if((ff.getDouble(0) != kFF)) { bottomPIDController.setFF(ff.getDouble(0)); topPIDController.setFF(ff.getDouble(0)); kFF = ff.getDouble(0); }
    if((maxAccel.getDouble(0) != kMaxAccel)) { bottomPIDController.setSmartMotionMaxAccel(maxAccel.getDouble(0), 0); topPIDController.setSmartMotionMaxAccel(maxAccel.getDouble(0), 0); kMaxAccel = maxAccel.getDouble(0); }
    if((max.getDouble(0) != kMaxOutput) || (min.getDouble(0) != kMinOutput)) { 
      bottomPIDController.setOutputRange(min.getDouble(0), max.getDouble(0)); 
      topPIDController.setOutputRange(min.getDouble(0), max.getDouble(0)); 
      kMinOutput = min.getDouble(0); kMaxOutput = max.getDouble(0); 
  }
}

// List<List<Double>> table = readData("ShootingTable.csv");
//     for(List<Double> row : table ) {
//         for(Double cell : row)
//         {
//             System.out.println(cell);
//         }
//     }
//     double[] speedsBottomTop = shootingInterpolation(30.0, table);
//     System.out.println("Bottom Motor Speed: " + speedsBottomTop[0]);
//     System.out.println("Top Motor Speed: " + speedsBottomTop[1]);

// public static List<List<Double>> readData(String file) throws IOException { 
//   List<List<Double>> content = new ArrayList<>();
//   try(BufferedReader br = new BufferedReader(new FileReader(file))) {
//       String line = "";
//       while ((line = br.readLine()) != null) {
//           List<Double> cells = new ArrayList<>();
//           for(String cell : line.split(",")) {
//               Double d = Double.parseDouble(cell);
//               cells.add(d.doubleValue());
//           }
//           content.add(cells);
//       }
//   } catch (FileNotFoundException e) {
//     //Some error logging
//   }
//   return content;
// }


public static double[] shootingInterpolation(Double distance, List<List<Double>> values) {
  double[] speedsBottomTop = new double[2];
  for (int i = 0; i < (values.size()-1); i++) {
      if(distance > values.get(i).get(0) && distance < values.get(i+1).get(0)) {
          double d0, d1, sb0, sb1, sr0, sr1;
          d0 = values.get(i).get(0);
          d1 = values.get(i+1).get(0);
          sb0 = values.get(i).get(1);
          sb1 = values.get(i+1).get(1);
          sr0 = values.get(i).get(2);
          sr1 = values.get(i+1).get(2);
          
          double sbm = (sb1-sb0)/(d1-d0);
          double srm = (sr1-sr0)/(d1-d0);
          double sb = sbm * (distance - d0) + sb0;
          double sr = srm * (distance - d0) + sr0;
          speedsBottomTop[0] = sb;
          speedsBottomTop[1] = sr*sb;
          break;
      }
  }
  return speedsBottomTop;
}

public static void calculateSpeedGivenDistance()
{
  // double[] optimalSpeeds = shootingInterpolation(PhotonLimelight.distanceToHub, shooterTable);
  // calculatedOptimalBottomSpeed = optimalSpeeds[0];
  // calculatedOptimalTopSpeed = optimalSpeeds[1];

  calculatedOptimalBottomSpeed = ShooterConstants.TARMAC_UPPER_SHOOTER_BOTTOM_SPEED;
  calculatedOptimalTopSpeed = ShooterConstants.TARMAC_UPPER_SHOOTER_TOP_SPEED;
}

public void setOptimalSpeedWithPID() {
  //bottomPIDController.setReference(calculatedOptimalBottomSpeed, CANSparkMax.ControlType.kVelocity);
  //topPIDController.setReference(calculatedOptimalTopSpeed, CANSparkMax.ControlType.kVelocity);
  topSetpoint = calculatedOptimalTopSpeed;
  bottomSetpoint = calculatedOptimalBottomSpeed;
  setMotors(topSetpoint, bottomSetpoint);
}
}


