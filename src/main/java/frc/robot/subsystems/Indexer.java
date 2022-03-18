package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends SubsystemBase {

  public static final NetworkTableEntry indexSpeed = Shuffleboard.getTab("Params").add("Indexer Speed", 0).getEntry();

  private CANSparkMax indexerMotorController;
  private DigitalInput limitSwitch;

  SlewRateLimiter filter;

  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotorController = new CANSparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(IndexerConstants.LIMIT_SWITCH_ID);

    SmartDashboard.putNumber("Indexer Slew Rate", SmartDashboard.getNumber ("Indexer Slew Rate", IndexerConstants.INDEXER_SLEW));
    filter = new SlewRateLimiter(SmartDashboard.getNumber("Indexer Slew Rate", IndexerConstants.INDEXER_SLEW));
    System.out.println ("Indexer SlewRateLimiter " + SmartDashboard.getNumber("Indexer Slew Rate", IndexerConstants.INDEXER_SLEW));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Is Ball Indexed?", this.isBallIndexed());

  }

  public void setSpeed(double indexerSpeed)
  {
    indexerSpeed = filter.calculate(indexerSpeed);
    indexerMotorController.set(indexerSpeed);
    // SmartDashboard.putNumber("Indexer speed ", indexerSpeed);
  }

  public boolean isBallIndexed()
  {
    return !limitSwitch.get();
  }
}
