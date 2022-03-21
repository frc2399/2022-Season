package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class Indexer extends SubsystemBase {


  private CANSparkMax indexerMotorController;
  private DigitalInput limitSwitch;

  SlewRateLimiter filter;

  public static NetworkTableEntry indexSpeedEntry, slewRateEntry;

  /** Creates a new Indexer. */
  public Indexer() {

    ShuffleboardLayout indexerParamsLayout = Shuffleboard.getTab("Params")
      .getLayout("Indexer", BuiltInLayouts.kList)
      .withSize(1, 2)
      .withPosition(1, 1)
    ;
    // .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
  
  
    // SimpleWidget indexSpeedWidget =  Shuffleboard.getTab("Params").addPersistent("Indexer Speed", 0.3);
    // indexSpeedEntry = indexSpeedWidget.getEntry();
    // SimpleWidget slewRateWidget = Shuffleboard.getTab("Params").addPersistent("Indexer Slew Rate", 5.0);
    // slewRateEntry = indexSpeedWidget.getEntry();

    indexSpeedEntry = indexerParamsLayout.addPersistent("indexer Speed", 0.3).getEntry();
    slewRateEntry = indexerParamsLayout.addPersistent("Indexer Slew Rate", 5.0).getEntry();  

    indexerMotorController = new CANSparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(IndexerConstants.LIMIT_SWITCH_ID);

    // SmartDashboard.putNumber("Indexer Slew Rate", SmartDashboard.getNumber ("Indexer Slew Rate", IndexerConstants.INDEXER_SLEW));

    filter = new SlewRateLimiter(slewRateEntry.getDouble(0));

    //set indexer motor to coast mode
    indexerMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);

    indexerMotorController.setInverted(true);

    // filter = new SlewRateLimiter(SmartDashboard.getNumber("Indexer Slew Rate", IndexerConstants.INDEXER_SLEW));
    // System.out.println ("Indexer SlewRateLimiter " + SmartDashboard.getNumber("Indexer Slew Rate", IndexerConstants.INDEXER_SLEW));
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
  }

  public boolean isBallIndexed()
  {
    return !limitSwitch.get();
  }
}
