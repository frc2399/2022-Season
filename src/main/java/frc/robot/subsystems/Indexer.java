package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends SubsystemBase {
  
  private CANSparkMax indexerMotorController;
  private DigitalInput limitSwitch;

  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotorController = new CANSparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(IndexerConstants.LIMIT_SWITCH_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is Ball Indexed?", this.isBallIndexed());

  }

  public void setSpeed(double indexerSpeed)
  {
    indexerMotorController.set(indexerSpeed);
    SmartDashboard.putNumber("Indexer speed ", indexerSpeed);
  }

  public boolean isBallIndexed()
  {
    return !limitSwitch.get();
  }
}
