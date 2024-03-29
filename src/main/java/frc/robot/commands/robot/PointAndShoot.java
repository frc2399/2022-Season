package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.TurnToHub;
import frc.robot.commands.indexer.IndexerCmdForGivenTime;
import frc.robot.subsystems.*;
import frc.robot.Constants.ShooterConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PointAndShoot extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Indexer m_indexer;
  private final Shooter m_shooter;

  public PointAndShoot(DriveTrain dt, Shooter sh, Indexer id) {
    m_driveTrain = dt;
    m_indexer = id;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
    
    new TurnToHub(m_driveTrain).withTimeout(1.0),
    // new InstantCommand(() -> PhotonLimelight.getDistanceToHub()),
    // new InstantCommand(() -> Shooter.calculateSpeedGivenDistance()),
    new IndexerCmdForGivenTime(m_indexer, -0.2, 0.5),
    new PrintCommand("Indexer backup completed!"),
    //new InstantCommand(() -> m_shooter.setOptimalSpeedWithPID(), m_shooter), 
    // commented out for hacking
    new InstantCommand(
         () -> m_shooter.setSpeedWithPID(ShooterConstants.TARMAC_UPPER_SHOOTER_TOP_SPEED,ShooterConstants.TARMAC_UPPER_SHOOTER_BOTTOM_SPEED), m_shooter),
          // () -> m_shooter.setOptimalSpeedWithPID(),
    new InstantCommand(
           // () -> m_shooter.setSpeedWithPID(ShooterConstants.TARMAC_UPPER_SHOOTER_TOP_SPEED, ShooterConstants.TARMAC_UPPER_SHOOTER_BOTTOM_SPEED), m_shooter),
    // new PrintCommand("Optimal speed set with PID!")
    // new PrintCommand("correct speed attained!")

    ),
    new WaitUntilCommand(() -> m_shooter.correctSpeed()).withTimeout(2.0),
    new IndexerCmdForGivenTime(m_indexer, 0.5, 2),
    new PrintCommand("Indexer for given time worked!!!!!")
    );
  }

  //Turn, then find distance between robot and hub
  //Get to Goldilocks zone (make command for driving forward to optimal shooting position)
  //Figure out shooter speed based on distance from hub - function
  //Shoot! 
}
