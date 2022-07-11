package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
      
    new TurnToHub(m_driveTrain),
    new InstantCommand(() -> PhotonLimelight.getDistanceToHub()),
    new InstantCommand(() -> Shooter.calculateSpeedGivenDistance()),
    new IndexerCmdForGivenTime(m_indexer, -0.2, 0.1),
    //new InstantCommand(() -> m_shooter.setOptimalSpeedWithPID(), m_shooter), 
    // commented out for hacking
    new InstantCommand(
         // () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT,ShooterConstants.BOTTOM_SETPOINT), m_shooter),
          () -> m_shooter.setOptimalSpeedWithPID()),
    new WaitUntilCommand(() -> m_shooter.correctSpeed()),
    new IndexerCmdForGivenTime(m_indexer, 0.5, 2)

    );
  }

  //Turn, then find distance between robot and hub
  //Get to Goldilocks zone (make command for driving forward to optimal shooting position)
  //Figure out shooter speed based on distance from hub - function
  //Shoot! 
}
