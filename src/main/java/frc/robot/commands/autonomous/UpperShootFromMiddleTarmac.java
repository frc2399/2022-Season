package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.indexer.IndexerCmdForGivenTime;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class UpperShootFromMiddleTarmac extends SequentialCommandGroup {
    /** Creates a new IntakeBallShootBothP1. */

    private final Indexer m_indexer;
    private final Shooter m_shooter;

  public UpperShootFromMiddleTarmac(Indexer id, Shooter sh) {

    m_indexer = id;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        new IndexerCmdForGivenTime(m_indexer, -0.5, 0.1),
        new InstantCommand(
            () -> m_shooter.setSpeedWithPID(ShooterConstants.MIDDLE_AUTON_TARMAC_UPPER_SHOOTER_TOP_SPEED, ShooterConstants.MIDDLE_AUTON_TARMAC_UPPER_SHOOTER_BOTTOM_SPEED), m_shooter),
        new WaitUntilCommand(() -> m_shooter.correctSpeed()),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 0.3),
        new WaitCommand(0.5),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 1)
      );
  }
}





