// bread
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveStraightGivenDistance;
import frc.robot.commands.robot.UpperShootFromFender;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PopAndDriveOutAuton extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Indexer m_indexer;
  private final Shooter m_shooter;

  public PopAndDriveOutAuton(DriveTrain dt, Shooter sh, Indexer id) {
    m_driveTrain = dt;
    m_indexer = id;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new UpperShootFromFender(m_indexer, m_shooter, m_driveTrain),
        new DriveStraightGivenDistance(0.25, 95, m_driveTrain)
    );
  }
}
