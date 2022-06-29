// bread
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.DriveStraightGivenDistance;
import frc.robot.commands.drivetrain.TurnNAngle;
import frc.robot.commands.indexer.IndexerCmdForGivenTime;
import frc.robot.commands.intake.IntakeCmdForGivenTime;
import frc.robot.commands.intakearm.ExtendIntakeArm;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PloopAndDriveOutAuton extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Indexer m_indexer;
  private final Shooter m_shooter;

  public PloopAndDriveOutAuton(DriveTrain dt, Shooter sh, Indexer id) {
    m_driveTrain = dt;
    m_indexer = id;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new LowerShootFromFender(m_indexer, m_shooter),  
        new DriveStraightGivenDistance(0.25, 95, m_driveTrain)
    );
  }
}
