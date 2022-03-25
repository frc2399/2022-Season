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
import frc.robot.commands.robot.PointAndShoot;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Position3AutonJELLY extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Intake m_intake;
  private final Indexer m_indexer;
  private final Shooter m_shooter;

  public Position3AutonJELLY(DriveTrain dt, Intake it, Shooter sh, Indexer id) {
    m_driveTrain = dt;
    m_intake = it;
    m_indexer = id;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
     //addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ExtendIntakeArm(m_intake), 
        new DriveStraightGivenDistance(0.5, 40, m_driveTrain)
      ),
      new IntakeCmdForGivenTime(m_intake, 0.5, 2),
      new DriveStraightGivenDistance(0.5, -65, m_driveTrain),
      new ParallelCommandGroup (
        new TurnNAngle(0.5, -22.5, m_driveTrain)
        // new InstantCommand(() -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT), m_shooter)
      ),
      // new WaitUntilCommand(() -> m_shooter.correctSpeed()),
      // new IndexerCmdForGivenTime(m_indexer, 0.5, 2)
      //new PointAndShoot(m_driveTrain, m_shooter, m_indexer)
      new DriveStraightGivenDistance(0.5, -38, m_driveTrain),
      //RobotContainer.upperShootFromFender
      new LowerShootFromFender(m_indexer, m_shooter)

    );
  }
}