// stale bread
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveStraightGivenDistance;
import frc.robot.commands.drivetrain.TurnNAngle;
import frc.robot.commands.indexer.IndexerCmdForGivenTime;
import frc.robot.commands.intake.IntakeCmdForGivenTime;
import frc.robot.commands.intakearm.ExtendIntakeArm;
import frc.robot.commands.robot.UpperShootFromMiddleTarmac;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Position4AutonStale extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Intake m_intake;
  private final Indexer m_indexer;
  private final Shooter m_shooter;

  public Position4AutonStale(DriveTrain dt, Intake it, Shooter sh, Indexer id) {
    m_driveTrain = dt;
    m_intake = it;
    m_indexer = id;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
     //addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // extends intake and leaves tarmac and intakes ball
      new ParallelDeadlineGroup(
        new DriveStraightGivenDistance(0.5, 40, m_driveTrain),
        new ExtendIntakeArm(m_intake), 
        new IntakeCmdForGivenTime(m_intake, 0.5, 2)
        ),

      new IntakeCmdForGivenTime(m_intake, 0.5, 0.5),

      // Drives backward
      new DriveStraightGivenDistance(0.5, -60, m_driveTrain),

      // turns and indexes a little
      new ParallelCommandGroup ( 
        new TurnNAngle(0.5, 22.5, m_driveTrain),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 2)
      ),
      // Shoots ball from middle of tarmac
      new UpperShootFromMiddleTarmac(m_indexer, m_shooter),
      new InstantCommand(() -> m_shooter.setMotors(0, 0)),
      // turns
      new TurnNAngle(0.5, 57.4, m_driveTrain),

      // drives straight and intakes
      new ParallelDeadlineGroup(
        new DriveStraightGivenDistance(0.5, 87, m_driveTrain),
        new IntakeCmdForGivenTime(m_intake, 0.5, 2),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 0.5)
      ),

      new ParallelCommandGroup(
        new IntakeCmdForGivenTime(m_intake, 0.5, 0.5),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 0.5)
      ),

      
      // intakes ball and drives backwards
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new DriveStraightGivenDistance(0.5, -87, m_driveTrain),
          new TurnNAngle(0.5, -57.4, m_driveTrain)
          // for in case the upper middle tarmac shot is not good
          //new DriveStraightGivenDistance(0.5, -38, m_driveTrain)
      ),
        new IntakeCmdForGivenTime(m_intake, 0.5, 0.5),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 2)
        
      ),
      // turns left to face hub

      // new WaitUntilCommand(() -> m_shooter.correctSpeed()),
      // new IndexerCmdForGivenTime(m_indexer, 0.5, 2)
      //new PointAndShoot(m_driveTrain, m_shooter, m_indexer)

      // drive backward to line up with fender
      //RobotContainer.upperShootFromFender

      // Shoot from fender
      new UpperShootFromMiddleTarmac(m_indexer, m_shooter)

    );
  }
}