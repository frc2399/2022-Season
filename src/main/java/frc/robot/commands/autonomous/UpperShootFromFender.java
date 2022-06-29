package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

public class UpperShootFromFender extends SequentialCommandGroup {
    /** Creates a new IntakeBallShootBothP1. */

    private final Indexer m_indexer;
    private final Shooter m_shooter;
    private final DriveTrain m_driveTrain;


  public UpperShootFromFender(Indexer id, Shooter sh, DriveTrain dt) {

    m_indexer = id;
    m_shooter = sh;
    m_driveTrain = dt;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new DriveStraightGivenDistance(0.1, 8, m_driveTrain).withTimeout(1),
      new IndexerCmdForGivenTime(m_indexer, -0.5, 0.1),
        new InstantCommand(
            () -> m_shooter.setSpeedWithPID(ShooterConstants.FENDER_UPPER_SHOOTER_TOP_SPEED,ShooterConstants.FENDER_UPPER_SHOOTER_BOTTOM_SPEED), m_shooter),
        new WaitUntilCommand(() -> m_shooter.correctSpeed()),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 0.3),
        new WaitCommand(0.5),
        new IndexerCmdForGivenTime(m_indexer, 0.5, 1)
        );
  }
}




