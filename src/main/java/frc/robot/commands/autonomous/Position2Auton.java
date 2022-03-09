package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.TurnNAngle;
import frc.robot.commands.indexer.IndexerCmdForGivenTime;
import frc.robot.commands.intake.IntakeCmdForGivenTime;
import frc.robot.commands.intakearm.ExtendIntakeArm;
import frc.robot.commands.shooter.SetShootSpeedCmd;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Position2Auton extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Intake m_intake;
  private final Indexer m_indexer;
  private final Shooter m_shooter;

  public Position2Auton(DriveTrain dt, Intake it, Shooter sh, Indexer id) {
    m_driveTrain = dt;
    m_intake = it;
    m_indexer = id;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ExtendIntakeArm(m_intake), 
        new DriveForwardGivenDistance(0.5, 67.5, m_driveTrain),
        new IntakeCmdForGivenTime(m_intake, 0.5, 2)
      ),

        new TurnNAngle(-90, m_driveTrain),
        new DriveForwardGivenDistance(0.5, 34, m_driveTrain),
        new TurnNAngle(90, m_driveTrain),
        new DriveForwardGivenDistance(0.5, -91.127, m_driveTrain),

        new ParallelCommandGroup(
            new TurnNAngle(-45.6, m_driveTrain),
            new SetShootSpeedCmd(m_shooter, Constants.ShooterConstants.TOP_SETPOINT, Constants.ShooterConstants.BOTTOM_SETPOINT)
        ),
    
      new WaitUntilCommand(() -> m_shooter.correctSpeed()),
      new IndexerCmdForGivenTime(m_indexer, 0.5, 2)
     
    );
  }
}
