//Strawberry Jelly
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drivetrain.DriveStraightGivenDistance;
import frc.robot.commands.drivetrain.TurnNAngle;
import frc.robot.commands.intake.IntakeCmdForGivenTime;
import frc.robot.commands.intakearm.ExtendIntakeArm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Position6AutonSTRAWBERRY extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Indexer m_indexer;

  public Position6AutonSTRAWBERRY(DriveTrain dt, Intake it, Shooter sh, Indexer id) {
    m_driveTrain = dt;
    m_intake = it;
    m_shooter = sh;
    m_indexer = id;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ExtendIntakeArm(m_intake), 
        new DriveStraightGivenDistance(0.5, 40, m_driveTrain)
      ),
        new IntakeCmdForGivenTime(m_intake, 0.5, 2),
        new DriveStraightGivenDistance(0.5, -65, m_driveTrain),

        new ParallelCommandGroup(
            new TurnNAngle(0.5, -22.5, m_driveTrain),
            new InstantCommand(() -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT), m_shooter)
        ),

        new TurnNAngle(0.5, -57.4, m_driveTrain),
        new DriveStraightGivenDistance(0.5, 85.2, m_driveTrain),
        new IntakeCmdForGivenTime(m_intake, 0.5, 2),
        new DriveStraightGivenDistance(0.5, -85.2, m_driveTrain),
        new TurnNAngle(0.5, 54.7, m_driveTrain),
        // edit shoot speed and time?
        //new ShootForNSeconds(m_shooter, Constants.ShooterConstants.SHOOTER_SPEED, Constants.ShooterConstants.SHOOTER_SPEED, 0.5),
        //new PointAndShoot(m_driveTrain, m_shooter, m_indexer)
        new DriveStraightGivenDistance(0.5, -38, m_driveTrain),
        //RobotContainer.upperShootFromFender
        new UpperShootFromFender(m_indexer, m_shooter, m_driveTrain)     
    );
  }
}
