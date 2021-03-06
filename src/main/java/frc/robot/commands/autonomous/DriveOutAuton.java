package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveStraightGivenDistance;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOutAuton extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;

  public DriveOutAuton(DriveTrain dt) {
    m_driveTrain = dt;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStraightGivenDistance(0.25, 54.96, m_driveTrain)
    );
  }
}
