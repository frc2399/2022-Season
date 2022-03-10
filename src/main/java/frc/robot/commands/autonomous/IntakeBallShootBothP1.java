// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// bread
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeBallShootBothP1 extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Intake m_intake;
  private final Indexer m_indexer;
  private final Shooter m_shooter;

  public IntakeBallShootBothP1(DriveTrain dt, Intake it, Shooter sh, Indexer id) {
    m_driveTrain = dt;
    m_intake = it;
    m_indexer = id;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ExtendIntakeArm(m_intake), 
        new DriveForwardGivenDistance(0.5, 40, m_driveTrain),
        new IntakeCmdForGivenTime(m_intake, 0.5, 2)
      ),
      new DriveForwardGivenDistance(0.5, -54.96, m_driveTrain),

     new ParallelCommandGroup (
        new TurnNAngle(22.5, m_driveTrain),
        new SetShootSpeedCmd(m_shooter, Constants.ShooterConstants.TOP_SETPOINT, Constants.ShooterConstants.BOTTOM_SETPOINT)
      ),
      new WaitUntilCommand(() -> m_shooter.correctSpeed()),
      new IndexerCmdForGivenTime(m_indexer, 0.5, 2)
     
    );
  }
}
