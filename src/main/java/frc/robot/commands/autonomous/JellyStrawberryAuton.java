// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// bread
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JellyStrawberryAuton extends SequentialCommandGroup {
  /** Creates a new IntakeBallShootBothP1. */

  private final DriveTrain m_driveTrain;
  private final Intake m_intake;
  private final Shooter m_shooter;

  public JellyStrawberryAuton(DriveTrain dt, Intake it, Shooter sh) {
    m_driveTrain = dt;
    m_intake = it;
    m_shooter = sh;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ExtendIntakeArm(m_intake), 
        new DriveForwardGivenDistance(0.5, 40, m_driveTrain)
      ),
        new IntakeCmdForGivenTime(m_intake, 0.5, 2),
        new DriveForwardGivenDistance(0.5, -65, m_driveTrain),

        new ParallelCommandGroup(
            new TurnNAngle(-22.5, m_driveTrain),
            new InstantCommand(() -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT, ShooterConstants.BOTTOM_SETPOINT), m_shooter)
        ),

        new TurnNAngle(-57.4, m_driveTrain),
        new DriveForwardGivenDistance(0.5, 85.2, m_driveTrain),
        new IntakeCmdForGivenTime(m_intake, 0.5, 2),
        new DriveForwardGivenDistance(0.5, -85.2, m_driveTrain),
        new TurnNAngle(54.7, m_driveTrain),
        // edit shoot speed and time?
        new ShootForNSeconds(m_shooter, Constants.ShooterConstants.SHOOTER_SPEED, Constants.ShooterConstants.SHOOTER_SPEED, 0.5)
     
    );
  }
}
