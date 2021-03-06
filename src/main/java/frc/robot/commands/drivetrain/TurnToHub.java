// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PhotonLimelight;

/**
 * Turns the robot until the limelight senses the hub
 * Uses a kP value to adjust robot turning speed relative to its distance to the hub
 * Used in PointAndShoot command
 */

public class TurnToHub extends CommandBase {
  /** Creates a new TurnToNAngle. */
  
  private DriveTrain m_driveTrain;

  public TurnToHub(DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurnToHub initialized!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double error = PhotonLimelight.angleToHub();
    double outputSpeed = m_driveTrain.kPSim * error;
    outputSpeed = MathUtil.clamp(outputSpeed, -0.2, 0.2);


    m_driveTrain.setMotors(outputSpeed, -outputSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_driveTrain.setMotors(0, 0);
    
    System.out.println("TurnToHub ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double errorTolerance = SmartDashboard.getNumber("Error Tolerance", 5);
    //System.out.println("difference " + Math.abs(modAngle(newAngle - currentAngle)));
    double error = PhotonLimelight.angleToHub();
    if (Math.abs(modAngle(error)) <= errorTolerance) {
      return true;
    }
    return false;
  }

  public double modAngle(double value) {
    return ((value + 180) % 360) - 180;
  }
}
