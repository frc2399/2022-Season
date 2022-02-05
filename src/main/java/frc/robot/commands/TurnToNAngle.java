// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToNAngle extends CommandBase {
  /** Creates a new TurnToNAngle. */
  public double targetAngle;
  private final DriveTrain m_driveTrain;
  private double currentAngle;

  public TurnToNAngle(double targetAngle, DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    m_driveTrain = subsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurnToNAngle initialized, targetAngle: " + targetAngle);
    SmartDashboard.putNumber("target angle", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentAngle = m_driveTrain.getAngle();
    currentAngle = modAngle(currentAngle);

    double error = targetAngle - currentAngle;
    error = modAngle(error);
    SmartDashboard.putNumber("error", error);

    double outputSpeed = m_driveTrain.kP * error;
    outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);

    m_driveTrain.setMotors(-outputSpeed, outputSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setMotors(0, 0);
    System.out.println("TurnToNangle ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(modAngle(targetAngle - currentAngle)) <= 3) {
      return true;
    }
    return false;
  }

  public double modAngle(double value) {
    return ((value + 180) % 360) - 180;
  }
}
