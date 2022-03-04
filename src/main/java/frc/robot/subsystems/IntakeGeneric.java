// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeGeneric extends SubsystemBase {
  /** Creates a new ShooterGeneric. */
  IntakeBase intake;
  public IntakeGeneric(IntakeBase intake) {
    this.intake = intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double intakeSpeed) {
    intake.setMotor(intakeSpeed);
    System.out.println("intake setspeed" + intakeSpeed);
    SmartDashboard.putNumber("Intake speed ", intakeSpeed);
    }

}
