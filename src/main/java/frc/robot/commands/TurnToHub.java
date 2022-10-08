// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.PhotonLimelightConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PhotonLimelight;

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
  
    PhotonLimelight.turnLEDOn();
  
    System.out.println("TurnToHub initialized!");

    // Sets motors to brake mode
    DriveTrain.autonomousInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double error = PhotonLimelight.getAngleToHub();
    SmartDashboard.putNumber("hub error", error);

    double outputSpeed = Constants.DriveConstants.TURN_TO_HUB_KP * -error;
    outputSpeed = MathUtil.clamp(outputSpeed, -0.1, 0.1);

    SmartDashboard.putNumber("hub outputSpeed", outputSpeed);
    System.out.println("Turn to hub outputSpeed" + outputSpeed);

    m_driveTrain.setMotors(outputSpeed, -outputSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_driveTrain.setMotors(0, 0);

  // Sets motors to correct mode
    if (RobotState.isAutonomous())
    {
      DriveTrain.autonomousInit();
    }
    else if (RobotState.isTeleop())
    {
      DriveTrain.teleopInit();
    }
    
    System.out.println("TurnToHub ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   double errorTolerance = PhotonLimelightConstants.ANGLE_ERROR_TOLERANCE;
    // SmartDashboard.getNumber("Error Tolerance", 5);
    //System.out.println("difference " + Math.abs(modAngle(newAngle - currentAngle)));
    double error = PhotonLimelight.getAngleToHub();
    System.out.println("Error: " + error);
    System.out.println("Error Tolerance: " + errorTolerance);
    if (PhotonLimelight.has_targets  && Math.abs(modAngle(error)) <= errorTolerance ){
      System.out.println("turn to hub finished!!");
      
      PhotonLimelight.turnLEDOff();

      return true;
    }
    return false;
  }

  public double modAngle(double value) {
    return ((value + 180) % 360) - 180;
  }
}
