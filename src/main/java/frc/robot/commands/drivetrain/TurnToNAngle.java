package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;

/**
 * Turns the robot to a target angle (value is passed through the constructor)
 * factors in the robot's current angle to calculate the angle the robot needs to turn in order to reach its target
 * Used in turnAuto sequential auton command
 */

public class TurnToNAngle extends CommandBase {
  /** Creates a new TurnToNAngle. */
  public double targetAngle;
  private final DriveTrain m_driveTrain;
  private double currentAngle;

  public TurnToNAngle(double targetAngle, DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurnToNAngle initialized, targetAngle: " + targetAngle);
    // SmartDashboard.putNumber("target angle", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentAngle = m_driveTrain.getAngle();
    currentAngle = modAngle(currentAngle);

    double error = targetAngle - currentAngle;
    error = modAngle(error);
    // SmartDashboard.putNumber("error", error);

    double outputSpeed = m_driveTrain.kP * error;
    outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);

    m_driveTrain.setMotors(outputSpeed, -outputSpeed);

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
    double errorTolerance = DriveTrain.angleErrorTolerance.getDouble(3);
    if (Math.abs(modAngle(targetAngle - currentAngle)) <= errorTolerance) {
      return true;
    }
    return false;
  }

  public double modAngle(double value) {
    return ((value + 180) % 360) - 180;
  }
}
