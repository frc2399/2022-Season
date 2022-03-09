package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnNAngle extends CommandBase {
  /** Creates a new TurnToNAngle. */
  public double turnAngle;
  private final DriveTrain m_driveTrain;
  private double currentAngle;
  double newAngle;

  public TurnNAngle(double turnAngle, DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turnAngle = turnAngle;
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurnNAngle initialized, turnAngle: " + turnAngle);
    SmartDashboard.putNumber("turn angle", turnAngle);

    if (RobotBase.isSimulation())
    {
      currentAngle = m_driveTrain.gyroSim.getAngle().getDegrees();
      System.out.println("current angle init" + currentAngle);
    }
    else
    {
      currentAngle = m_driveTrain.getAngle();
    }
    currentAngle = modAngle(currentAngle);
    
    newAngle = modAngle(currentAngle + turnAngle);

    SmartDashboard.putNumber("new Angle", newAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isSimulation())
    {
      currentAngle = m_driveTrain.gyroSim.getAngle().getDegrees();
      System.out.println("current angle sim " + currentAngle);
    }
    else
    {
      currentAngle = m_driveTrain.getAngle();
    }
    currentAngle = modAngle(currentAngle);

    double error = newAngle - currentAngle;
    System.out.println("error angle " +  error);

    error = modAngle(error);
    SmartDashboard.putNumber("error", error);

    double outputSpeed;
    if (RobotBase.isSimulation()) 
    {
      outputSpeed = m_driveTrain.kPSim * error;
    }
    else
    {
      outputSpeed = m_driveTrain.kP * error;
    }
    outputSpeed = MathUtil.clamp(outputSpeed, -0.2, 0.2);

    m_driveTrain.setMotors(outputSpeed, -outputSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_driveTrain.setMotors(0, 0);
    
    System.out.println("TurnNangle ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double errorTolerance = SmartDashboard.getNumber("Error Tolerance", 5);
    //System.out.println("difference " + Math.abs(modAngle(newAngle - currentAngle)));
    if (Math.abs(modAngle(newAngle - currentAngle)) <= errorTolerance) {
      return true;
    }
    return false;
  }

  public double modAngle(double value) {
    return ((value + 180) % 360) - 180;
  }
}
