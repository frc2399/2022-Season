package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetShootPowerCmd extends CommandBase {

  //declare member variables
  private final Shooter shooterSubsystem;
  private final double topSpeed;
  private final double bottomSpeed;

  public SetShootPowerCmd(Shooter shooterSubsystem, double topSpeed, double bottomSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SetShootPowerCmd started! (" + topSpeed + ", " + bottomSpeed + ")");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //tell the shooter to move the motors
    this.shooterSubsystem.setMotors(topSpeed, bottomSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SetShootPowerCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}