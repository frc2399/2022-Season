// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Shooter;

// public class SetShootSpeedCmd extends CommandBase {

//   private final Shooter shooterSubsystem;
//   private final double topSpeed;
//   private final double bottomSpeed;

//   /** Creates a new SetSpeedPowerCmd. */
//   public SetShootSpeedCmd(Shooter shooterSubsystem, double topSpeed, double bottomSpeed) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.shooterSubsystem = shooterSubsystem;
//     this.topSpeed = topSpeed;
//     this.bottomSpeed = bottomSpeed;
//     addRequirements(shooterSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }