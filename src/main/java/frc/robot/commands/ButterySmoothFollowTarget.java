package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ButterySmoothFollowTarget extends CommandBase {

    private final DriveTrain driveSubsystem;


    public ButterySmoothFollowTarget(DriveTrain driveSubsystem) {

        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("FollowTarget started!");
    }

    @Override
    public void execute() {
        double x_angle = Robot.tx.getDouble(1);

        this.driveSubsystem.setMotors(DriveConstants.BUTTERY_FOLLOWING_SPEED * x_angle, -DriveConstants.BUTTERY_FOLLOWING_SPEED * x_angle);
        SmartDashboard.putString("churning buttery", "churning buttery");
        SmartDashboard.putNumber("buttery speed left", DriveConstants.BUTTERY_FOLLOWING_SPEED * x_angle);
        SmartDashboard.putNumber("buttery speed right", -DriveConstants.BUTTERY_FOLLOWING_SPEED * x_angle);
        SmartDashboard.putNumber("x_angle", Robot.tx.getDouble(1));



    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}