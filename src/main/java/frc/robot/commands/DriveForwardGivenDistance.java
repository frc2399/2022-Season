
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

// import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveForwardGivenDistance extends CommandBase {

    //insantiate global variables
    double sp;
    double cp;
    double td;
    DriveTrain m_driveTrain;
    
 
	public DriveForwardGivenDistance(double speed, double targetDistance, DriveTrain subsystem) {
        
        //initialize variables
        sp = speed;
        td = targetDistance;
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

        //set command to be interruptible
		//setInterruptible(true);
    }
    

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
        cp = 0;
        m_driveTrain.leftEncoder.setPosition(0);
        m_driveTrain.rightEncoder.setPosition(0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        // Get the average position between leftEncoder and rightEncoder
        cp = ((m_driveTrain.leftEncoder.getPosition() + m_driveTrain.rightEncoder.getPosition()) / 2);

        SmartDashboard.putNumber("current position", cp);

        double error = td - cp;

        SmartDashboard.putNumber("error distance", error);

        double outputSpeed = m_driveTrain.kP * error;
        outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);

        m_driveTrain.setMotors(outputSpeed, outputSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        double butteryErrorTolerance = SmartDashboard.getNumber("Error Tolerance Distance", 3);
        if (Math.abs(td - cp) <= butteryErrorTolerance)
        {
            return true;
        }
        return false;
        
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.setMotors(0, 0);
        System.out.println("DriveForwardGivenDistance ended");
    }
}
