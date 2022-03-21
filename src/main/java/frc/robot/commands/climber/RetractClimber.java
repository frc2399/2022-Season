package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RetractClimber extends CommandBase {

    private final Climber m_climber;
    private final double speed;


    public RetractClimber (Climber climber, double speed) {
        this.m_climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        this.m_climber.setLeftSpeed(-speed);
        this.m_climber.setRightSpeed(-speed);


    //     if (m_climber.isLeftRetracted()){
    //     this.m_climber.setLeftSpeed(0);   
    //    }
    //    else {
    //     this.m_climber.setLeftSpeed(-speed);
    //    };

    //    if (m_climber.isRightRetracted()){
    //     this.m_climber.setRightSpeed(0);   
    //    }
    //    else {
    //     this.m_climber.setRightSpeed(-speed);
    //    };
    }

    @Override
    public void end(boolean interrupted) {
        this.m_climber.setLeftSpeed(0);
        this.m_climber.setRightSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // if (m_climber.isLeftRetracted() && m_climber.isRightRetracted()) {
        //     return true;
        // }
       return false;
    }
}