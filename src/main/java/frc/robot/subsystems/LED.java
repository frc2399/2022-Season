package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.LEDController;

public class LED extends SubsystemBase {
  LEDController red = new LEDController(Constants.LEDConstants.RED_CHANNEL);
  LEDController green = new LEDController(Constants.LEDConstants.GREEN_CHANNEL);
  LEDController blue = new LEDController(Constants.LEDConstants.BLUE_CHANNEL);
  LEDController white = new LEDController(Constants.LEDConstants.WHITE_CHANNEL);

  Timer timer = new Timer();

  boolean isPink = true;

  /** Creates a new LED. */
  public LED() {
    timer.reset();
    timer.start();
  }

public void setColor(int r, int g, int b, int w) {
  red.set(r);
  green.set(g);
  blue.set(b);
  white.set(w);
}



  @Override
  public void periodic() {

    //setColor(100, 100, 0, 100);
    if(timer.get() > Constants.LEDConstants.WAIT_TIME)
    {
      if(isPink)
      {
        setColor(Constants.LEDConstants.blue2399[0], 
        Constants.LEDConstants.blue2399[1], 
        Constants.LEDConstants.blue2399[2], 0);
      }
      else {
        setColor(Constants.LEDConstants.pink2399[0], 
        Constants.LEDConstants.pink2399[1], 
        Constants.LEDConstants.pink2399[2], 0);
      }
      isPink = !isPink;
      timer.reset();
      timer.start();
    }

  }
}