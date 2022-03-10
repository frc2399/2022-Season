package frc.robot.subsystems;

import frc.robot.Constants.PhotonLimelightConstants;

//import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonLimelight extends SubsystemBase {

  PhotonCamera camera;

  public PhotonLimelight() {
    camera = new PhotonCamera("gloworm");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
     * double y_angle = Robot.ty.getDouble(1);
     * SmartDashboard.putNumber("y_angle", Robot.ty.getDouble(1));
     * double y_angle_radians = Math.toRadians(y_angle);
     * SmartDashboard.putNumber("y_angle_radians", y_angle_radians);
     * double y_distance =
     * LimelightConstants.HEIGHT_INCHES/(Math.tan(y_angle_radians));
     * SmartDashboard.putNumber("y_distance", y_distance);
     * 
     * double x_angle = Robot.tx.getDouble(1);
     * SmartDashboard.putNumber("x_angle", Robot.tx.getDouble(1));
     * double x_angle_radians = Math.toRadians(x_angle);
     * SmartDashboard.putNumber("x_angle_radians", x_angle_radians);
     * double x_distance = y_distance * Math.sin(x_angle_radians);
     * SmartDashboard.putNumber("x_distance", x_distance);
     */
    // System.out.println("Testing for photon limelight targets");
    var result = camera.getLatestResult();
    SmartDashboard.putBoolean("Photon Limelight hasTargets: ", result.hasTargets());
    if (result.hasTargets()) {
      // System.out.println("Photon Limelight has targets!");
      List<PhotonTrackedTarget> targets = result.getTargets();
      ArrayList<Double> x_distances = new ArrayList<>();
      ArrayList<Double> y_distances = new ArrayList<>();
      int countTargets = 0;
      for (PhotonTrackedTarget target : targets) {
        ArrayList<Double> coordinates = getTargetLocation(target, countTargets);
        x_distances.add(coordinates.get(1));
        y_distances.add(coordinates.get(0));
        String smartdashx = "X" + countTargets;
        String smartdashy = "Y" + countTargets;
        SmartDashboard.putNumber(smartdashx, coordinates.get(1));
        SmartDashboard.putNumber(smartdashy, coordinates.get(0));

        countTargets++;
      }
      double totalCenterX = 0;
      double totalCenterY = 0;
      for (int i = 0; i< targets.size() - 1; i++)
      {
        double x1 = x_distances.get(i);
        double x2 = x_distances.get(i+1);
        double y1 = y_distances.get(i);
        double y2 = y_distances.get(i+1);

        String smartdashxi = "Xi=" + i;
        String smartdashyi = "Yi=" + i;

        var circle = circle_from_p1p2r(x1, y1, x2, y2, PhotonLimelightConstants.HUB_RADIUS_FEET);
        if(circle != null)
        {
          SmartDashboard.putNumber(smartdashxi, circle.get(0));
          SmartDashboard.putNumber(smartdashyi, circle.get(1));
          totalCenterX += circle.get(0);
          totalCenterY += circle.get(1);

        }
        else
        {
          SmartDashboard.putString(smartdashxi, "null");
          SmartDashboard.putString(smartdashyi, "null");
        }
        
      }
      double averageCenterX = totalCenterX/targets.size();
      double averageCenterY = totalCenterY/targets.size();
      
      SmartDashboard.putNumber("averageCenterX", averageCenterX);
      SmartDashboard.putNumber("averageCenterY", averageCenterY);
      SmartDashboard.putNumber("NUMBER OF TARGETS", targets.size());
    }


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public ArrayList<Double> getTargetLocation(PhotonTrackedTarget target, int countTargets) {
    ArrayList<Double> coordinates = new ArrayList<>();

    String smartdashx = "Xangle" + countTargets;
    String smartdashy = "Yangle" + countTargets;

    double y_angle = target.getPitch() + PhotonLimelightConstants.TILT_DEGREES;
    double y_angle_radians = Math.toRadians(y_angle);
    double heightDifference = PhotonLimelightConstants.TARGET_HEIGHT_FEET - PhotonLimelightConstants.CAMERA_HEIGHT_FEET;
    double y_distance = heightDifference / (Math.tan(y_angle_radians));
    coordinates.add(y_distance);

    double x_angle = target.getYaw();
    double x_angle_radians = Math.toRadians(x_angle);
    double x_distance = y_distance * Math.sin(x_angle_radians);
    coordinates.add(x_distance);

    SmartDashboard.putNumber(smartdashx, x_angle);
    SmartDashboard.putNumber(smartdashy, y_angle);

    return coordinates;
  }

  public ArrayList<Double> circle_from_p1p2r(double x1, double y1, double x2, double y2, double r) {
    // Following explanation at http://mathforum.org/library/drmath/view/53027.html
    ArrayList<Double> center = new ArrayList<Double>();
    if (r == 0.0) {
      System.out.println("ERROR: radius of zero");
      return null;
    }
    if (x1 == x2 && y1 == y2) {
      System.out.println("ERROR: points one and two are equal");
      return null;
    }
    // delta x, delta y between points
    double dx = x2 - x1;
    double dy = y2 - y1;
    // dist between points
    double q = Math.sqrt(dx * dx + dy * dy);
    if (q > 2.0 * r) {
      // System.out.println("ERROR: separation of points > diameter");
      return null;
    }
    // halfway point
    double x3 = (x1 + x2) / 2;
    double y3 = (y1 + y2) / 2;
    // distance along the mirror line
    double d = Math.sqrt(r * r - (q / 2) * (q / 2));
    // One answer
    double c1x = x3 - d * dy / q;
    double c1y = y3 + d * dy / q;
    // The other answer
    double c2x = x3 + d * dy / q;
    double c2y = y3 - d * dy / q;
    if (c1y >= c2y) {
      center.add(c1x);
      center.add(c1y);
      return (center);
    }
    center.add(c2x);
    center.add(c2y);
    return (center);
  }

}