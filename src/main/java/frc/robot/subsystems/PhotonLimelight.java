package frc.robot.subsystems;

import frc.robot.Constants.PhotonLimelightConstants;
import frc.robot.util.Coords;

//import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PhotonLimelight extends SubsystemBase {

  public static final NetworkTableEntry amountTargets = Shuffleboard.getTab("Driver").add("Num of Targets", 0).getEntry();
  static PhotonCamera camera;
  public static double distanceToHub;

  Boolean photonNotFoundMessagePrinted = false ;

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

    PhotonPipelineResult result = null;

    Boolean photonExists = NetworkTableInstance.getDefault().getTable("photonvision").getEntry("gloworm").exists();


    // if (photonExists)
    // {
    //   result = camera.getLatestResult();

    //   System.out.println("photon exists!");
    // }
    // else
    // {
    //   if (!photonNotFoundMessagePrinted)
    //   {
    //     System.out.println("****** Photon Limelight not found *******");
    //     photonNotFoundMessagePrinted = true ;
    //   }
    //   System.out.println("photon does not exist!");
    //   return;
    // }

    result = camera.getLatestResult();

    Boolean has_targets = result.hasTargets() ;
    SmartDashboard.putBoolean("Photon Limelight hasTargets: ", has_targets);
    if (has_targets) {
      System.out.println("Photon Limelight has targets!");
      List<PhotonTrackedTarget> targets = result.getTargets();
      ArrayList<Double> x_distances = new ArrayList<>();
      ArrayList<Double> y_distances = new ArrayList<>();
      int countTargets = 0;
      for (PhotonTrackedTarget target : targets) {
        ArrayList<Double> coordinates = getTargetLocation(target, countTargets);
        x_distances.add(coordinates.get(0));
        y_distances.add(coordinates.get(1));
        // String smartdashx = "X" + countTargets;
        // String smartdashy = "Y" + countTargets;
        // SmartDashboard.putNumber(smartdashx, coordinates.get(1));
        // SmartDashboard.putNumber(smartdashy, coordinates.get(0));

        countTargets++;
      
      }


      System.out.println("number of targets: " + countTargets);

      amountTargets.setNumber(countTargets);
    

      double totalCenterX = 0;
      double totalCenterY = 0;
      for (int i = 0; i< targets.size() - 1; i++)
      {
        double x1 = x_distances.get(i);
        double x2 = x_distances.get(i+1);
        double y1 = y_distances.get(i);
        double y2 = y_distances.get(i+1);
        System.out.println("x coordinates: " + x1 + " " + x2);
        System.out.println("y coordinates: " + y1 + " " + y2);
        String smartdashxi = "Xi=" + i;
        String smartdashyi = "Yi=" + i;

        var circle = circle_from_p1p2r(x1, y1, x2, y2, PhotonLimelightConstants.HUB_RADIUS_INCHES);
        if(circle != null)
        {
          // SmartDashboard.putNumber(smartdashxi, circle.get(0));
          // SmartDashboard.putNumber(smartdashyi, circle.get(1));
          totalCenterX += circle.get(0);
          totalCenterY += circle.get(1);

          System.out.println("Y Circle points: " + y1 + " " + y2);
          System.out.println("Circle Center Y: " + circle.get(1));

          System.out.println("X Circle points: " + x1 + " " + x2);
          System.out.println("Circle Center X: " + circle.get(0));
        }
        else
        {
         // SmartDashboard.putString(smartdashxi, "null");
         // SmartDashboard.putString(smartdashyi, "null");
         System.out.println("No circle :(");
        }
        
      }
      System.out.println("Total CenterX: " + totalCenterX);
      System.out.println("Total CenterY: " + totalCenterY);

      
      double averageCenterX = totalCenterX/(countTargets - 1);  //targets.size();
      double averageCenterY = totalCenterY/(countTargets - 1);   //targets.size();
      
      SmartDashboard.putNumber("averageCenterX", averageCenterX);
      SmartDashboard.putNumber("averageCenterY", averageCenterY);
      SmartDashboard.putNumber("NUMBER OF TARGETS", targets.size());
      System.out.println("targets.size: " + targets.size());
      System.out.println("y distance to hub center: " + averageCenterY);
      System.out.println("x distance to hub center: " + averageCenterX);

      double angle = 180/Math.PI * (Math.atan2(averageCenterX, averageCenterY));
      System.out.println("angle!  " + angle);


    }


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static ArrayList<Double> getTargetLocation(PhotonTrackedTarget target, int countTargets) {
    ArrayList<Double> coordinates = new ArrayList<>();

    // String smartdashx = "Xangle" + countTargets;
    // String smartdashy = "Yangle" + countTargets;

    double y_angle = target.getPitch() + PhotonLimelightConstants.TILT_DEGREES;
    double y_angle_radians = Math.toRadians(y_angle);
    double heightDifference = PhotonLimelightConstants.TARGET_HEIGHT_INCHES - PhotonLimelightConstants.CAMERA_HEIGHT_INCHES;
    double y_distance = heightDifference / (Math.tan(y_angle_radians));

    double x_angle = target.getYaw();
    double x_angle_radians = Math.toRadians(x_angle);
    double x_distance = y_distance * Math.sin(x_angle_radians);
    coordinates.add(x_distance);
    coordinates.add(y_distance);

    // SmartDashboard.putNumber(smartdashx, x_angle);
    // SmartDashboard.putNumber(smartdashy, y_angle);

    return coordinates;
  }

  public static ArrayList<Double> circle_from_p1p2r(double x1, double y1, double x2, double y2, double r) {
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
    System.out.println("dist between points: " + q);
    System.out.println("radieus: " + r);
    if (q > 2.0 * r) {
      System.out.println("ERROR: separation of points > diameter");
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
  public static double angleToHub() {
    // Find the angle from the hub.
    // Get x and y values from all target centers, find the center of the circle from that
    // Then find the angle between where the robot is facing and the center of circle
   var target_centers = get_target_centers();
   var circle_center = get_hub_center_from_target_centers(target_centers);

   double angle = Math.atan2(circle_center.x, circle_center.y) * (180 / Math.PI);
   return angle;
  }

  public static void getDistanceToHub() {
    // Find the distance from the hub.
    // Get x and y values from all target centers, find the center of the circle from that
    // Then find the angle between where the robot is facing and the center of circle
   var target_centers = get_target_centers();
   var circle_center = get_hub_center_from_target_centers(target_centers);

   distanceToHub = circle_center.y;
  }

  private static Coords get_hub_center_from_target_centers(ArrayList<Coords> target_centers) {
    double totalCenterX = 0;
    double totalCenterY = 0;
    var amount_target_centers = 0;

    for (int i = 0; i < target_centers.size() - 1; i++) {
      Coords target_center_1 = target_centers.get(i);
      Coords target_center_2 = target_centers.get(i + 1);

      var circle = circle_from_p1p2r(target_center_1.x, target_center_1.y, target_center_2.x, target_center_2.y,
          PhotonLimelightConstants.HUB_RADIUS_INCHES);
      if (circle != null) {
        totalCenterX += circle.get(0);
        totalCenterY += circle.get(1);
        amount_target_centers++;
      } else {
        // SmartDashboard.putString(smartdashxi, "null");
        // SmartDashboard.putString(smartdashyi, "null");
      }

    }
    double averageCenterX = totalCenterX / (amount_target_centers - 1);
    double averageCenterY = totalCenterY / (amount_target_centers - 1);
    Coords coordinate;
    coordinate = new Coords(averageCenterX, averageCenterY);
    return coordinate;
  }

  public static ArrayList<Coords> get_target_centers () {
    ArrayList<Coords> centers = new ArrayList<Coords>();

    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      // System.out.println("Photon Limelight has targets!");
      List<PhotonTrackedTarget> targets = result.getTargets();
      int countTargets = 0;
      for (PhotonTrackedTarget target : targets) {
        ArrayList<Double> coordinates = getTargetLocation(target, countTargets);
        Coords c;
        c = new Coords(coordinates.get(0),coordinates.get(1));
        centers.add(c);
        countTargets++;
      }
    }
    return centers;
  }
}