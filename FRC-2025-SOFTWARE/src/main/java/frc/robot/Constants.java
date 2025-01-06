package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    
    public class Swerve {

        public static final double MAXSPEED = Units.feetToMeters(2);
        //taken from offseason swerve 
        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
        public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
        
    }
    public static class OperatorConstants
    {
  
      // Joystick Deadband
      public static final double LEFT_X_DEADBAND  = 0.1;
      public static final double LEFT_Y_DEADBAND  = 0.1;
      public static final double RIGHT_X_DEADBAND = 0.1;
      public static final double TURN_CONSTANT    = 6;
    }
    public class Control {

    }
   //For Easier camera setup to be used with already made vision examples  
  public static class PoseCameraConstants
  {
    public static final String CAM1N = "W";
   public static final Rotation3d CAM1R = new Rotation3d(0, 0, 0);
   public static final Translation3d CAM1T = new Translation3d(Units.inchesToMeters(-4.628),
                      Units.inchesToMeters(-10.687),
                      Units.inchesToMeters(16.129));

    public static final String CAM2N = "h";
   public static final Rotation3d CAM2R = new Rotation3d(0, 0, 0);
   public static final Translation3d CAM2T = new Translation3d(Units.inchesToMeters(-4.628),
                      Units.inchesToMeters(-10.687),
                      Units.inchesToMeters(16.129));

    //it goes up to 4 but it is commented out in SwerveVision

}
}