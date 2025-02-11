package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;


public class Constants {
    
    public class Swerve {

      public static Pose2d[][] targetPosition = new Pose2d[23][3];
      

      


        public static boolean VISION = true;

        public static final double MAXSPEED = Units.feetToMeters(16);
        //taken from offseason swerve 
        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
        public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
        // Create and configure a drivetrain simulation configuration
public static final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofPigeon2())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(new SwerveModuleSimulationConfig(
                DCMotor.getNEO(1), // Drive motor is a NEO
                DCMotor.getNEO(1), // Steer motor is a NEO
                6.75, // Drive motor gear ratio. SDS L2 GEAR RATIO
                150/7, // Steer motor gear ratio. SDS L2 GEAR RATIO
                Volts.of(0.1), // Drive friction voltage.
                Volts.of(0.1), // Steer friction voltage
                Inches.of(3), // Wheel radius
                KilogramSquareMeters.of(0.03), // Steer MOI
                1.2)) // Wheel COF
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(27), Inches.of(27))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(29), Inches.of(28.5));

    public static final Pose2d STARTING_POSE = new Pose2d(3,3, new Rotation2d());


    }
    public static class OperatorConstants
    {
  
      // Joystick Deadband
      public static final double LEFT_X_DEADBAND  = 0.15;
      public static final double LEFT_Y_DEADBAND  = 0.15;
      public static final double RIGHT_X_DEADBAND = 0.1;
      public static final double TURN_CONSTANT    = 6;
    
    //DRIVER PRACTICE
    public static final boolean driverPractice = true; // advantage scope has peformance issues when running mutiple robots change rendering mode to standard or low
    }

   //For Easier camera setup to be used with already made vision examples  
   //Camera initiation can be found on line 376 of Vision swerve
  public static class PoseCameraConstants
  {
    public static double maxVisionStdDevsDistance = 30; //the distance at which std devs start to increase with distance

    public static final String CAM1N = "W"; //Cam 1 is roughly on top of the lower mount for elevator lokking in towards the reef tags.
   public static final Rotation3d CAM1R = new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(30));
   public static final Translation3d CAM1T = new Translation3d(Units.inchesToMeters(4),//transform of camera (dont forget forward+ left+ up+)
                      Units.inchesToMeters(-12),
                      Units.inchesToMeters(12));

    public static final String CAM2N = "RearOut"; //cam 2 and 3 are roughly halfway up the elevator bracing and mounted to the outside looking the direction of thier name. (this is important as looking directly (same plane) at a tag is bad)
   public static final Rotation3d CAM2R = new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(180+10)); 
   public static final Translation3d CAM2T = new Translation3d(Units.inchesToMeters(-4), //transform of camera (dont forget forward+ left+ up+)
                      Units.inchesToMeters(10.687),
                      Units.inchesToMeters(20.915));

  public static final String CAM3N = "FrontIn";
   public static final Rotation3d CAM3R = new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(20)); 
   public static final Translation3d CAM3T = new Translation3d(Units.inchesToMeters(4), //transform of camera (dont forget forward+ left+ up+)
                      Units.inchesToMeters(-10.687),
                      Units.inchesToMeters(20.915));

    //it goes up to 4 but it is commented out in SwerveVision
}
    public static class PathPlanner 
    {
        private static final double AUTO_CURRENT_LIMIT = 60; 
        private static final Translation2d FL = new Translation2d(12,12);
        private static final Translation2d FR = new Translation2d(12,-12);
        private static final Translation2d BL = new Translation2d(-12,12);
        private static final Translation2d BR = new Translation2d(-12,-12);
        private static final ModuleConfig modConfig = new ModuleConfig(2, Swerve.MAXSPEED, 1.15, DCMotor.getNEO(1), AUTO_CURRENT_LIMIT, 1);

        public static final RobotConfig config = new RobotConfig(Swerve.ROBOT_MASS, 1, modConfig, FL,FR,BL,BR);
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.7, 0, 0);
        public static final PIDConstants ANGLE_PID       = new PIDConstants(5.4, 0, 0.01);
        public static final PPHolonomicDriveController DRIVE_CONTROLLER = new PPHolonomicDriveController(TRANSLATION_PID,ANGLE_PID);
        // PPHolonomicController is the built in path following controller for holonomic drive trains

        public static final String DEFAULT = "TestAuto";

        public static Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
        public static PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    }

    public static final Mode currentMode = Mode.SIM; // TODO:IS MODE SET CURRECTLY??

    public static enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }
}