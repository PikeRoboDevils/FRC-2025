package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

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
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import swervelib.math.Matter;

public class Constants {

  public enum RobotState {
    L1Coral(0, 0),
    L2Coral(0, 0),
    L3Coral(0, 0),
    L4Coral(25.5, 0),
    Source(0, 0),
    L2Algae(0, 0),
    L3Algae(0, 0);

    public double WristPos;
    public double ElevatorPos;

    RobotState(double elevatorPos, double wristPos) {
      WristPos = wristPos;
      ElevatorPos = elevatorPos;
    }
  };

  public class gearRatios {

    public static double Arm = (11.0 / 42.0) * (1.0 / 5.0); // inverted
  }

  public class Swerve {

    public static Pose2d[][] targetPosition = new Pose2d[23][3];

    public static boolean VISION = true;

    public static final double MAXSPEED =
        Units.feetToMeters(15.1); // Why did we lower the drive speed int teleop?
    // taken from offseason swerve
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    // Create and configure a drivetrain simulation configuration
    public static final DriveTrainSimulationConfig driveTrainSimulationConfig =
        DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2())
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DCMotor.getNEO(1), // Drive motor is a NEO
                    DCMotor.getNEO(1), // Steer motor is a NEO
                    6.75, // Drive motor gear ratio. SDS L2 GEAR RATIO
                    150 / 7, // Steer motor gear ratio. SDS L2 GEAR RATIO
                    Volts.of(0.1), // Drive friction voltage.
                    Volts.of(0.1), // Steer friction voltage
                    Inches.of(3), // Wheel radius
                    KilogramSquareMeters.of(0.03), // Steer MOI
                    1.2)) // Wheel COF
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(Inches.of(27), Inches.of(27))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(Inches.of(29), Inches.of(28.5));

    public static final Pose2d STARTING_POSE =
        new Pose2d(8, 6, new Rotation2d(Math.toRadians(180)));
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.15;
    public static final double LEFT_Y_DEADBAND = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    // DRIVER PRACTICE
    public static final boolean driverPractice =
        false; // advantage scope has peformance issues when running mutiple robots change rendering
    // mode to standard or low
  }

  // For Easier camera setup to be used with already made vision examples
  // Camera initiation can be found on line 376 of Vision swerve
  public static class PoseCameraConstants {
    public static Object[] CAM1 = new Object[3];

    {
      // Cam 1 is roughly on top of the lower mount for elevator lokking in towards the reef tags.
      CAM1[0] = "CAM1";

      CAM1[1] = new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(30));

      CAM1[2] =
          new Translation3d(
              Units.inchesToMeters(4), // transform of camera (dont forget forward+ left+ up+)
              Units.inchesToMeters(-12),
              Units.inchesToMeters(12));
    }

    // cam 2 and 3 are roughly halfway up the elevator bracing and mounted to the
    // outside looking the direction of thier name. (this is important as looking
    // directly (same plane) at a tag is bad)
    public static Object[] CAM2 = new Object[3];

    {
      CAM2[0] = "RearOut";

      CAM2[1] = new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(180 + 10));

      CAM2[2] =
          new Translation3d(
              Units.inchesToMeters(-4), // transform of camera (dont forget forward+ left+ up+)
              Units.inchesToMeters(10.687),
              Units.inchesToMeters(20.915));
    }

    public static Object[] CAM3 = new Object[3];

    {
      CAM3[0] = "FrontIn";

      CAM3[1] = new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(20));

      CAM3[2] =
          new Translation3d(
              Units.inchesToMeters(4), // transform of camera (dont forget forward+ left+ up+)
              Units.inchesToMeters(-10.687),
              Units.inchesToMeters(20.915));
    }

    // it goes up to 4 but it is commented out in SwerveVision
    public static final double maxVisionStdDevsDistance = 30;

    // it goes up to 4 but it is commented out in SwerveVision
  }

  public static class PathPlanner {
    private static final double AUTO_CURRENT_LIMIT = 40;
    private static final Translation2d FL = new Translation2d(12, 12);
    private static final Translation2d FR = new Translation2d(12, -12);
    private static final Translation2d BL = new Translation2d(-12, 12);
    private static final Translation2d BR = new Translation2d(-12, -12);
    private static final ModuleConfig modConfig =
        new ModuleConfig(3, Swerve.MAXSPEED, 1, DCMotor.getNEO(1), AUTO_CURRENT_LIMIT, 1);

    public static final RobotConfig config =
        new RobotConfig(Swerve.ROBOT_MASS, 1, modConfig, FL, FR, BL, BR);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(1.4, 0, 0.01);
    public static final PPHolonomicDriveController DRIVE_CONTROLLER =
        new PPHolonomicDriveController(TRANSLATION_PID, ANGLE_PID);
    // PPHolonomicController is the built in path following controller for holonomic drive trains

    public static final String DEFAULT = "";

    public static Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
    public static PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
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

  public static class Encoders {

    public static final int WristChannel = 8;
    // public static final int WristChannelB = 0;
    public static final boolean WristReverse = false;

    public static final double kP_Wrist = 0.001;
    public static final double kI_Wrist = 0;
    public static final double kD_Wrist = 0;

    public static final int ElevatorChannelA = 2;
    public static final int ElevatorChannelB = 3;
    public static final boolean ElevatorReverse = false;

    public static final double kP_Elev = 7;
    public static final double kI_Elev = 0;
    public static final double kD_Elev = 0.0007;

    public static final double kG_Elev = 0.090;
    public static final double kV_Elev = 0.2;
    ;
  }
}
