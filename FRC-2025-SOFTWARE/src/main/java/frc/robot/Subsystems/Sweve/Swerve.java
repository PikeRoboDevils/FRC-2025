// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Sweve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Sweve.VisionSwerve.Cameras;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;


import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;

public class Swerve extends SubsystemBase
{


  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  private VisionSwerve vision;

  private Pose2d[][] targetPosition = new Pose2d[23][3];

  private ProfiledPIDController translateX;
  private SimpleMotorFeedforward feedX;
  private ProfiledPIDController translateY;
  private SimpleMotorFeedforward feedY;
  private ProfiledPIDController rotateControl;

  private Rotation2d lastAngle;


      
  //not 2025 yet
   //private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  
    public Swerve(SwerveIO swerveIO)
    {
      this.io = swerveIO;
      Constants.Swerve.targetPosition[17][0] = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

      // Set Pose for tag ID and location (0 = alliance wall left 1 = alliance wall right) //this is
    // blue alliance
      targetPosition[17][0] =
      new Pose2d(
          new Translation2d(3.666, 3.003),
          new Rotation2d(
              Units.degreesToRadians(
                 55.5))); // should probably be named constants but Im in a time crunch (they
    // also need tuned)
      targetPosition[17][1] =
          new Pose2d(new Translation2d(3.94, 2.820), new Rotation2d(Units.degreesToRadians(55.5)));
      targetPosition[18][0] =
          new Pose2d(new Translation2d(3.183, 4.181), new Rotation2d(Units.degreesToRadians(0)));
      targetPosition[18][1] =
          new Pose2d(new Translation2d(3.183, 3.857), new Rotation2d(Units.degreesToRadians(0)));
      targetPosition[19][0] =
          new Pose2d(new Translation2d(4.0, 5.258), new Rotation2d(Units.degreesToRadians(-60)));
      targetPosition[19][1] =
          new Pose2d(new Translation2d(3.663, 5.086), new Rotation2d(Units.degreesToRadians(-60)));
      targetPosition[20][0] =
          new Pose2d(new Translation2d(5.011, 5.240), new Rotation2d(Units.degreesToRadians(-121)));
      targetPosition[20][1] =
          new Pose2d(new Translation2d(5.318, 5.079), new Rotation2d(Units.degreesToRadians(-121)));
      targetPosition[21][0] =
          new Pose2d(new Translation2d(5.803, 4.184), new Rotation2d(Units.degreesToRadians(180)));
      targetPosition[21][1] =
          new Pose2d(new Translation2d(5.803, 3.862), new Rotation2d(Units.degreesToRadians(180)));
      targetPosition[22][0] =
          new Pose2d(new Translation2d(5.301, 2.982), new Rotation2d(Units.degreesToRadians(121)));
      targetPosition[22][1] =
          new Pose2d(new Translation2d(5.011, 2.802), new Rotation2d(Units.degreesToRadians(121)));
  

      //Setup the closed loop controllers for auto align
      double TkP = 1;
      double TkI = 0;
      double TkD = 0.5;
      TrapezoidProfile.Constraints tConstraints =
          new TrapezoidProfile.Constraints(
              io.getMaxVelocity(),
              -io.getMaxVelocity()
                  / 2); // not making the accel limit negative causes it to do weird stuff with its
    // first move

      double fS = 0.03;
      double fV = 2;

      double RkP = 6;
      double RkI = 0;
      double RkD = 0;
      TrapezoidProfile.Constraints rConstraints =
          new TrapezoidProfile.Constraints(
              io.getMaxAnglularVelocity(), io.getMaxAnglularVelocity() / 1.5);

      translateX = new ProfiledPIDController(TkP, TkI, TkD, tConstraints, 0.02);
      feedX = new SimpleMotorFeedforward(fS, fV);
      translateY = new ProfiledPIDController(TkP, TkI, TkD, tConstraints, 0.02);
      feedY = new SimpleMotorFeedforward(fS, fV);
      rotateControl = new ProfiledPIDController(RkP, RkI, RkD, rConstraints, 0.02);
      rotateControl.enableContinuousInput(0, 360);


  
      if (Constants.Swerve.VISION || Robot.isSimulation())
      {
        setupPhotonVision();
        // Stop the odometry thread if we are using vision that way we can synchronize updates better.
        io.getSwerve().stopOdometryThread();
      }
  


    // Initialize the previous setpoint to the robot's current speeds & module states
    ChassisSpeeds currentSpeeds = io.getRobotVelocity(); // Method to get current robot-relative chassis speeds
    SwerveModuleState[] currentStates = io.getModuleState(); // Method to get the current swerve module states
    previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(Constants.PathPlanner.config.numModules));
      setupPathPlanner();


    setpointGenerator = new SwerveSetpointGenerator(
        Constants.PathPlanner.config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
        Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
    );

          }
        
         
     //TODO fix path planner again :(    
  private void setupPathPlanner() {
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> io.driveRobotRelative(speeds,feedforwards), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            Constants.PathPlanner.DRIVE_CONTROLLER,
            Constants.PathPlanner.config, // The robot configuration
            io::isRedAlliance,
            this // Reference to this subsystem to set requirements
    );
    //PathfindingCommand.warmupCommand().schedule();

    //Logging for the path planner values.
     PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        }); // loging stuff
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        }); // logging stuff
    }

// this crashes the robot idk why 
// // Since AutoBuilder is configured, we can use it to build pathfinding commands
// Command pathfindingCommand = AutoBuilder.pathfindToPose(
//         Constants.PathPlanner.targetPose,
//         Constants.PathPlanner.constraints,
//         0.0 // Goal end velocity in meters/sec
// );

  public void setupPhotonVision()
  {
    vision = new VisionSwerve(io::getPose, io.getField());
}

public void driveRobotRelative(ChassisSpeeds speeds) {
  // Note: it is important to not discretize speeds before or after
  // using the setpoint generator, as it will discretize them for you
  previousSetpoint = setpointGenerator.generateSetpoint(
      previousSetpoint, // The previous setpoint
      speeds, // The desired target speeds
      0.02 // The loop time of the robot code, in seconds
  );
  io.setModuleStates(previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
}

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);
      // Make the robot move    
      io.driveFieldOriented(io.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),headingX.getAsDouble(),headingY.getAsDouble()));
    });
  }
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      io.drive(new Translation2d(translationX.getAsDouble() * io.getMaxVelocity(),
                                          translationY.getAsDouble() * io.getMaxVelocity()),
                        angularRotationX.getAsDouble() * 2.5,
                        false);
    });
  }

  public Command fieldRelativeTeleop(DoubleSupplier LeftX, DoubleSupplier LeftY, DoubleSupplier RightX, DoubleSupplier steerSens) {
    return run(() -> {

    double Xvalue;
    double Yvalue;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
        Xvalue = -LeftX.getAsDouble();
        Yvalue = -LeftY.getAsDouble();
      } else {
        Xvalue = LeftX.getAsDouble();
        Yvalue = LeftY.getAsDouble();
      } 
    } else {
      Xvalue = LeftX.getAsDouble();
      Yvalue = LeftY.getAsDouble();
    }

    ChassisSpeeds desiredSpeeds = io.getTargetSpeeds(Yvalue, Xvalue, new Rotation2d(RightX.getAsDouble() * Math.PI));
    desiredSpeeds.omegaRadiansPerSecond = RightX.getAsDouble() * Math.PI * steerSens.getAsDouble();

    io.driveFieldOriented(desiredSpeeds);
    });
  }

  public Command fieldRelativeHybrid(DoubleSupplier LeftX, DoubleSupplier LeftY, DoubleSupplier RightX, DoubleSupplier steerSens) {
    return run(() -> {
    ChassisSpeeds desiredSpeeds = io.getTargetSpeeds(LeftY.getAsDouble(), LeftX.getAsDouble(), new Rotation2d(RightX.getAsDouble() * Math.PI));
    desiredSpeeds.omegaRadiansPerSecond = RightX.getAsDouble() * Math.PI * steerSens.getAsDouble();

    ChassisSpeeds holdingSpeeds;
    if (lastAngle != null) {
      double x = lastAngle.getSin();
      double y = lastAngle.getCos();
      holdingSpeeds = io.getTargetSpeeds(LeftY.getAsDouble(), LeftX.getAsDouble(), x,y);
    } else {
      holdingSpeeds = desiredSpeeds;
    }

    if (Math.abs(RightX.getAsDouble()) <= 0) {
      desiredSpeeds.omegaRadiansPerSecond = holdingSpeeds.omegaRadiansPerSecond;
    } else {
      lastAngle = getHeading();
    }

    io.driveFieldOriented(desiredSpeeds);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    io.drive(translation,
                      rotation,
                      fieldRelative);
    }

           //TODO: add to robot container

           //Aligns the robot to the requested reef pose automatically
  public Command autoAlign(int position){

    //identify the important face using the bes tag id coming from front camera
    int tagId = vision.getBestTagId(Cameras.CAM_1);

    //Identify pose
    Pose2d pose = targetPosition[tagId][position]; 
      
    //make sure its a valid tag
    if (pose == null) {
      return Commands.none();
    }

    //set the pid goals so they behave correctly
    translateX.setGoal(pose.getX());
    translateY.setGoal(pose.getY());
    rotateControl.setGoal(pose.getRotation().getDegrees());

    return run(
      () -> {
        //update PID values every loop 
        double x = translateX.calculate(getPose().getX());
        double y = translateY.calculate(getPose().getY());
        double rotate = rotateControl.calculate(getPose().getRotation().getDegrees());
        double xF = feedX.calculate(pose.getX() - getPose().getX());
        double yF = feedY.calculate(pose.getY() - getPose().getY());
        double xS = MathUtil.clamp(x + xF, -io.getMaxVelocity(), io.getMaxVelocity());
        double yS = MathUtil.clamp(y + yF, -io.getMaxVelocity(), io.getMaxVelocity());


        //parse field relative chasis speeds
        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, Units.degreesToRadians(rotate));

        //run chassis speeds
        io.driveFieldOriented(speeds);
      });
  } 

  @Override
  public void periodic()
  {
    Logger.processInputs("Swerve", inputs);
    Logger.recordOutput("Odometry/Pose", io.getPose());
    Logger.recordOutput("Odometry/driveToPose/translateXPID", translateX.getPositionError());
    Logger.recordOutput("Odometry/driveToPose/translateYPID", translateY.getPositionError());
    Logger.recordOutput("Odometry/driveToPose/rotatePID", rotateControl.getPositionError());

    if (Robot.isSimulation()) {
      if (io.getSimPose().isPresent()) { //might be able to merge the ifs but just to be safe against null errors it isnt
        Logger.recordOutput("Odometry/SimPose", io.getSimPose().get());
      }
    }

    // When vision is enabled we must manually update odometry in SwerveDrive
    if (Constants.Swerve.VISION || Robot.isSimulation())
    {
      io.updateOdometry();
      vision.visionSim.update(io.getPose());
      updatePoseWithVision();
      Logger.recordOutput("Odometry/Vision", vision.ReturnPhotonPose());
      Logger.recordOutput("bestTarget",vision.getBestTagId(Cameras.CAM_1));
      //vision
    }
  }
  @Override
  public void simulationPeriodic()
  {

  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    io.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() 
  {
    return !Constants.Swerve.VISION ? io.getPose() : getVisionPose();//inline if statement
    // return io.swerveDrive.getPose();
    // Made it simple, can still use getMesPose for the normal pose
  }
  public Pose2d getMesPose() 
  {
    return io.getPose();
    // use getPose() for the default pose
  }
  public void resetPose(Pose2d pose) 
  {
    io.resetOdometry(pose);
  }
  public ChassisSpeeds getRobotRelativeSpeeds() 
  {
    return io.getRobotVelocity();
  }
  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    io.setChassisSpeeds(chassisSpeeds);
  }


  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    io.zeroGyro();
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    io.zeroGyroWithAlliance();
  }



  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }


  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return io.getTargetSpeeds(scaledInputs.getX(),
                              scaledInputs.getY(),
                                                        headingX,
                                                        headingY);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d heading)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return io.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), heading);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return io.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return io.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return io.getSwerveDriveConfiguration();
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    io.lockPose();
  }

  /**
   * Update the pose estimation with vision data.
   */
  public void updatePoseWithVision()
  {
    vision.updatePoseEstimation(io.getSwerve());
  }

  /**
   * Get the pose while updating with vision readings.
   *
   * @return The robots pose with the vision estimates in place.
   */
  public Pose2d getVisionPose()
  {
    vision.updatePoseEstimation(io.getSwerve());
    return io.getPose();
  }


}
