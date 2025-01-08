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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;


import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;


public class Swerve extends SubsystemBase
{


  private final SwerveIO io;

  private VisionSwerve vision;
      
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  
    public Swerve(SwerveIO swerveIO)
    {
      this.io = swerveIO;
  
  
      if (Constants.Swerve.VISION)
      {
        setupPhotonVision();
        // Stop the odometry thread if we are using vision that way we can synchronize updates better.
        io.getSwerve().stopOdometryThread();
      }
  
      setupPathPlanner();
          }
        
         
         
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
    PathfindingCommand.warmupCommand().schedule();
    }

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
                        angularRotationX.getAsDouble() * io.getMaxAnglularVelocity(),
                        true);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    io.drive(translation,
                      rotation,
                      fieldRelative);
    }


  @Override
  public void periodic()
  {
    Logger.recordOutput("Odometry/Pose", io.getPose());
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (Constants.Swerve.VISION)
    {
      io.updateOdometry();
      
      //vision
    }
  }
  @Override
  public void simulationPeriodic()
  {
    Logger.recordOutput("Odometry/SimPose", io.getSimPose());

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
