package frc.robot.Utils.AutoDrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Sweve.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Set;

public class DriveToSource extends DriveTo {

  public boolean isPIDLoopRunning = false;

  private Swerve mSwerve;

  public static ArrayList<Pose2d> blueSourceTagPoses = new ArrayList<>();
  public static ArrayList<Pose2d> redSourceTagPoses = new ArrayList<>();
  public static ArrayList<Pose2d> allSourceTagPoses = new ArrayList<>();

  public DriveToSource(Swerve mSwerve, AprilTagFieldLayout field) {
    super(mSwerve, field); // for DriveTo parent
    this.mSwerve = mSwerve;

    Arrays.stream(new int[] {1, 2})
        .forEach(
            (i) -> {
              field
                  .getTagPose(i)
                  .ifPresent(
                      (p) -> {
                        blueSourceTagPoses.add(
                            new Pose2d(
                                p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
                      });
            });

    Arrays.stream(new int[] {12, 13})
        .forEach(
            (i) -> {
              field
                  .getTagPose(i)
                  .ifPresent(
                      (p) -> {
                        redSourceTagPoses.add(
                            new Pose2d(
                                p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
                      });
            });

    Arrays.stream((new int[] {1, 2, 12, 13}))
        .forEach(
            (i) -> {
              field
                  .getTagPose(i)
                  .ifPresent(
                      (p) -> {
                        allSourceTagPoses.add(
                            new Pose2d(
                                p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
                      });
            });
    /*
     * In the future we should use a util file just for posisions of april tags
     * i was lazy so i just looked at the field
     */

  }

  public Command generateCommand() {
    return Commands.defer(
        () -> {
          return getPathFromWaypoint(getWaypointSide(getClosestSide(mSwerve)));
        },
        Set.of());
  }

  /**
   * Where all the robot tranformations should be done
   *
   * @return Pathplanner waypoint with direction of travel away from the associated source side
   */
  private Pose2d getWaypointSide(Pose2d branch) {
    return new Pose2d(
        branch.getTranslation(), branch.getRotation().rotateBy(Rotation2d.k180deg)
        // getBranchRotation(mSwerve)
        );
  }

  // combine the two
  public static Pose2d getClosestSide(Swerve swerve) {
    Pose2d swervePose = swerve.getVisionPose();

    Pose2d tag = getClosestSide(swervePose);
    return tag;
  }

  public static Pose2d getClosestSide(Pose2d pose) {
    var alliance = DriverStation.getAlliance();

    ArrayList<Pose2d> SourcePoseList;
    if (alliance.isEmpty()) {
      SourcePoseList = allSourceTagPoses;
    } else {
      SourcePoseList = alliance.get() == Alliance.Red ? blueSourceTagPoses : redSourceTagPoses;
    }

    return pose.nearest(SourcePoseList);
  }
}
