/*
 * HEAVILY INSPIRED BY 4915
 * https://docs.google.com/document/d/10if4xjAaETTceUVn7l4J-jOCOnm5CJUDS5RAVNIJMQM/edit?tab=t.0
 * ACTUALLY PRETTY COOL SHOULD READ
 */
package frc.robot.Utils;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.reduxrobotics.canand.CanandDeviceDetails.Msg;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Utils.Constants.PathPlanner.*;
import frc.robot.Subsystems.Sweve.*;

public class AlignToReef extends DriveTo{

    public boolean isPIDLoopRunning = false;

    private Swerve mSwerve;
                    
        public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
        public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
        public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();
                    
            public AlignToReef(Swerve mSwerve, AprilTagFieldLayout field) {
                super(mSwerve, field);//for pathplanner 
                this.mSwerve = mSwerve;

            Arrays.stream(new int[]{6, 7, 8, 9, 10, 11}).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                blueReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(new int[]{17, 18, 19, 20, 21, 22}).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                redReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream((new int[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22})).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                allReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });        
        /* 
         * In the future we should use a util file just for posisions of april tags
         * i was lazy so i just looked at the field 
         */

        }
        
     //TODO: USE AKIT
    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("desired branch", Pose2d.struct).publish();

        
    /**
     * Side can be "L" or "R" and will go to that side of the april tag
     */
    public Command generateCommand(String side) {
        // var temp = mSwerve.getCurrentCommand();
        return Commands.defer(() -> {
            var branch = getClosestBranch(side, mSwerve);
                desiredBranchPublisher.accept(branch);
            
    return getPathFromWaypoint(getWaypointFromBranch(branch));
                }, Set.of());
                // .finallyDo(()->mSwerve.setDefaultCommand(temp));

            }

    /**
     * 
     * @return Pathplanner waypoint with direction of travel away from the associated reef side
     */
    private Pose2d getWaypointFromBranch(Pose2d branch){
        return new Pose2d(
            branch.getTranslation(),
            branch.getRotation().rotateBy(Rotation2d.k180deg)
            // getBranchRotation(mSwerve)
        );
    }

    /**
     * 
     * @return target rotation for the robot when it reaches the final waypoint
     */
    private Rotation2d getBranchRotation(Swerve swerve){
        return getClosestReefAprilTag(swerve.getPose()).getRotation().rotateBy(Rotation2d.k180deg);
    }


    public static Pose2d getClosestBranch(String branch, Swerve swerve){
        Pose2d swervePose = swerve.getVisionPose();
        
        Pose2d tag = getClosestReefAprilTag(swervePose);
        Translation2d branchOffset =
        (branch == "L")?
        new Translation2d(0,0):
        new Translation2d(0,0);
        return getBranchFromTag(tag, branchOffset);
    }


    private static Pose2d getBranchFromTag(Pose2d tag, Translation2d offset) {
        var translation = tag.getTranslation().plus(
            offset
            );
            //.rotateBy(tag.getRotation()); //broke it completely

        return new Pose2d(
            translation.getX(),
            translation.getY(),
            tag.getRotation()
        );
    }
    
    /**
     * get closest reef april tag pose to given position
     * 
     * @param pose field relative position
     * @return
     */
    public static Pose2d getClosestReefAprilTag(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        
        ArrayList<Pose2d> reefPoseList;
        if (alliance.isEmpty()) {
            reefPoseList = allReefTagPoses;
        } else{
            reefPoseList = alliance.get() == Alliance.Red ? 
                blueReefTagPoses :
                redReefTagPoses;
        }


        return pose.nearest(reefPoseList);

    }

}