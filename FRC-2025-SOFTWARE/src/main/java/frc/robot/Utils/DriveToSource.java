package frc.robot.Utils;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.Utils.Constants.PathPlanner.*;
import frc.robot.Subsystems.Sweve.*;

public class DriveToSource extends DriveTo{

    public boolean isPIDLoopRunning = false;

    private Swerve mSwerve;
                    
        public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
        public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
        public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();
                    
            public DriveToSource(Swerve mSwerve, AprilTagFieldLayout field) {
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
}