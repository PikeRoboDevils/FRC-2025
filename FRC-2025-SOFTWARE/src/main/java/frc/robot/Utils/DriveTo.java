package frc.robot.Utils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Utils.Constants.PathPlanner.*;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Sweve.Swerve;

/**
 * A Util Class for making path on the fly for all auto movement with vision or object detection in the future
 */
public class DriveTo {

    public boolean isPIDLoopRunning = false;

    private final Swerve mSwerve;
            public DriveTo(Swerve mSwerve, AprilTagFieldLayout field) {
            this.mSwerve = mSwerve;

            }

/**
 * Actual Path On-the-fly Command Start from here
 * @param waypoint
 * @return
 */
public Command getPathFromWaypoint(Pose2d waypoint) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    new Pose2d(mSwerve.getVisionPose().getTranslation(), getPathVelocityHeading(mSwerve.getFieldVelocity(), waypoint)),waypoint);
        
        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
        return 
        Commands.sequence(
        Commands.print("start position PID loop"),
        PositionPIDCommand.generateCommand(mSwerve, waypoint, kAutoAlignAdjustTimeout,true),
        Commands.print("end position PID loop")
     );
                }
        
                PathPlannerPath path = new PathPlannerPath(
                    waypoints, 
                    DriverStation.isAutonomous() ? kAutoPathConstraints : kTeleopPathConstraints,
                    new IdealStartingState(getVelocityMagnitude(mSwerve.getFieldVelocity()), mSwerve.getHeading()), 
                    new GoalEndState(0.0, waypoint.getRotation())
                );
        
                path.preventFlipping = false;// check this
        
                return (AutoBuilder.followPath(path).andThen(
                    Commands.print("start position PID loop"),
                    PositionPIDCommand.generateCommand(mSwerve, waypoint, (
                        DriverStation.isAutonomous() ? kAutoAlignAdjustTimeout : kTeleopAlignAdjustTimeout
            ),DriverStation.isAutonomous() ? true : false)//for teleop at least
                .beforeStarting(Commands.runOnce(() -> {isPIDLoopRunning = true;}))
                .finallyDo(() -> {isPIDLoopRunning = false;})
        )).finallyDo((interupt) -> {
            if (interupt) { //if this is false then the position pid would've X braked & called the same method
                Commands.none();// does nothing so we can keep driving
            }
        });
    }


    public LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

        /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            System.out.println("approach: straight line");
            var diff = target.getTranslation().minus(mSwerve.getVisionPose().getTranslation());
            System.out.println("diff calc: \nx: " + diff.getX() + "\ny: " + diff.getY() + "\nDoT: " + diff.getAngle().getDegrees());
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }

        System.out.println("approach: compensating for velocity");

        var rotation = new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
        
        System.out.println("velocity calc: \nx: " + cs.vxMetersPerSecond + "\ny: " + cs.vyMetersPerSecond + "\nDoT: " + rotation);

        return rotation;
    }

        /**
     * Will go straight to the given pose
     * not much logic behind it
     */
    public Command generateCommand(Pose2d pose) {
        // var temp = mSwerve.getCurrentCommand();
        return Commands.defer(() -> {
            
    return getPathFromWaypoint(pose);
                }, Set.of());
                // .finallyDo(()->mSwerve.setDefaultCommand(temp));

            }

    }