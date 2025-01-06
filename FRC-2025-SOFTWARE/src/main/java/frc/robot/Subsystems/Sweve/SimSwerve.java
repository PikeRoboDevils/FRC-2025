package frc.robot.Subsystems.Sweve;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SimSwerve implements SwerveIO {
    private double maxSpeed= Constants.Swerve.MAXSPEED;
    private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    

    public SimSwerve()
    {
        try {
            SwerveDrive  swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maxSpeed);
        } catch (IOException e) {}


    }

    @Override
public void updateInputs(SwerveIOInputs inputs) {}

    @Override
    public void driveChasisSpeeds(ChassisSpeeds velocity) {}
    
    @Override
    public void driveTeleop(double translationX, double translationY, double angularRotationX, boolean fieldRelative) {}

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {}

    @Override
    public void driveFieldOriented(ChassisSpeeds velocity) {}

    @Override
    public  void drivePathPlanner(ChassisSpeeds speeds, SwerveModuleState[] modules, Force[] forwardForce) {}

    @Override
    public void resetOdometry(Pose2d initialHolonomicPose) {}

    @Override
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {}

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void updateOdometry() {}

    @Override
    public void zeroGyro() {}

    @Override
    public void moduleLock() {}

    @Override
    public boolean isRedAlliance() {
        return false;
    }
}
