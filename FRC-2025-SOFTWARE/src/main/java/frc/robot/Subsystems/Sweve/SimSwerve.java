package frc.robot.Subsystems.Sweve;



//NOT taken from mason because i cant figure out how to make mine work😀
//ok mine didnt work cuz it was a bug and i thought I made a mistake but no its wasnt even my code...
//I DID THIS ALL ON MY OWN

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.io.IOException;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;


public class SimSwerve implements SwerveIO {
    private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        private final SelfControlledSwerveDriveSimulation simulatedDrive;
        SwerveDrive swerveDrive;
        SwerveDriveSimulation swerveDriveSimulation;
        SwerveModuleSimulation[] modules;
        SwerveModuleSimulation lf;
        SwerveModuleSimulation rf;
        SwerveModuleSimulation lr;
        SwerveModuleSimulation rr;
        
            public SimSwerve()
            {
        //for taget speeds
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.MAXSPEED);
        } catch (IOException e) {}
        /* Create a swerve drive simulation */
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
            // Specify Configuration
            Constants.Swerve.driveTrainSimulationConfig,
            // Specify starting pose
            new Pose2d(3, 3, new Rotation2d())
    )); 



// Register the drivetrain simulation to the default simulation world
SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

modules = simulatedDrive.getDriveTrainSimulation().getModules();
SwerveModuleSimulation lf = modules[0];
SwerveModuleSimulation rf = modules[1];
SwerveModuleSimulation lr = modules[2];
SwerveModuleSimulation br = modules[3];



    }


    @Override
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX,double headingY) {
        return new ChassisSpeeds(xInput*Constants.Swerve.MAXSPEED, yInput*Constants.Swerve.MAXSPEED, getHeading().interpolate(new Rotation2d(headingX, headingY),5).getRadians());
    }

    @Override
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d heading) {
        return new ChassisSpeeds(xInput*Constants.Swerve.MAXSPEED, yInput*Constants.Swerve.MAXSPEED, getHeading().rotateBy(heading).getRadians());
    }


    @Override
    public double getMaxVelocity() {
        return Constants.Swerve.MAXSPEED;
    }

    @Override
    public void driveChasisSpeeds(ChassisSpeeds velocity) {
        this.simulatedDrive.runChassisSpeeds(velocity, new Translation2d(), false,true);
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds velocity,DriveFeedforwards feedforwards) {
        simulatedDrive.runChassisSpeeds(velocity,new Translation2d(),false,true);
    }
    
    @Override
    public void driveTeleop(double translationX, double translationY, double angularRotationX, boolean fieldRelative) {
        drive(new Translation2d(translationX,translationY), angularRotationX, fieldRelative);
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        this.simulatedDrive.runChassisSpeeds(
            new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
            new Translation2d(),
            fieldRelative,
            true);
    }

    @Override
    public void driveFieldOriented(ChassisSpeeds velocity) {
        simulatedDrive.runChassisSpeeds(velocity,new Translation2d(),true, true);
    }

    @Override
    public void drivePathPlanner(ChassisSpeeds speeds, SwerveModuleState[] modules, Force[] forwardForce) {
        driveChasisSpeeds(speeds);
        simulatedDrive.runSwerveStates(modules);
    }

    @Override
    public void resetOdometry(Pose2d initialHolonomicPose) {
        simulatedDrive.resetOdometry(initialHolonomicPose);
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        simulatedDrive.getDriveTrainSimulation().setRobotSpeeds(chassisSpeeds);
    }
    @Override
    public ChassisSpeeds getFieldVelocity() {
        return simulatedDrive.getActualSpeedsFieldRelative();
    }


    // @Override
    // public void setBrakeMode(boolean enabled) {
        
    // }

    @Override
    public void updateOdometry() {
        simulatedDrive.periodic();
    }

    @Override
    public void zeroGyro() {
        simulatedDrive.getDriveTrainSimulation().getGyroSimulation().setRotation(simulatedDrive.getActualPoseInSimulationWorld().getRotation());
        resetOdometry(new Pose2d(simulatedDrive.getActualPoseInSimulationWorld().getTranslation(),new Rotation2d()));
    }

    @Override
    public void lockPose() {
        //idk
    }

    @Override
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }
    @Override
    public ChassisSpeeds getRobotVelocity() {
        return simulatedDrive.getActualSpeedsRobotRelative();
    }
    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }
    @Override
    public Pose2d getSimPose() {
        return simulatedDrive.getActualPoseInSimulationWorld();
    }
    @Override
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }
    @Override
    public double getMaxAnglularVelocity() {
        return Constants.Swerve.MAX_ANGULAR_VELOCITY;
    }
    @Override
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return null;
    }











@Override
public void updateInputs(SwerveIOInputs inputs) {
    
        //left front
        inputs.LeftFrontModuleAngleCurrent = lf.getSteerMotorSupplyCurrent().abs(Amp);
        inputs.LeftFrontModuleAngleEncoderAngle = lf.getSteerRelativeEncoderPosition().abs(Radians);
        inputs.LeftFrontModuleAngleInternalAngle = lf.getSteerRelativeEncoderPosition().abs(Radians);
        inputs.LeftFrontModuleAngleVelocity = lf.getSteerRelativeEncoderVelocity().abs(RadiansPerSecond);
        inputs.LeftFrontModuleAngleVolt = lf.getSteerMotorAppliedVoltage().abs(Volts);
        
        inputs.LeftFrontModuleDriveCurrent = lf.getDriveMotorSupplyCurrent().abs(Amp);
        inputs.LeftFrontModuleDrivePosition = lf.getDriveWheelFinalPosition().abs(Radians);
        inputs.LeftFrontModuleDriveVelocity = lf.getDriveWheelFinalSpeed().abs(RadiansPerSecond);
        inputs.LeftFrontModuleDriveVolt = lf.getDriveMotorAppliedVoltage().abs(Volts);


        //right front
        inputs.RightFrontModuleAngleCurrent = rf.getSteerMotorSupplyCurrent().abs(Amp);
        inputs.RightFrontModuleAngleEncoderAngle = rf.getSteerRelativeEncoderPosition().abs(Radians);
        inputs.RightFrontModuleAngleInternalAngle = rf.getSteerRelativeEncoderPosition().abs(Radians);
        inputs.RightFrontModuleAngleVelocity = rf.getSteerRelativeEncoderVelocity().abs(RadiansPerSecond);
        inputs.RightFrontModuleAngleVolt = rf.getSteerMotorAppliedVoltage().abs(Volts);
        
        inputs.RightFrontModuleDriveCurrent = rf.getDriveMotorSupplyCurrent().abs(Amp);
        inputs.RightFrontModuleDrivePosition = rf.getDriveWheelFinalPosition().abs(Radians);
        inputs.RightFrontModuleDriveVelocity = rf.getDriveWheelFinalSpeed().abs(RadiansPerSecond);
        inputs.RightFrontModuleDriveVolt = rf.getDriveMotorAppliedVoltage().abs(Volts);

        //left rear
        inputs.LeftRearModuleAngleCurrent = lr.getSteerMotorSupplyCurrent().abs(Amp);
        inputs.LeftRearModuleAngleEncoderAngle = lr.getSteerRelativeEncoderPosition().abs(Radians);
        inputs.LeftRearModuleAngleInternalAngle = lr.getSteerRelativeEncoderPosition().abs(Radians);
        inputs.LeftRearModuleAngleVelocity = lr.getSteerRelativeEncoderVelocity().abs(RadiansPerSecond);
        inputs.LeftRearModuleAngleVolt = lr.getSteerMotorAppliedVoltage().abs(Volts);
        
        inputs.LeftRearModuleDriveCurrent = lr.getDriveMotorSupplyCurrent().abs(Amp);
        inputs.LeftRearModuleDrivePosition = lr.getDriveWheelFinalPosition().abs(Radians);
        inputs.LeftRearModuleDriveVelocity = lr.getDriveWheelFinalSpeed().abs(RadiansPerSecond);
        inputs.LeftRearModuleDriveVolt = lr.getDriveMotorAppliedVoltage().abs(Volts);


        //right rear
        inputs.RightRearModuleAngleCurrent = rr.getSteerMotorSupplyCurrent().abs(Amp);
        inputs.RightRearModuleAngleEncoderAngle = rr.getSteerRelativeEncoderPosition().abs(Radians);
        inputs.RightRearModuleAngleInternalAngle = rr.getSteerRelativeEncoderPosition().abs(Radians);
        inputs.RightRearModuleAngleVelocity = rr.getSteerRelativeEncoderVelocity().abs(RadiansPerSecond);
        inputs.RightRearModuleAngleVolt = rr.getSteerMotorAppliedVoltage().abs(Volts);
        
        inputs.RightRearModuleDriveCurrent = rr.getDriveMotorSupplyCurrent().abs(Amp);
        inputs.RightRearModuleDrivePosition = rr.getDriveWheelFinalPosition().abs(Radians);
        inputs.RightRearModuleDriveVelocity = rr.getDriveWheelFinalSpeed().abs(RadiansPerSecond);
        inputs.RightRearModuleDriveVolt = rr.getDriveMotorAppliedVoltage().abs(Volts);
}
}

