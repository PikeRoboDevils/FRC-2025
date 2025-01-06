package frc.robot.Subsystems.Sweve;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class RealSwerve implements SwerveIO {
    private double maxSpeed= Constants.Swerve.MAXSPEED;
    private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    private SwerveDrive swerveDrive;
    SwerveModule[] modules;
    SwerveMotor lfAngleMotor;
    SwerveMotor lrAngleMotor;
    SwerveMotor rfAngleMotor;
    SwerveMotor rrAngleMotor;
    SwerveMotor lfDriveMotor;
    SwerveMotor rfDriveMotor;
    SwerveMotor lrDriveMotor;
    SwerveMotor rrDriveMotor;
    

    public RealSwerve()
    {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        try {
            SwerveDrive  swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maxSpeed);
        } catch (IOException e) {}

        
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible

        
        modules = swerveDrive.getModules();

        lfAngleMotor = modules[0].getAngleMotor();
        lfDriveMotor = modules[0].getDriveMotor();

        rfAngleMotor = modules[1].getAngleMotor();
        rfDriveMotor = modules[1].getDriveMotor();

        lrAngleMotor = modules[2].getAngleMotor();
        lrDriveMotor = modules[2].getDriveMotor();

        rrAngleMotor = modules[3].getAngleMotor();
        rrDriveMotor = modules[3].getDriveMotor();


    }


    @Override
    public void driveChasisSpeeds(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }
    
    @Override
    public void driveTeleop(double translationX, double translationY, double angularRotationX, boolean fieldRelative) {
        drive(new Translation2d(translationX,translationY), angularRotationX, fieldRelative);
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    @Override
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    @Override
    public void drivePathPlanner(ChassisSpeeds speeds, SwerveModuleState[] modules, Force[] forwardForce) {
        swerveDrive.drive(speeds, modules, forwardForce);
    }

    @Override
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        swerveDrive.setMotorIdleMode(enabled);
    }

    @Override
    public void updateOdometry() {
        swerveDrive.updateOdometry();
    }

    @Override
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    @Override
    public void moduleLock() {
        swerveDrive.lockPose();
    }

    @Override
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

@Override
public void updateInputs(SwerveIOInputs inputs) {
    
        //left front
        inputs.LeftFrontModuleAngleCurrent = lfAngleMotor.getAppliedOutput();
        inputs.LeftFrontModuleAngleEncoderAngle = modules[0].getAbsolutePosition();
        inputs.LeftFrontModuleAngleInternalAngle = lfAngleMotor.getPosition();
        inputs.LeftFrontModuleAngleVelocity = lfAngleMotor.getVelocity();
        inputs.LeftFrontModuleAngleVolt = lfAngleMotor.getVoltage();
        
        inputs.LeftFrontModuleDriveCurrent = lfDriveMotor.getAppliedOutput();
        inputs.LeftFrontModuleDrivePosition = lfDriveMotor.getPosition();
        inputs.LeftFrontModuleDriveVelocity = lfDriveMotor.getVelocity();
        inputs.LeftFrontModuleDriveVolt = lfDriveMotor.getVoltage();


        //right front
        inputs.RightFrontModuleAngleCurrent = rfAngleMotor.getAppliedOutput();
        inputs.RightFrontModuleAngleEncoderAngle = modules[1].getAbsolutePosition();
        inputs.RightFrontModuleAngleInternalAngle = rfAngleMotor.getPosition();
        inputs.RightFrontModuleAngleVelocity = rfAngleMotor.getVelocity();
        inputs.RightFrontModuleAngleVolt = rfAngleMotor.getVoltage();
        
        inputs.RightFrontModuleDriveCurrent = rfDriveMotor.getAppliedOutput();
        inputs.RightFrontModuleDrivePosition = rfDriveMotor.getPosition();
        inputs.RightFrontModuleDriveVelocity = rfDriveMotor.getVelocity();
        inputs.RightFrontModuleDriveVolt = rfDriveMotor.getVoltage();

        //left rear
        inputs.LeftRearModuleAngleCurrent = lrAngleMotor.getAppliedOutput();
        inputs.LeftRearModuleAngleEncoderAngle = modules[2].getAbsolutePosition();
        inputs.LeftRearModuleAngleInternalAngle = lrAngleMotor.getPosition();
        inputs.LeftRearModuleAngleVelocity = lrAngleMotor.getVelocity();
        inputs.LeftRearModuleAngleVolt = lrAngleMotor.getVoltage();
        
        inputs.LeftRearModuleDriveCurrent = lrDriveMotor.getAppliedOutput();
        inputs.LeftRearModuleDrivePosition = lrDriveMotor.getPosition();
        inputs.LeftRearModuleDriveVelocity = lrDriveMotor.getVelocity();
        inputs.LeftRearModuleDriveVolt = lrDriveMotor.getVoltage();


        //right rear
        inputs.RightRearModuleAngleCurrent = rrAngleMotor.getAppliedOutput();
        inputs.RightRearModuleAngleEncoderAngle = modules[3].getAbsolutePosition();
        inputs.RightRearModuleAngleInternalAngle = rrAngleMotor.getPosition();
        inputs.RightRearModuleAngleVelocity = rrAngleMotor.getVelocity();
        inputs.RightRearModuleAngleVolt = rrAngleMotor.getVoltage();
        
        inputs.RightRearModuleDriveCurrent = rrDriveMotor.getAppliedOutput();
        inputs.RightRearModuleDrivePosition = rrDriveMotor.getPosition();
        inputs.RightRearModuleDriveVelocity = rrDriveMotor.getVelocity();
        inputs.RightRearModuleDriveVolt = rrDriveMotor.getVoltage();
}
}

