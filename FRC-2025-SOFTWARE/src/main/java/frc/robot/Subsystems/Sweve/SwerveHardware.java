package frc.robot.Subsystems.Sweve;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveHardware implements SwerveIO {
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

  public SwerveHardware() {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    // double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER
    // RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get
    // meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    // double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4),
    // 6.75);
    // System.out.println("\"conversionFactors\": {");
    // System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    // System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
    // System.out.println("}");

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.MAXSPEED);

    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(
        false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(
        false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
    // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(
        true, true,
        0); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(
        true, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    swerveDrive
        .pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder
    // and push the offsets onto it. Throws warning if not possible
    resetOdometry(Constants.Swerve.STARTING_POSE);

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
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, headingX, headingY, getHeading().getRadians(), Constants.Swerve.MAXSPEED);
  }

  @Override
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d heading) {
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, heading.getRadians(), getHeading().getRadians(), Constants.Swerve.MAXSPEED);
  }

  @Override
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  @Override
  public Field2d getField() {
    return swerveDrive.field;
  }

  @Override
  public double getMaxVelocity() {
    return swerveDrive.getMaximumChassisVelocity();
  }

  @Override
  public SwerveController getController() {
    return swerveDrive.getSwerveController();
  }

  @Override
  public SwerveDrive getSwerve() {
    return swerveDrive;
  }

  @Override
  public void driveChasisSpeeds(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  @Override
  public SwerveModuleState[] getModuleState() {
    return swerveDrive.getStates();
  }

  @Override
  public void driveTeleop(
      double translationX, double translationY, double angularRotationX, boolean fieldRelative) {
    drive(new Translation2d(translationX, translationY), angularRotationX, fieldRelative);
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
  public void setModuleStates(SwerveModuleState[] modules) {
    swerveDrive.setModuleStates(modules, isRedAlliance());
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
  public void lockPose() {
    swerveDrive.lockPose();
  }

  @Override
  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  @Override
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  @Override
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  @Override
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  @Override
  public Optional<Pose2d> getSimPose() {
    return swerveDrive.getSimulationDriveTrainPose();
  }

  @Override
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  @Override
  public void drivePathPlanner(
      ChassisSpeeds speedsRobotRelative, DriveFeedforwards moduleFeedForwards) {
    swerveDrive.drive(
        speedsRobotRelative,
        // swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
        swerveDrive.kinematics.toWheelSpeeds(speedsRobotRelative),
        moduleFeedForwards.linearForces());
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {

    // left front
    inputs.LeftFrontModuleAngleCurrent = lfAngleMotor.getAppliedOutput();
    inputs.LeftFrontModuleAngleEncoderAngle = modules[0].getAbsolutePosition();
    inputs.LeftFrontModuleAngleInternalAngle = lfAngleMotor.getPosition();
    inputs.LeftFrontModuleAngleVelocity = lfAngleMotor.getVelocity();
    inputs.LeftFrontModuleAngleVolt = lfAngleMotor.getVoltage();

    inputs.LeftFrontModuleDriveCurrent = lfDriveMotor.getAppliedOutput();
    inputs.LeftFrontModuleDrivePosition = lfDriveMotor.getPosition();
    inputs.LeftFrontModuleDriveVelocity = lfDriveMotor.getVelocity();
    inputs.LeftFrontModuleDriveVolt = lfDriveMotor.getVoltage();

    // right front
    inputs.RightFrontModuleAngleCurrent = rfAngleMotor.getAppliedOutput();
    inputs.RightFrontModuleAngleEncoderAngle = modules[1].getAbsolutePosition();
    inputs.RightFrontModuleAngleInternalAngle = rfAngleMotor.getPosition();
    inputs.RightFrontModuleAngleVelocity = rfAngleMotor.getVelocity();
    inputs.RightFrontModuleAngleVolt = rfAngleMotor.getVoltage();

    inputs.RightFrontModuleDriveCurrent = rfDriveMotor.getAppliedOutput();
    inputs.RightFrontModuleDrivePosition = rfDriveMotor.getPosition();
    inputs.RightFrontModuleDriveVelocity = rfDriveMotor.getVelocity();
    inputs.RightFrontModuleDriveVolt = rfDriveMotor.getVoltage();

    // left rear
    inputs.LeftRearModuleAngleCurrent = lrAngleMotor.getAppliedOutput();
    inputs.LeftRearModuleAngleEncoderAngle = modules[2].getAbsolutePosition();
    inputs.LeftRearModuleAngleInternalAngle = lrAngleMotor.getPosition();
    inputs.LeftRearModuleAngleVelocity = lrAngleMotor.getVelocity();
    inputs.LeftRearModuleAngleVolt = lrAngleMotor.getVoltage();

    inputs.LeftRearModuleDriveCurrent = lrDriveMotor.getAppliedOutput();
    inputs.LeftRearModuleDrivePosition = lrDriveMotor.getPosition();
    inputs.LeftRearModuleDriveVelocity = lrDriveMotor.getVelocity();
    inputs.LeftRearModuleDriveVolt = lrDriveMotor.getVoltage();

    // right rear
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
