// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.*;


/** Add your docs here. */
public class WristHardware implements WristIO {

  SparkMax wristMotor;
  AbsoluteEncoder wristEncoder;
  RelativeEncoder internalEncoder;
  // SparkClosedLoopController closedLoopController;
  private ArmFeedforward _feedforward;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private final PIDController positionController;

  SparkMaxConfig motorConfig;


  public WristHardware() {

    wristMotor = new SparkMax(16, MotorType.kBrushless);
    wristMotor.setControlFramePeriodMs(
        30); // defualt is 20 ms. This system should be fine with slightly lower polling

    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    internalEncoder = wristMotor.getEncoder();
    wristEncoder = wristMotor.getAbsoluteEncoder();


    _feedforward = new ArmFeedforward(Encoders.kS_Wrist, Encoders.kG_Wrist, Encoders.kV_Wrist);
    profile = new TrapezoidProfile(new Constraints(Encoders.maxVelocityWrist, Encoders.maxAccelerationWrist)); // deg/s

    // only control method being used
    positionController = new PIDController(Encoders.kP_Wrist, 0, Encoders.kD_Wrist);

    positionController.setTolerance(0.1);
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(40);
    motorConfig.idleMode(IdleMode.kCoast);
    
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig
        .absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360)
        .zeroOffset(Constants.Encoders.Offset_Wrist)
        .inverted(Constants.Encoders.invert_Wrist)
        .zeroCentered(true);
    // this is actually a "mechanisms" 1/gear (smaller than 1 reduction) ratio
    // would convert it to be in final rotations (I think. CTRE is gear ratio/1
    // [greater than 1 reduction])

    motorConfig.inverted(true);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    wristMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    inputs.WristCurrent = wristMotor.getOutputCurrent();
    inputs.WristEncoderAngle = getAngleDeg();

    inputs.WristVolt = getVoltage();
    inputs.WristVelocity = wristEncoder.getVelocity();
    inputs.WristInternalAngle = internalEncoder.getPosition();

    if (DriverStation.isDisabled()) {
      resetController();
    }
  }

  // set wrist angle in degrees
  @Override
  public void setAngle(double angleDeg) {

    goal = new TrapezoidProfile.State(angleDeg, 0.0);

    setpoint = profile.calculate(0.02, setpoint, goal);
    // setpoint = new TrapezoidProfile.State(0, 6);
    runPosition(setpoint);
  }

  private void runPosition(TrapezoidProfile.State setpoint) {
    double ff = _feedforward.calculate(setpoint.position,setpoint.velocity);
    double output = positionController.calculate(getAngleDeg(), setpoint.position);
    setVoltage(output + ff);
  }

  // set voltage from 0-1
  @Override
  public void setVoltage(double speed) {
    wristMotor.setVoltage(MathUtil.clamp(speed, -12, 12));
  }

  private void resetController() {
    setpoint = new TrapezoidProfile.State(getAngleDeg(), 0.0);
  }

  @Override
  public double getAngleDeg() {
    return wristEncoder.getPosition();
  }

  // @Override
  // public double getAngleRad() {
  //   return wristEncoder.getPosition() * (2 * Math.PI);
  // }

  @Override
  public double getVoltage() {
    return wristMotor.getAppliedOutput();
  }

  public double getVelocityDeg() {
    return (wristEncoder.getVelocity())
        / 60; // R/M * (deg/rotation) = Deg/M. Deg/M * M/Sec = Deg/Sec
  }
}
