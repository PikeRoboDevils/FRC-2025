// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utils.Constants;

/** Add your docs here. */
public class ElevatorHardware implements ElevatorIO {
  SparkMax Leader;
  SparkMax Follower;
  AbsoluteEncoder elevatorEncoder;
  RelativeEncoder internalEncoder;
  // SparkClosedLoopController closedLoopController;
  private ElevatorFeedforward _feedforward;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private final PIDController positionController;
  SparkMaxConfig motorConfig;

  public ElevatorHardware() {
    Leader = new SparkMax(12, MotorType.kBrushless);
    Follower = new SparkMax(14, MotorType.kBrushless);

    Follower.setControlFramePeriodMs(
        50); // defualt is 20 ms. The follower motor should be fine with slightly lower polling

    internalEncoder = Leader.getEncoder();
    elevatorEncoder = Leader.getAbsoluteEncoder();

    // position control
    _feedforward =
        new ElevatorFeedforward(
            Constants.Encoders.kS_Elev,
            Constants.Encoders.kG_Elev,
            Constants.Encoders.kV_Elev);
    positionController =
        new PIDController(
            Constants.Encoders.kP_Elev, 0, Constants.Encoders.kD_Elev);
    profile =
        new TrapezoidProfile(
            new Constraints(
                Constants.Encoders.maxVelocityElevator,
                Constants.Encoders.maxAccelerationElevator));

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.voltageCompensation(
        12); // may be tweaked depending on voltage drain. Highly reccomended from a consistancy and
    // smoothness standpoint

    motorConfig.smartCurrentLimit(40, 30);

  
    motorConfig.absoluteEncoder
    .positionConversionFactor(Constants.gearRatios.Elevator)
    .velocityConversionFactor(Constants.gearRatios.Elevator)
    .zeroOffset(Constants.Encoders.Offset_Elev)
    .inverted(Constants.Encoders.invert_Elev);

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
    Leader.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    Follower.configure(
        motorConfig.follow(Leader.getDeviceId()),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.ElevatorVelocity = getVelocity();
    inputs.ElevatorVolt = getVoltage();
    inputs.ElevatorCurrent = Leader.getOutputCurrent();
    inputs.ElevatorPosition = getPosition();
    inputs.ElevatorAtSetpoint = atSetpoint();

    if (DriverStation.isDisabled()) {
      resetController();
    }
  }

  @Override
  public void setPosition(double position) {
    
    //TEMP NEW SETPOINTS IN SIM BRANCH
    goal = new TrapezoidProfile.State(position/13.5, 0.0);

    setpoint = profile.calculate(0.02, setpoint, goal);
    runPosition(setpoint);
  }

  private void runPosition(TrapezoidProfile.State setpoint) {
    double ff = _feedforward.calculate(setpoint.velocity);
    double output = positionController.calculate(getPosition(), setpoint.position);
    setVoltage(output + ff);
  }

  // set voltage from 0-1
  @Override
  public void setVoltage(double speed) {
    Leader.setVoltage(MathUtil.clamp(speed, -12, 12));
  }

  private void resetController() {
    setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
    profile.calculate(getPosition(), setpoint, setpoint);
  }

  @Override
  @Deprecated // TODO: un depreciate it
  public void setVelocity(double speed) {
    // double volts =
    //     MathUtil.clamp(
    //         _profiledPIDController.calculate(speed, getVelocity()),
    //         0,
    //         0); // im lazy (also dont know if we need a set velocity)
    //     Leader.setVoltage(volts);
  }

  @Override
  public double getVelocity() {
    return elevatorEncoder.getVelocity();
  }

  @Override
  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  @Override
  public double getVoltage() {
    return Leader.getAppliedOutput();
  }

  @Override
  public boolean atSetpoint(){
    double offset = Math.abs(getPosition() - setpoint.position);
        return (offset > Constants.Encoders.Tolerance_Elev);
  }
}
