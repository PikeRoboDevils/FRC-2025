// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.thethriftybot.ThriftyNova.EncoderType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class ElevatorHardware implements ElevatorIO {
  SparkMax Leader;
  SparkMax Follower;
  RelativeEncoder elevatorEncoder;
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

    // closedLoopController = Leader.getClosedLoopController();
    
    internalEncoder = Leader.getEncoder();
    DigitalInput dio = new DigitalInput(0);
    // elevatorEncoder = new Encoder(0, 0, false);
    elevatorEncoder = internalEncoder; //TEMPORARY

//     // Configures the encoder to return a distance of 4 for every 256 pulses
// // Also changes the units of getRate
// elevatorEncoder.setDistancePerPulse(2/2048);
// // Configures the encoder to consider itself stopped when its rate is below 10
// elevatorEncoder.setMinRate(10);
// // Reverses the direction of the encoder
// elevatorEncoder.setReverseDirection(true);
// // Configures an encoder to average its period measurement over 5 samples
// // Can be between 1 and 127 samples
// elevatorEncoder.setSamplesToAverage(5);

    // position control
    _feedforward = new ElevatorFeedforward(0,Constants.Encoders.kG_Elev,0); // based on random numbers in recalc
    positionController =
        new PIDController(
            Constants.Encoders.kP_Elev,
            Constants.Encoders.kI_Elev,
            Constants.Encoders.kD_Elev);
    profile = new TrapezoidProfile(new Constraints(3, 2)); //rotations a second



    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.voltageCompensation(
        12); // may be tweaked depending on voltage drain. Highly reccomended from a consistancy and
    // smoothness standpoint

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    // /*
    //  * Configure the closed loop controller. We want to make sure we set the
    //  * feedback sensor as the primary encoder.
    //  */
    // motorConfig
    //     .closedLoop
    //     .feedbackSensor()
    //     // Set PID values for position control. We don't need to pass a closed
    //     // loop slot, as it will default to slot 0.
    //     .p(0.4)
    //     .i(0)
    //     .d(0)
    //     .outputRange(-1, 1)
    //     // Set PID values for velocity control in slot 1
    //     .p(0.0001, ClosedLoopSlot.kSlot1)
    //     .i(0, ClosedLoopSlot.kSlot1)
    //     .d(0, ClosedLoopSlot.kSlot1)
    //     .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig
        .closedLoop
        .maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

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

    if(DriverStation.isDisabled()) {
      resetController();
    }
  }

  @Override
  public void setPosition(double position) {

    goal = new TrapezoidProfile.State(position, 0.0);

    setpoint = profile.calculate(0.02, setpoint, goal);
    // setpoint = new TrapezoidProfile.State(0, 6);
    runPosition(setpoint);
  }

  private void runPosition(TrapezoidProfile.State setpoint) {
    double ff = _feedforward.calculate(setpoint.velocity, 0);
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
  }

  @Override @Deprecated //TODO: un depreciate it
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
}
