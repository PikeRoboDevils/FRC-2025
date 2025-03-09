// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import com.revrobotics.AbsoluteEncoder;
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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.Constants;
import frc.robot.Constants.gearRatios;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class WristHardware implements WristIO {

  SparkMax wristMotor;
  RelativeEncoder wristEncoder;
  RelativeEncoder internalEncoder;
  // SparkClosedLoopController closedLoopController;
  private ArmFeedforward _feedforward;
  private ProfiledPIDController _profiledPIDController;

  SparkMaxConfig motorConfig;

  public WristHardware() {
    wristMotor = new SparkMax(16, MotorType.kBrushless);
    wristMotor.setControlFramePeriodMs(
        30); // defualt is 20 ms. This system should be fine with slightly lower polling

    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    // closedLoopController = wristMotor.getClosedLoopController();
    internalEncoder = wristMotor.getEncoder();
    // wristEncoder = wristMotor.getAbsoluteEncoder();
    wristEncoder = internalEncoder;

        // position control
    _feedforward = new ArmFeedforward(0,0,0); // based on random numbers in recalc
    _profiledPIDController =
        new ProfiledPIDController(
            Constants.Encoders.kP_Wrist,
            Constants.Encoders.kI_Wrist,
            Constants.Encoders.kD_Wrist,
            new Constraints(
                1,
                2)); // for some reason if the accell isnt negative the first motion is slowed or wonky

    
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    motorConfig.voltageCompensation(
        12); // may be tweaked depending on voltage drain. Highly reccomended from a consistancy and
    // smoothness standpoint

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder.positionConversionFactor(gearRatios.Arm).velocityConversionFactor(gearRatios.Arm); //this is actually a "mechanisms" 1/gear (smaller than 1 reduction) ratio would convert it to be in final rotations (I think. CTRE is gear ratio/1 [greater than 1 reduction])

    // /*
    //  * Configure the closed loop controller. We want to make sure we set the
    //  * feedback sensor as the primary encoder.
    //  */
    // motorConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
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
    wristMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // // Initialize dashboard values
    // SmartDashboard.setDefaultNumber("Target Position", 0);
    // SmartDashboard.setDefaultNumber("Target Velocity", 0);
    // SmartDashboard.setDefaultBoolean("Control Mode", false);
    // SmartDashboard.setDefaultBoolean("Reset Encoder", false);

    _profiledPIDController.reset(getAngleDeg(), getVelocityDeg()); //consistant unit + in seconds
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    inputs.WristCurrent = wristMotor.getOutputCurrent();
    inputs.WristEncoderAngle = getAngleDeg();
    inputs.WristVolt = getVoltage();
    // inputs.WristVelocity = wristEncoder.get();
    inputs.WristInternalAngle = internalEncoder.getPosition();
  }

  // set wrist angle in degrees
  @Override
  public void setAngle(double angleDeg) {
    double volts =
        MathUtil.clamp(
            _feedforward.calculate(angleDeg, 0)
                + _profiledPIDController.calculate(getAngleDeg(), angleDeg),
            -12,
            12);
    wristMotor.setVoltage(volts);
  }

  // set voltage from 0-1
  @Override
  public void setVoltage(double speed) {
    wristMotor.setVoltage(speed);
  }

  @Override
  public double getAngleDeg() {
    return wristEncoder.getPosition() * (360); //getPosition returns rotations now
  }

  @Override
  public double getAngleRad() {
    double radians = wristEncoder.getPosition() * (2 * Math.PI); //getPosition returns rotations now
    return radians;
  }

  //TODO: make overide and set value using limit switch to about +45 deg (dont forget Units.degreeToRadian()).
  public void setEncoderPosition(Rotation2d rotations) {
    wristEncoder.setPosition(rotations.getRotations());
  }

  @Override
  public double getVoltage() {
    return wristMotor.getAppliedOutput();
  }

  public double getVelocityDeg() {
    return (wristEncoder.getVelocity() * 360) /60; // R/M * (deg/rotation) = Deg/M. Deg/M * M/Sec = Deg/Sec 
  }
}
