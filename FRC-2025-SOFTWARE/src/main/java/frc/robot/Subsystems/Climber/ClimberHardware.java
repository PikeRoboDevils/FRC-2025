// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Climber;

import java.lang.constant.ConstantDesc;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Utils.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class ClimberHardware implements ClimberIO {

  SparkMax ClimberMotor;
  SparkMax ClimberFollow;
  // SparkAbsoluteEncoder ClimberEncoder;
  RelativeEncoder internalEncoder;
  SparkClosedLoopController closedLoopController;

  SparkMaxConfig motorConfig;

  DigitalInput limitSwitch;

  public ClimberHardware() {
    ClimberMotor = new SparkMax(13, MotorType.kBrushless);
    ClimberFollow = new SparkMax(15, MotorType.kBrushless);

    limitSwitch = new DigitalInput(Constants.Encoders.ClimbChannel);
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    closedLoopController = ClimberMotor.getClosedLoopController();
    internalEncoder = ClimberMotor.getEncoder();
    // ClimberEncoder = ClimberMotor.getAbsoluteEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(60, 40);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
    .positionConversionFactor(Constants.gearRatios.Climber)
    .velocityConversionFactor(Constants.gearRatios.Climber);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1);


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

    motorConfig.idleMode(IdleMode.kBrake);

    ClimberMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    ClimberFollow.configure(
        motorConfig.follow(ClimberMotor, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // // Initialize dashboard values
    // SmartDashboard.setDefaultNumber("Target Position", 0);
    // SmartDashboard.setDefaultNumber("Target Velocity", 0);
    // SmartDashboard.setDefaultBoolean("Control Mode", false);
    // SmartDashboard.setDefaultBoolean("Reset Encoder", false);

  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.ClimberCurrent = ClimberMotor.getOutputCurrent();
    inputs.ClimberEncoderAngle = getAngleDeg();
    inputs.ClimberVolt = getVoltage();
    inputs.ClimberVelocity = internalEncoder.getVelocity();
    inputs.ClimberInternalAngle = internalEncoder.getPosition();
  }

  // set Climber angle in degrees
  @Override
  public void setAngle(double angleDeg) {

    // closedLoopController.setReference(angleDeg,ControlType.kPosition);
  }

  @Override
  public void setVoltage(double volts) {
    ClimberMotor.setVoltage(volts);
    ;
  }

  public void setEncoderPosition(Rotation2d rotations) {
    internalEncoder.setPosition(rotations.getRotations());
  }

  @Override
  public double getAngleDeg() {
    // if (limitSwitch.get()) {
    //   setEncoderPosition(new Rotation2d(Math.toRadians(35)));
    // }
    return (internalEncoder.getPosition() * 360);
  }

  @Override
  public double getAngleRad() {
    return internalEncoder.getPosition() * (2 * Math.PI);
  }

  @Override
  public double getVoltage() {
    return ClimberMotor.getAppliedOutput();
  }
}
