// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class CoralIntakeHardware implements CoralIntakeIO {

  SparkMax CoralintakeMotor;
  SparkAbsoluteEncoder coralintakeencoder;
  RelativeEncoder internalEncoder;
  SparkClosedLoopController closedLoopController;

  SparkMaxConfig motorConfig;

  public CoralIntakeHardware() {
    CoralintakeMotor = new SparkMax(2, MotorType.kBrushless);

    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    closedLoopController = CoralintakeMotor.getClosedLoopController();
    internalEncoder = CoralintakeMotor.getEncoder();
    coralintakeencoder = CoralintakeMotor.getAbsoluteEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encooder.
     */
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
    // Set PID values for posotion control. We don't need to pass a closed
    // loop slot, as it will default to slot 0.
  }
}
