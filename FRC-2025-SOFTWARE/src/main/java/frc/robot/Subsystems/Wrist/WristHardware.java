// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;


/** Add your docs here. */
public class WristHardware implements WristIO {

    SparkMax wristMotor;
    SparkAbsoluteEncoder wristEncoder;
    RelativeEncoder internalEncoder;
    SparkClosedLoopController closedLoopController;

   SparkMaxConfig motorConfig;

    public WristHardware() {
        wristMotor = new SparkMax(0, MotorType.kBrushless);

      /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    closedLoopController = wristMotor.getClosedLoopController();
    internalEncoder = wristMotor.getEncoder();
    wristEncoder = wristMotor.getAbsoluteEncoder();

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
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig.closedLoop.maxMotion
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
    wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // // Initialize dashboard values
    // SmartDashboard.setDefaultNumber("Target Position", 0);
    // SmartDashboard.setDefaultNumber("Target Velocity", 0);
    // SmartDashboard.setDefaultBoolean("Control Mode", false);
    // SmartDashboard.setDefaultBoolean("Reset Encoder", false);


    }

    @Override
    public void updateInputs(WristIOInputs inputs) {

        inputs.WristCurrent = wristMotor.getOutputCurrent();
        inputs.WristEncoderAngle = getAngleDeg();
        inputs.WristVolt = getVoltage();
        inputs.WristVelocity = wristEncoder.getVelocity();
        inputs.WristInternalAngle = internalEncoder.getPosition();

    }

    // set wrist angle in degrees
    @Override
    public void setAngle(double angleDeg) {
        closedLoopController.setReference(angleDeg, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
    }

    // set voltage from 0-1
    @Override
    public void setVoltage(double speed) {
        wristMotor.set(speed);
    }

    @Override
    public double getAngleDeg() {
        return wristEncoder.getPosition();
    }

    @Override
    public double getAngleRad() {
        double radians = wristEncoder.getPosition() * (Math.PI/180);
        return radians;
    }

    @Override
    public double getVoltage() {
        return wristMotor.getAppliedOutput();
    }
}
