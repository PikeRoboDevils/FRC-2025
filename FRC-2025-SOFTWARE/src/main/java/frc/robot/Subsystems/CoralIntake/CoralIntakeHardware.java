// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Subsystems.Climber.ClimberIO.ClimberIOInputs;

/** Add your docs here. */
public class CoralIntakeHardware implements CoralIntakeIO {

  SparkMax intakeMotor;
  SparkAbsoluteEncoder intakeEncoder;
  RelativeEncoder internalEncoder;

  public CoralIntakeHardware() {
    intakeMotor = new SparkMax(2, MotorType.kBrushless);

    internalEncoder = intakeMotor.getEncoder();
    intakeEncoder = intakeMotor.getAbsoluteEncoder();

   
  }

  @Override
  public void updateInputs(CoralIOInputsAutoLogged inputs) {

    inputs.IntakeCurrent = intakeMotor.getOutputCurrent();
    inputs.IntakeVolt = getVoltage();
    inputs.IntakeVelocity = intakeEncoder.getVelocity();
 
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);;
  }
  @Override
  public double getVoltage() {
    return intakeMotor.getAppliedOutput();
  }
}
