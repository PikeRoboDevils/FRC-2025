// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.BangBangController;

/** Add your docs here. */
public class CoralIntakeHardware implements CoralIntakeIO {

  SparkMax intakeMotor;
  SparkAbsoluteEncoder intakeEncoder;
  RelativeEncoder internalEncoder;
  BangBangController mBangController;// simple no tuning 
  
  SparkMaxConfig motorConfig;

  public CoralIntakeHardware() {
    intakeMotor = new SparkMax(2, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    internalEncoder = intakeMotor.getEncoder();
    intakeEncoder = intakeMotor.getAbsoluteEncoder();

    mBangController.setTolerance(10);
    
    motorConfig.idleMode(IdleMode.kCoast);
    motorConfig.smartCurrentLimit(30, 20);
    
  }

  @Override
  public void updateInputs(CoralIOInputsAutoLogged inputs) {

    inputs.IntakeCurrent = intakeMotor.getOutputCurrent();
    inputs.IntakeVolt = getVoltage();
    inputs.IntakeVelocity = intakeEncoder.getVelocity();
    inputs.hasCoral = hasCoral();
  }

  @Override
  public void setVelocity(double velocity ) {
    if (mBangController.atSetpoint()) {
      intakeMotor.set(0);
      return;
    };

    double output = mBangController.calculate(internalEncoder.getVelocity(), velocity);
    intakeMotor.set(output);

  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public double getVoltage() {
    return intakeMotor.getAppliedOutput();
  }

  public boolean hasCoral() {
     return intakeMotor.getOutputCurrent() > 20 ;
  }
}
