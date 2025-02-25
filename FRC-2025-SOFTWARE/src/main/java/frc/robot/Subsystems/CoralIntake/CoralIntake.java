// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  CoralIntakeIO io;

  CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  public CoralIntake(CoralIntakeIO coralIntakeIO) {
    this.io = coralIntakeIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Coral Intake", inputs);
  }
  public Command setVoltage(DoubleSupplier volts){
    return 
    run(()->io.setVoltage(volts.getAsDouble())).finallyDo(()->io.setVoltage(0));
  }
  public Command runIntake(DoubleSupplier speed){
    return 
    run(()->io.setVelocity(speed.getAsDouble()))
    .unless(()->io.hasCoral())
    .finallyDo(()->io.setVelocity(0));
  }

  public Command runIntakeAuto(){
    return 
    run(()->io.setVoltage(12))
    .until(()->io.hasCoral())
    .withTimeout(3);
  }

}
