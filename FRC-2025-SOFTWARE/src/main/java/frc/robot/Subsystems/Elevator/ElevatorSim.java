// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorSim implements ElevatorIO {

  private edu.wpi.first.wpilibj.simulation.ElevatorSim _elevator;

  private ElevatorFeedforward _feedforward;
  private ProfiledPIDController _profiledPIDController;

  public ElevatorSim() {

    double[] stdDevs = new double[2];
    stdDevs[0] = 0.000002;
    stdDevs[1] = 0.000002;

    _elevator =
        new edu.wpi.first.wpilibj.simulation.ElevatorSim(
            DCMotor.getNEO(2),
            9,
            15,
            Units.inchesToMeters(2), // 2 inches
            Units.inchesToMeters(0),
            Units.inchesToMeters(74), // 84 inches //74 because of model rigging
            true,
            0,
            stdDevs);

    // position control
    _feedforward = new ElevatorFeedforward(0.05, 0.34, 6.91); // based on random numbers in recalc
    _profiledPIDController =
        new ProfiledPIDController(
            12,
            0,
            0.1,
            new Constraints(
                2.73,
                -2)); // for some reason if the accell isnt negative the first motion is slowed or
    // wonky
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    _elevator.update(0.02);

    inputs.ElevatorVelocity = getVelocity();
    inputs.ElevatorVolt = getVoltage();
    inputs.ElevatorCurrent = _elevator.getCurrentDrawAmps();
    inputs.ElevatorPosition = _elevator.getPositionMeters();
  }

  @Override
  public void setVoltage(double speed) {
    _elevator.setInputVoltage(speed);
  }

  @Override
  public void setPosition(double position) {
    double volts =
        MathUtil.clamp(
            _feedforward.calculate(position - getPosition())
                + _profiledPIDController.calculate(getPosition(), position),
            -12,
            12);
    _elevator.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double speed) {
    double volts =
        MathUtil.clamp(
            _profiledPIDController.calculate(speed, getVelocity()),
            0,
            0); // im lazy (also dont know if we need a set velocity)
    _elevator.setInputVoltage(volts);
  }

  @Override
  public double getVelocity() {
    return _elevator.getVelocityMetersPerSecond();
  }

  @Override
  public double getPosition() {
    return _elevator.getPositionMeters();
  }

  @Override
  public double getVoltage() {
    return _elevator.getInput(
        0); // I think this is the reference for voltage. Still needs checked, it would apear to be;
  }
}
