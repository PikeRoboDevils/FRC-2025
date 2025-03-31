// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utils.Constants;

/** Add your docs here. */
public class ElevatorSim implements ElevatorIO {

  private edu.wpi.first.wpilibj.simulation.ElevatorSim _elevator;

  private ElevatorFeedforward _feedforward;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private final PIDController positionController;

  public ElevatorSim() {

    double[] stdDevs = new double[2];
    stdDevs[0] = 0.000002;
    stdDevs[1] = 0.000002;

    _elevator =
        new edu.wpi.first.wpilibj.simulation.ElevatorSim(
            DCMotor.getNEO(2),
            6.6,
            10,
            Units.inchesToMeters(2), // 2 inches
            Units.inchesToMeters(0),
            Units.inchesToMeters(74), // 84 inches //74 because of model rigging
            true,
            0,
            stdDevs);

    // position control
    _feedforward =
        new ElevatorFeedforward(0, Constants.Encoders.kG_Elev, Constants.Encoders.kV_Elev);
    // profile = new TrapezoidProfile(new Constraints(15, 20)); // rotations a second
    profile = new TrapezoidProfile(new Constraints(Constants.Encoders.maxVelocityElevator,Constants.Encoders.maxAccelerationElevator)); // m/s
    positionController =
        new PIDController(
            Constants.Encoders.kP_Elev, Constants.Encoders.kI_Elev, Constants.Encoders.kD_Elev);
    // wonky
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    _elevator.update(0.02);

    inputs.ElevatorVelocity = getVelocity();
    inputs.ElevatorVolt = getVoltage();
    inputs.ElevatorCurrent = _elevator.getCurrentDrawAmps();
    inputs.ElevatorPosition = getPosition();

    if (DriverStation.isDisabled()) {
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

  @Override
  public void setVoltage(double speed) {
    _elevator.setInputVoltage(MathUtil.clamp(speed, -12, 12));
  }

  @Override
  public double getVelocity() {
    return _elevator.getVelocityMetersPerSecond();
  }

  @Override
  public double getPosition() {
    return _elevator.getPositionMeters();
        // * 13.5; // TODO: Remove "/13.5" once absolute encoder is added
  }

  @Override
  public double getVoltage() {
    return _elevator.getInput(0); // this is Voltage
  }

  private void resetController() {
    setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
  }
}
