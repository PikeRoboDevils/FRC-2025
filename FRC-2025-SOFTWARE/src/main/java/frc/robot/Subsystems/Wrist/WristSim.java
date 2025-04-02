// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Utils.Constants;

/** Add your docs here. */
public class WristSim implements WristIO {

  private SingleJointedArmSim _wrist;
  private ArmFeedforward _feedforward;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private final PIDController positionController;

  public WristSim() {
    _wrist =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            50,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), 5),
            Units.inchesToMeters(8),
            Units.degreesToRadians(-90),
            Units.degreesToRadians(90),
            true,
            Units.degreesToRadians(90));

    _feedforward = new ArmFeedforward(0.0, 0.0, 1); // not being used
    profile = new TrapezoidProfile(new Constraints(10, 5)); // deg/s //not being used

    positionController = new PIDController(0.15, 0, 0); // from wrist hardware
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    _wrist.update(0.02);

    inputs.WristCurrent = _wrist.getCurrentDrawAmps();
    inputs.WristEncoderAngle = Units.radiansToDegrees(_wrist.getAngleRads());
    inputs.WristVolt = _wrist.getInput(0); // first index is voltage
    inputs.WristVelocity = Units.radiansPerSecondToRotationsPerMinute(_wrist.getVelocityRadPerSec());
        inputs.WristAtSetpoint = atSetpoint();
    if (DriverStation.isDisabled()) {
      resetController();
    }
  }

  @Override
  public void setAngle(double angleDeg) {

    goal = new TrapezoidProfile.State(angleDeg, 0.0);

    setpoint = profile.calculate(0.02, setpoint, goal);
    // setpoint = new TrapezoidProfile.State(0, 6);
    runPosition(setpoint);
  }


  private void runPosition(TrapezoidProfile.State setpoint) {
    double ff = _feedforward.calculate(setpoint.velocity, 0);
    double output = positionController.calculate(getAngleDeg(), setpoint.position);
    setVoltage(output + ff);
  }


  @Override
  public void setVoltage(double speed) {
    _wrist.setInputVoltage(MathUtil.clamp(speed, -12, 12));
  }

  @Override
  public double getAngleDeg() {
    return Units.radiansToDegrees(_wrist.getAngleRads());
  }

  @Override
  public double getAngleRad() {
    return _wrist.getAngleRads();
  }

  @Override
  public double getVoltage() {
    return _wrist.getInput(0); // This is voltage
  }

  public double getVelocity() {
    return (Units.radiansPerSecondToRotationsPerMinute(_wrist.getVelocityRadPerSec()) * 360) / 60;
  }

  private void resetController() {
    setpoint = new TrapezoidProfile.State(getAngleDeg(), 0.0);
  }

  
  @Override
  public boolean atSetpoint(){
    double offset = Math.abs(getAngleRad() - setpoint.position);
        return (offset > Constants.Encoders.Tolerance_Wrist);
  }
}
