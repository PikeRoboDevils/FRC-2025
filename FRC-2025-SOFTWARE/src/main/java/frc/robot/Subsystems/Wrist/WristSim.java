// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class WristSim implements WristIO {

  private SingleJointedArmSim _wrist;
  private ArmFeedforward _feedforward;
  private PIDController _pid;

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

    _feedforward = new ArmFeedforward(0.01, 0.3, 5.2);
    _pid = new PIDController(12, 0, 0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    _wrist.update(0.02);

    inputs.WristCurrent = _wrist.getCurrentDrawAmps();
    inputs.WristEncoderAngle = Units.radiansToDegrees(_wrist.getAngleRads());
    inputs.WristVolt =
        _wrist.getInput(
            0); // I think this is the reference for voltage. Still needs checked, it would apear to
    // be
    inputs.WristVelocity =
        Units.radiansPerSecondToRotationsPerMinute(_wrist.getVelocityRadPerSec());
  }

  @Override
  public void setAngle(double angleDeg) {
    double ffw =
        _feedforward.calculate(
            Units.degreesToRadians(angleDeg),
            _wrist.getAngleRads() - Units.degreesToRadians(angleDeg));
    double pid = _pid.calculate(_wrist.getAngleRads(), Units.degreesToRadians(angleDeg));

    _wrist.setInputVoltage(MathUtil.clamp(ffw + pid, -12, 12));
  }

  @Override
  public void setVoltage(double speed) {
    _wrist.setInputVoltage(speed * 12);
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
}
