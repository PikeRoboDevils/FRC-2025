// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// TODO add a flywheel sim and set up MappleSim game piece simulation.
// https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/

/** Add your docs here. */
public class CoralIntakeSim {

  private FlywheelSim flywheel;

  public CoralIntakeSim() {
    flywheel = new FlywheelSim(null, DCMotor.getNEO(1), null);
  }

  /** Set voltage from 0-1 */
  public void setVoltage(double speed) {}

  public void updateInputs(CoralIOInputsAutoLogged inputs) {}

  public void setVelocity(double speed) {}

  public double getVelocity() {
    return 0;
  }

  public double getVoltage() {
    return 0;
  }

  public boolean hasCoral() {
    return false;
  }
}
