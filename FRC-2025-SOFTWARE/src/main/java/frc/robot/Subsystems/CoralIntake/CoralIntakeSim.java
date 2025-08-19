// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Wrist.WristSim;

// TODO add a flywheel sim and set up MappleSim game piece simulation.
// https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/

/** Add your docs here. */
public class CoralIntakeSim implements CoralIntakeIO {

  private FlywheelSim flywheel;
  private LinearSystem<N1,N1,N1> system;
  private final IntakeSimulation intakeSim;

  private SwerveDriveSimulation drivebaseSim;
  public CoralIntakeSim(SwerveDriveSimulation drivebase) {

    drivebaseSim = drivebase;
    intakeSim = IntakeSimulation.OverTheBumperIntake(
    "Coral",
    drivebase,
    Inches.of(29),
    Inches.of(10),
    IntakeSide.FRONT,
    1);

    // gearing doesnt really matter just for intake sim 
    system = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 1, 1);

    flywheel = new FlywheelSim(system, DCMotor.getNEO(1));
  }

  /** Set voltage from 0-1 */
  @Override
  public void setVoltage(double speed) {
    flywheel.setInputVoltage(speed);

    //could be better
    Double elev = RobotContainer.ElevetorTop.getMeasureZ().abs(Meters);

    if (speed > 1 && elev > 0.5){
      intakeSim.startIntake();
    } 
    else {intakeSim.stopIntake();}

    if (speed <0){
      tryScore();
    }

  }

  @Override
  public void updateInputs(CoralIOInputsAutoLogged inputs) {
    flywheel.update(0.02);
    
    inputs.IntakeCurrent = flywheel.getCurrentDrawAmps();
    inputs.IntakeVolt = getVoltage();
    inputs.IntakeVelocity = getVelocity();
    inputs.hasCoral = hasCoral();

  }

  @Override
  public void setVelocity(double speed) {
    flywheel.setAngularVelocity(speed);
  }

  @Override
  public double getVelocity() {
    return flywheel.getAngularVelocityRPM();// more understandable
    // return flywheel.getAngularVelocityRadPerSec();
  }

  @Override
  public double getVoltage() {
    return flywheel.getInputVoltage();
  }

  @Override
  public boolean hasCoral() {
    // return flywheel.getCurrentDrawAmps() > 30? true:false;
    return intakeSim.getGamePiecesAmount()> 0 ? true:false;
  }

  public void tryScore() {
    //intake pose
    Pose3d intake = RobotContainer.WristPose;
    if (!hasCoral()) {return;}

    intakeSim.obtainGamePieceFromIntake();

    SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        drivebaseSim.getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism location at base of robot
        new Translation2d(intake.getMeasureX(),intake.getMeasureY()),
        // Obtain robot speed from drive simulation
        drivebaseSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        drivebaseSim.getSimulatedDriveTrainPose().getRotation(),
        // The height at which the coral is ejected
        intake.getMeasureZ(),
        // The initial speed of the coral
        MetersPerSecond.of(-1),
        // The coral angle
        intake.getRotation().getMeasureAngle()));

  }
  @Override
  public void addCoral() {
    intakeSim.addGamePieceToIntake();
  }
} 
