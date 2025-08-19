// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.Constants;
import frc.robot.Utils.LoggedCommandScheduler;
import frc.robot.Utils.Simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  public Robot() {
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("FRC-2025", "2025 Robot"); // Set a metadata value

    // Always log to network tables
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      if (Constants.currentMode == Constants.Mode.SIM) {
        // Obtains the default instance of the simulation world, which is a REEFSCAPE Arena.
        SimulatedArena.getInstance();
        // SimulatedArena.getInstance().placeGamePiecesOnField();
        SimulatedArena.getInstance().resetFieldForAuto(); // better

      } else {
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      }
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    // Thanks to 1683
    LoggedCommandScheduler.init(CommandScheduler.getInstance());

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    LoggedCommandScheduler.periodic();
  }

  
  // simulation period method in your Robot.java
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    // Get the positions of the notes (both on the field and in the air)
    Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    Pose3d[] algaePoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/CoralPositions", coralPoses);
    Logger.recordOutput("FieldSimulation/AlgeePositions", algaePoses);
    if (Constants.OperatorConstants.driverPractice) {
      Logger.recordOutput(
          "FieldSimulation/OpponentRobotPositions", Simulation.getOpponentRobotPoses());
      Logger.recordOutput(
          "FieldSimulation/AlliancePartnerRobotPositions",
          Simulation.getAlliancePartnerRobotPoses());
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    if (Robot.isSimulation()){
      SimulatedArena.getInstance().resetFieldForAuto();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationInit() {
    if (Constants.OperatorConstants.driverPractice) {
      Simulation.startOpponentRobotSimulations();
    }
  }

  @Override
  public void disabledInit() {
  }
}
