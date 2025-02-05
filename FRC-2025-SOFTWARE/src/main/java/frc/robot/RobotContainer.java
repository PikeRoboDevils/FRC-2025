// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Simulation;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Climber.ClimberHardware;
import frc.robot.Subsystems.CoralIntake.CoralIntake;
import frc.robot.Subsystems.CoralIntake.CoralIntakeHardware;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorHardware;
import frc.robot.Subsystems.Sweve.Swerve;
import frc.robot.Subsystems.Sweve.SwerveHardware;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristHardware;
import frc.robot.Subsystems.Wrist.WristSim;
import frc.robot.Subsystems.commands.AbsoluteDriveAdv;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0); //driver Controller

  private final Swerve drivebase = new Swerve(new SwerveHardware());

  private Wrist wrist;
  private Elevator elevator;
  private Climber climb;
  private CoralIntake intake;

  private final SendableChooser<Command> autoChooser;

  private final Field2d field;

  public RobotContainer() {
    if (Robot.isReal()) {
      wrist = new Wrist(new WristHardware());
      // elevator = new Elevator(new ElevatorHardware());
      climb = new Climber(new ClimberHardware());
      intake = new CoralIntake(new CoralIntakeHardware());
    } else {
      wrist = new Wrist(new WristSim());
      // elevator = new Elevator(new ElevatorSim());
      // climb = new Climber(new ClimberSim());
      // intake = new CoralIntake(new CoralIntakeSim());
    }
    
    configureBindings();
    field = new Field2d();

    SmartDashboard.putData("Pathplanner", field);

    // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            Logger.recordOutput("PathPlanner/CurrentPose", pose);
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Logger.recordOutput("PathPlanner/targetPose", pose);
            field.getObject("target pose").setPose(pose);
        });

    autoChooser = AutoBuilder.buildAutoChooser(Constants.PathPlanner.DEFAULT);
    SmartDashboard.putData("Auto Chooser", autoChooser);

//closedAbsoluteDriveAdv does not work at least in sim dont use it :)
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
        OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
        OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
        OperatorConstants.RIGHT_X_DEADBAND),
        driverXbox.getHID()::getYButtonPressed,
        driverXbox.getHID()::getAButtonPressed,
        driverXbox.getHID()::getXButtonPressed,
        driverXbox.getHID()::getBButtonPressed);

    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.fieldRelativeTeleop(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> 2.5);
     // im not sure where the inversions are supposed to be but right now 
    // it takes inverted controls and returns the correct speeds

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }


  private void configureBindings() {}

  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    drivebase.getDefaultCommand();
  }

  
}

