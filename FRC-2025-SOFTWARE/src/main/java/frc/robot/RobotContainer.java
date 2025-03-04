// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Climber.Climber;import frc.robot.Subsystems.Climber.ClimberHardware;
import frc.robot.Subsystems.CoralIntake.CoralIntake;
import frc.robot.Subsystems.CoralIntake.CoralIntakeHardware;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorHardware;
import frc.robot.Subsystems.Elevator.ElevatorSim;
import frc.robot.Subsystems.Sweve.Swerve;
import frc.robot.Subsystems.Sweve.SwerveHardware;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristHardware;
import frc.robot.Subsystems.Wrist.WristSim;
import frc.robot.Subsystems.commands.AbsoluteDriveAdv;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0); // driver Controller
  final CommandXboxController operatorXbox = new CommandXboxController(1); // driver Controller

  private final Swerve drivebase = new Swerve(new SwerveHardware());

  public static Wrist wrist; // TODO: temp solution
  public static Elevator elevator; // TODO: temp solution
  private Climber climb;
  private CoralIntake intake;

  private final SendableChooser<Command> autoChooser;

  private final Field2d field;

  private boolean debounce = false;
  private int id = 0;

  public RobotContainer() {
    if (Robot.isReal()) {
      elevator = new Elevator(new ElevatorHardware());
      // wrist = new Wrist(new WristHardware(), elevator);
      climb = new Climber(new ClimberHardware());
      // intake = new CoralIntake(new CoralIntakeHardware());
    } else {
      elevator = new Elevator(new ElevatorSim());
      wrist =
          new Wrist(
              new WristSim(),
              elevator); // pass the current location of the wrist due to the stacked dof with
      // seperate subsystems
      // climb = new Climber(new ClimberSim());
      // intake = new CoralIntake(new CoralIntakeSim());
    }

    field = new Field2d();

    configureBindings();

    SmartDashboard.putData("Pathplanner", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          Logger.recordOutput("PathPlanner/CurrentPose", pose);
          field.setRobotPose(pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          Logger.recordOutput("PathPlanner/targetPose", pose);
          field.getObject("target pose").setPose(pose);
        });

    autoChooser =
        AutoBuilder.buildAutoChooser(
            Constants.PathPlanner.DEFAULT); // BE aware this does not remove old paths automatically
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private Command SetRobotState(Constants.RobotState robotState){
    return Commands.parallel(elevator.setPoint(()->robotState.ElevatorPos), Commands.waitSeconds(.5).andThen(()->wrist.setAngle(()->robotState.WristPos)));
  }
  private void configureBindings() {

    // closedAbsoluteDriveAdv does not work at least in sim dont use it :)
    AbsoluteDriveAdv closedAbsoluteDriveAdv =
        new AbsoluteDriveAdv(
            drivebase,
            () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () ->
                -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            driverXbox.getHID()::getYButtonPressed,
            driverXbox.getHID()::getAButtonPressed,
            driverXbox.getHID()::getXButtonPressed,
            driverXbox.getHID()::getBButtonPressed);

    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> -driverXbox.getRightX(),
            () -> -driverXbox.getRightY());

    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity =
        drivebase.fieldRelativeTeleop(
            () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND),
            () ->
                MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> 2.5);
    Command driveFieldOrientedHybrid =
        drivebase.fieldRelativeTeleop(
            () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND),
            () ->
                MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> 2.5);

    // im not sure where the inversions are supposed to be but right now
    // it takes inverted controls and returns the correct speeds

    // Drive Controller Commands

    // Generic
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); 
    driverXbox.b().whileTrue(Commands.runOnce(() -> drivebase.zeroGyro()));
    driverXbox.x().whileTrue(Commands.runOnce(() -> drivebase.lock()).repeatedly());
    driverXbox.x().whileFalse(Commands.run(()->drivebase.unlock()));

    // Season Specififc
    driverXbox.rightTrigger().whileTrue(intake.setVoltage(()-> MathUtil.applyDeadband(driverXbox.getRightTriggerAxis(), 0.1) * 0.1));
    driverXbox.leftTrigger().whileTrue(intake.setVoltage(()-> MathUtil.applyDeadband(-driverXbox.getLeftTriggerAxis(), 0.1) * 0.1));

    operatorXbox.rightTrigger(0.5).whileTrue(wrist.setVoltage(()-> MathUtil.applyDeadband(-operatorXbox.getRightY(), 0.1) * 0.1));
    operatorXbox.leftTrigger(0.5).whileTrue(climb.setAngle(()->operatorXbox.getRightY()));
    operatorXbox.leftStick().whileTrue(elevator.setVoltage(()-> -operatorXbox.getLeftY()));



    // Drive To pose commands. Might be worth rediong to be a single command
    if (Constants.Swerve.VISION) {
      driverXbox
          .leftBumper()
          .whileTrue(Commands.defer(() -> drivebase.autoAlign(0), Set.of(drivebase)));
      driverXbox
          .rightBumper()
          .whileTrue(Commands.defer(() -> drivebase.autoAlign(1), Set.of(drivebase)));
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    drivebase.getDefaultCommand();
  }
}
