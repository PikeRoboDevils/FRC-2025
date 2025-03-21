// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Climber.ClimberHardware;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.CoralIntake.CoralIntake;
import frc.robot.Subsystems.CoralIntake.CoralIntakeHardware;
import frc.robot.Subsystems.CoralIntake.CoralIntakeIO;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorHardware;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Sweve.Swerve;
import frc.robot.Subsystems.Sweve.SwerveHardware;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristHardware;
import frc.robot.Subsystems.Wrist.WristIO;
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
      wrist = new Wrist(new WristHardware(), elevator);
      climb = new Climber(new ClimberHardware());
      intake = new CoralIntake(new CoralIntakeHardware());
    } else {
      elevator = new Elevator(new ElevatorIO() {});
      wrist =
          new Wrist(
              new WristIO() {},
              elevator); // pass the current location of the wrist due to the stacked dof with
      // seperate subsystems
      climb = new Climber(new ClimberIO() {});
      intake = new CoralIntake(new CoralIntakeIO() {});
    }

    NamedCommands.registerCommand("CORALOUT", intake.setVoltage(() -> -3));

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

  private Command SetRobotState(Constants.RobotState robotState) {
    return Commands.parallel(
        elevator.setPoint(() -> robotState.ElevatorPos),
        Commands.waitSeconds(.5).andThen(() -> wrist.setAngle(() -> robotState.WristPos)));
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
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND),
            () ->
                MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> 4);
    Command driveFieldOrientedHybrid =
        drivebase.fieldRelativeTeleop(
            () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND),
            () ->
                MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> 2.5);

    Command stow = Commands.parallel(elevator.setPoint(() -> 0), wrist.setAngle(() -> 34));
    Command coralSource =
        Commands.parallel(
            elevator.setPoint(() -> 7.2 + operatorXbox.getLeftY() * 2), wrist.setAngle(() -> 30));
    Command coralL1 =
        Commands.parallel(
            elevator.setPoint(() -> 3 + operatorXbox.getLeftY() * 2), wrist.setAngle(() -> 30));
    Command coralL2 =
        Commands.parallel(
            elevator.setPoint(() -> 8.4 + operatorXbox.getLeftY() * 2), wrist.setAngle(() -> 0));
    Command algaeL2 =
            Commands.parallel(
                elevator.setPoint(() -> 8.4 + operatorXbox.getLeftY() * 2), wrist.setAngle(() -> 30));
    Command coralL3 =
        Commands.parallel(
            elevator.setPoint(() -> 14. + operatorXbox.getLeftY() * 2), wrist.setAngle(() -> 0));
    Command algaeL3 =
            Commands.parallel(
                elevator.setPoint(() -> 14. + operatorXbox.getLeftY() * 2), wrist.setAngle(() -> 0));
    Command coralL4 =
        Commands.parallel(
            elevator.setPoint(() -> 25.4 + operatorXbox.getLeftY() * 2), wrist.setAngle(() -> -20));

    // im not sure where the inversions are supposed to be but right now
    // it takes inverted controls and returns the correct speeds

    // Drive Controller Commands

    // Generic
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.b().whileTrue(Commands.runOnce(() -> drivebase.zeroGyro()));
    driverXbox.a().whileTrue(Commands.runOnce(() -> drivebase.lock()).repeatedly());
    driverXbox.a().whileFalse(Commands.run(() -> drivebase.unlock()));
    // driverXbox.x().whileTrue(Commands.run());
    
    // Season Specififc

    intake.setDefaultCommand(intake.setVoltage(() -> 1));
    driverXbox.rightTrigger().whileTrue(intake.setVoltage(() -> 2)); // In
    driverXbox.leftTrigger().whileTrue(intake.setVoltage(() -> -3)); // Out

    driverXbox.rightBumper().onTrue(Commands.runOnce(() -> drivebase.switchCamera(), drivebase));
    // driverXbox.leftBumper().toggleOnTrue(Commands.runOnce(()->drivebase.setDefaultCommand(closedAbsoluteDriveAdv), drivebase));
    // driverXbox.leftBumper().toggleOnFalse(Commands.runOnce(()->drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity), drivebase));

    // operatorXbox.leftTrigger().onTrue(drivebase.swictchCamera());

    // Overides

    operatorXbox.leftStick().whileTrue(elevator.setVoltage(() -> -operatorXbox.getLeftY() * 3));
    wrist.setDefaultCommand(wrist.setVoltage(() -> 0.02));
    operatorXbox.rightStick().whileTrue(wrist.setVoltage(() -> operatorXbox.getRightY() * 2));

    // Climber
    operatorXbox
        .rightTrigger(0.5)
        .whileTrue(climb.setVoltage(() -> 0.25 - operatorXbox.getRightY() * 4))
        .whileFalse(climb.setVoltage(() -> 0)); // POSITIVE IS DOWN

    // Elevator & Wrist
    operatorXbox.start().onTrue(Commands.runOnce(() -> elevator.reset(), elevator));
    operatorXbox.a().onTrue(stow);
    operatorXbox.povDown().onTrue(coralL1);
    operatorXbox.rightBumper().onTrue(coralSource);
    operatorXbox.b().onTrue(coralL2);
    operatorXbox.x().onTrue(coralL3);
    operatorXbox.y().onTrue(coralL4);

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
