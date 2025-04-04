// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Climber.ClimberHardware;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberSim;
import frc.robot.Subsystems.CoralIntake.CoralIntake;
import frc.robot.Subsystems.CoralIntake.CoralIntakeHardware;
import frc.robot.Subsystems.CoralIntake.CoralIntakeIO;
import frc.robot.Subsystems.CoralIntake.CoralIntakeSim;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorHardware;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorSim;
import frc.robot.Subsystems.Sweve.Swerve;
import frc.robot.Subsystems.Sweve.SwerveHardware;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristHardware;
import frc.robot.Subsystems.Wrist.WristIO;
import frc.robot.Subsystems.Wrist.WristSim;
import frc.robot.Utils.AlignToReef;
import frc.robot.Utils.DriveToSource;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.OperatorConstants;
import frc.robot.Utils.commands.AbsoluteDriveAdv;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0); // driver Controller
  final CommandXboxController operatorXbox = new CommandXboxController(1); // driver Controller

  private final Swerve drivebase = new Swerve(new SwerveHardware());

  private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
//   private DriveTo otfPathFactory = new DriveTo(drivebase,tagLayout); //do not use will get confusing
  private AlignToReef reefAlignmentFactory = new AlignToReef(drivebase, tagLayout); // we love 4915
  private DriveToSource sourcePathFactory = new DriveToSource(drivebase,tagLayout);

  public static Wrist wrist; // TODO: temp solution
  public static Elevator elevator; // TODO: temp solution
  private Climber climb;
  private CoralIntake intake;

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  private final Field2d field;
  
  public RobotContainer() {
    if (Robot.isReal()) {   

      elevator = new Elevator(new ElevatorHardware());
      wrist = new Wrist(new WristHardware(), elevator);
      climb = new Climber(new ClimberHardware());
      intake = new CoralIntake(new CoralIntakeHardware());

    } else if (Robot.isSimulation()) {

      elevator = new Elevator(new ElevatorSim());
      wrist =
          new Wrist(
              new WristSim(),
              elevator); // pass the current location of the wrist due to the stacked dof with
      // seperate subsystems
      climb = new Climber(new ClimberSim());

      // pass swerve sim to intake
      if (drivebase.getAsSim().isPresent()) {
      intake = new CoralIntake(new CoralIntakeSim(drivebase.getAsSim().get())); 
      }

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



    // breifly brings elevator down and resets its position
    // Command home = elevator.setVoltage(()->-1).andThen(()->elevator.reset()).withTimeout(0.1);
    // NamedCommands.registerCommand("E_RESET",home);

    // Auto Align
    // NamedCommands.registerCommand("AUTO_ALIGN", drivebase.autoAlign(0));
    NamedCommands.registerCommand("REEF_AUTO_ALIGN", reefAlignmentFactory.generateCommand("L"));
    NamedCommands.registerCommand("SOURCE_AUTO_ALIGN", sourcePathFactory.generateCommand());

    // Intake Auto Commands
    NamedCommands.registerCommand("CORAL_IN", intake.setVoltage(()->1)); // NEVER STOPS
    NamedCommands.registerCommand("CORAL_OUT", intake.runOutakeAuto(-3));// ALMOST INSTANT
    NamedCommands.registerCommand("L_CORAL_OUT", intake.runOutakeAuto(-1)); // ALMOST INSTANT

    NamedCommands.registerCommand("L1", wrist.home()); // ALMOST INSTANT
    // Command autoSource = Commands.deadline(
    Command autoSource = Commands.parallel(elevator.setPoint(() -> 7.2), wrist.setAngle(() -> 30)).withTimeout(3);
    // .until(()->intake.hasCoral()); // FIXED

    NamedCommands.registerCommand("SOURCE", autoSource);

    // Other levels are with the operator commands

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

    // logging autos
    autoChooser.addDefaultOption("DEFAULT", AutoBuilder.buildAuto(Constants.PathPlanner.DEFAULT));
    // autoChooser.addOption("PISSID-DRIVE", drivebase.sysIdDriveMotorCommand());
    // autoChooser.addOption("PISSID-ANGLE", drivebase.sysIdAngleMotorCommand());
    String[] autos = AutoBuilder.getAllAutoNames().toArray(new String[0]);
    for (int i = 0; i < autos.length; i++) {
      autoChooser.addOption(autos[i], AutoBuilder.buildAuto(autos[i]));
    }

    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
  }

//   private Command SetRobotState(Constants.RobotState robotState) {
//     return Commands.parallel(
//         elevator.setPoint(() -> robotState.ElevatorPos),
//         Commands.waitSeconds(.5).andThen(() -> wrist.setAngle(() -> robotState.WristPos)));
//   }

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
            () -> 4);
    Command driveFieldOrientedHybrid =
        drivebase.fieldRelativeTeleop(
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND),
            () ->
                MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> 2.5);

    // same as driveFieldOrientedAnglularVelocity but slower
    Command driveControlled =
        drivebase.fieldRelativeTeleop(
            () ->
                MathUtil.applyDeadband(
                    (-driverXbox.getLeftX()) * 0.25, OperatorConstants.LEFT_Y_DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getLeftY() * 0.25, OperatorConstants.LEFT_X_DEADBAND),
            () ->
                MathUtil.applyDeadband(-driverXbox.getRightX()*.25, OperatorConstants.RIGHT_X_DEADBAND),
            () -> 2);

    Command stow = Commands.parallel(elevator.setPoint(() -> 0),
        wrist.setAngle(() -> 30));
    NamedCommands.registerCommand("STOW", stow);

    Command coralSource =
        Commands.parallel(
            elevator.setPoint(() -> 5.4 + operatorXbox.getLeftY() * 2),
            wrist.setAngle(() -> 26));

    // Command coralL1 =
    //     Commands.parallel(
    //         elevator.setPoint(() -> 3 + operatorXbox.getLeftY() * 2),
    //         wrist.setAngle(() -> 26 + operatorXbox.getRightY() * 10));

    Command coralL2 =
        Commands.parallel(
            elevator.setPoint(() -> 5.4 + operatorXbox.getLeftY() * 2),
            wrist.setAngle(() -> 0 + operatorXbox.getRightY() * 10));
    NamedCommands.registerCommand("L2", coralL2);

    Command algaeL2 =
        Commands.parallel(
            elevator.setPoint(() -> 3.4 + operatorXbox.getLeftY() * 2),
            wrist.setAngle(() -> 30 + operatorXbox.getRightY() * 10));
    Command coralL3 =
        Commands.parallel(
            elevator.setPoint(() -> 11.76 + operatorXbox.getLeftY() * 2), 
            wrist.setAngle(() -> 0 + operatorXbox.getRightY() * 10));
    NamedCommands.registerCommand("L3", coralL3);

    Command algaeL3 =
        Commands.parallel(
            elevator.setPoint(() -> 13.45 + operatorXbox.getLeftY() * 2),
            wrist.setAngle(() -> 26 + operatorXbox.getRightY() * 10));
    Command coralL4 =
        Commands.parallel(
            elevator.setPoint(() -> 25.4 + operatorXbox.getLeftY() * 2),            
            wrist.setAngle(() -> -50.65+ operatorXbox.getRightY() * 10));
            NamedCommands.registerCommand("L4", coralL4);
    // Command coralL4AUTO =
    //     Commands.parallel(
    //             elevator.setPoint(() -> 25.4 + operatorXbox.getLeftY() * 2),
    //             wrist.setAngle(() -> -50.65 + operatorXbox.getRightY() * 10))
    //         .until(() -> !intake.hasCoral());
    

    // im not sure where the inversions are supposed to be but right now
    // it takes inverted controls and returns the correct speeds

    // Drive Controller Commands

    // Generic
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverXbox.b().whileTrue(Commands.runOnce(() -> drivebase.zeroGyro()));
    driverXbox.a().whileTrue(Commands.runOnce(() -> drivebase.lock()).repeatedly());
    driverXbox.a().whileFalse(Commands.run(() -> drivebase.unlock()));
    // driverXbox.x().whileTrue(Commands.run());

    // Season Specififc
    driverXbox.rightTrigger().toggleOnTrue(intake.setVoltage(() -> 1.5)); // In
    driverXbox.leftTrigger().whileTrue(intake.setVoltage(() -> -3)); // Out

    driverXbox
    .leftBumper()
    .onTrue(
        Commands.runOnce(() -> drivebase.setDefaultCommand(driveControlled), drivebase));
driverXbox
    .leftBumper()
    .onFalse(
        Commands.runOnce(
            () -> drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity), drivebase));
 

    // Overides
    operatorXbox.leftStick().whileTrue(elevator.setVoltage(() -> -operatorXbox.getLeftY() * 3));
    wrist.setDefaultCommand(wrist.setVoltage(() -> 0.02));
    operatorXbox.rightStick().whileTrue(wrist.setVoltage(() -> operatorXbox.getRightY() * 2));

    // Climber
    operatorXbox
        .rightTrigger(0.5)
        .whileTrue(
            climb.setVoltage(
                () -> 0.5 - operatorXbox.getRightY() * 8)) // quick climb 0.5 is holding voltage
        .whileFalse(climb.setVoltage(() -> 0)); // POSITIVE IS DOWN

    // Elevator & Wrist
    operatorXbox.start().onTrue(Commands.runOnce(() -> elevator.reset(), elevator));

    operatorXbox.a().onTrue(stow);

    operatorXbox.rightBumper().onTrue(coralSource);

    operatorXbox.b().onTrue(coralL2);
    operatorXbox.b().and(operatorXbox.povUp()).whileTrue(algaeL2);

    operatorXbox.x().onTrue(coralL3);
    operatorXbox.x().and(operatorXbox.povUp()).whileTrue(algaeL3);

    operatorXbox.y().onTrue(coralL4);



    // Drive To pose commands.
    if (Constants.Swerve.VISION) {

        //IT WORKED
    // i want to keep working on this one because this is much easier to drive with
    // and i think its pretty close but i barely got to test it we will see 

        // To Reef 
      driverXbox
          .povUp()
          .whileTrue(reefAlignmentFactory.generateCommand("L").withInterruptBehavior(InterruptionBehavior.kCancelSelf));
       driverXbox
        .povUp()
        .toggleOnFalse(
            Commands.runOnce(
                () -> drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity), drivebase));

    // To Closest Source
    driverXbox.rightBumper()
        .whileTrue(sourcePathFactory.generateCommand());
        // im too proud to remove this 
        
    // //if not working look at logged commands
    // // if is working but bad uncomment the other PID values
    // driverXbox.povLeft()
    // .whileTrue(Commands.defer(()->drivebase.autoAlign(0),Set.of()));
        
    // driverXbox.povRight()
    // .whileTrue(Commands.defer(()->drivebase.autoAlign(1),Set.of()));
}
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void setDriveMode() {
    drivebase.getDefaultCommand();
  }
}
