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
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.Utils.ElevatorWrist;
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

  private AlignToReef reefAlignmentFactory = new AlignToReef(drivebase, tagLayout); // we love 4915
  private DriveToSource sourcePathFactory = new DriveToSource(drivebase,tagLayout);

  private ElevatorWrist EWHandler;

  public Wrist wrist;
  public Elevator elevator;
  private Climber climb;
  private CoralIntake intake;

  public static Pose3d WristPose,ElevetorTop;

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  private final Field2d field = new Field2d();
  
  public RobotContainer() {
    if (Robot.isReal()) {   

      elevator = new Elevator(new ElevatorHardware());
      wrist = new Wrist(new WristHardware(), elevator); // Locations depend on each other
      climb = new Climber(new ClimberHardware());
      intake = new CoralIntake(new CoralIntakeHardware());

    } else if (Robot.isSimulation()) {
        
      elevator = new Elevator(new ElevatorSim());
      wrist = new Wrist(new WristSim(), elevator); 
      climb = new Climber(new ClimberSim());
      // pass swerve sim to intake
      if (drivebase.getAsSim().isPresent()) {
      intake = new CoralIntake(new CoralIntakeSim(drivebase.getAsSim().get())); 
      }

    } else {

      elevator = new Elevator(new ElevatorIO() {});
      wrist = new Wrist(new WristIO() {}, elevator); 
      climb = new Climber(new ClimberIO() {});
      intake = new CoralIntake(new CoralIntakeIO() {});
    }

    WristPose = wrist._wristPose;
    ElevetorTop =elevator.stage3Visuals;

    // handles all Elevator and Wrist movement
    EWHandler = new ElevatorWrist(
        elevator,
        wrist,
        ()->operatorXbox.getLeftY(),// Shim
        ()->operatorXbox.getRightY());

    // Auto Align
    // NamedCommands.registerCommand("AUTO_ALIGN", drivebase.autoAlign(0));
    NamedCommands.registerCommand("REEF_AUTO_ALIGN", reefAlignmentFactory.generateCommand("L"));
    NamedCommands.registerCommand("SOURCE_AUTO_ALIGN", sourcePathFactory.generateCommand());

    // Intake Auto Commands
    NamedCommands.registerCommand("CORAL_IN", intake.setVoltage(()->1)); // NEVER STOPS
    NamedCommands.registerCommand("CORAL_OUT", intake.setVoltage(()->-3,0.25));
    NamedCommands.registerCommand("L_CORAL_OUT", intake.setVoltage(()->-1,0.25));


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
    String[] autos = AutoBuilder.getAllAutoNames().toArray(new String[0]);
    for (int i = 0; i < autos.length; i++) {
      autoChooser.addOption(autos[i], AutoBuilder.buildAuto(autos[i]));
    }

    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
  }

  private void configureBindings() {

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




    
    // Drive Controller Commands

    // Generic
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverXbox.b().whileTrue(Commands.runOnce(() -> drivebase.zeroGyro()));
    driverXbox.x().whileTrue(Commands.runOnce(() -> drivebase.lock()).repeatedly());
    driverXbox.x().whileFalse(Commands.run(() -> drivebase.unlock()));
    driverXbox.a().whileTrue(EWHandler.stow());

    // Season Specififc
    driverXbox.rightTrigger().whileTrue(intake.setVoltage(() -> 3)).toggleOnFalse(intake.setVoltage(()->1.25)); // In

    driverXbox.leftTrigger().whileTrue(intake.setVoltage(() -> -3)); // Out

    driverXbox
        .leftBumper()
        .onTrue(driveControlled); 
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
    operatorXbox.start().onTrue(Commands.runOnce(() -> elevator.reset(), elevator)); // manual zero

    NamedCommands.registerCommand("STOW",EWHandler.stow(1));
    operatorXbox.a().onTrue(EWHandler.stow());

    NamedCommands.registerCommand("SOURCE",EWHandler.coralSource(1) );
    operatorXbox.rightBumper().onTrue(EWHandler.coralSource());

    NamedCommands.registerCommand("L2",EWHandler.coralL2(1) );
    operatorXbox.b().onTrue(EWHandler.coralL2());
    // operatorXbox.b().and(operatorXbox.povUp()).whileTrue(EWHandler.algaeL2());

    NamedCommands.registerCommand("L3",EWHandler.coralL3(1) );
    operatorXbox.x().onTrue(EWHandler.coralL3());
    // operatorXbox.x().and(operatorXbox.povUp()).whileTrue(EWHandler.algaeL3());

    NamedCommands.registerCommand("L4",EWHandler.coralL4(1) );
    operatorXbox.y().onTrue(EWHandler.coralL4());




    // Drive To pose commands.
    if (Constants.Swerve.VISION) {

        //IT WORKED
    // i want to keep working on this one because this is much easier to drive with
    // and i think its pretty close but i barely got to test it we will see 

        // To Reef 
      driverXbox
          .povUp()
          .whileTrue(reefAlignmentFactory.generateCommand("L").withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // To Closest Source
    driverXbox.rightBumper()
        .whileTrue(sourcePathFactory.generateCommand());
        // im too proud to remove this 
}
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void setDriveMode() {
    drivebase.getDefaultCommand();
  }
}
