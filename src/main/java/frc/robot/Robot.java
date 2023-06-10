// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.Delay;
import frc.robot.commands.DumbDriveTrajectory;
import frc.robot.commands.SetSuperstructure;
import frc.robot.subsystems.Arm.ArmIOSparkMAX;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.ChassisIOMXP;
import frc.robot.subsystems.Chassis.ChassisSubsystem;
import frc.robot.subsystems.Chassis.Modules.ModuleIOSparkMAX;
import frc.robot.subsystems.Intake.IntakeIOSparkMAXPWM;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Pivot.PivotIOFalcon;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.utilities.HeadingController;
import frc.robot.utilities.LEDController;
import frc.robot.utilities.MotionHandler.MotionMode;
import java.io.File;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static ChassisSubsystem swerveDrive;
  public static ArmSubsystem arm;
  public static PivotSubsystem pivot;
  public static IntakeSubsystem intake;
  public static LEDController leds;

  public static MotionMode motionMode = MotionMode.LOCKDOWN;

  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Routine");
  private Command autoCommand;

  @Override
  public void robotInit() {
    initializeLogging();

    initializeSubsystems();

    createAutoCommands();

    createSwerveCommands();

    createOperatorCommands();
  }

  private void createAutoCommands() {
    autoChooser.addDefaultOption(
        "HighCube",
        new SequentialCommandGroup(
            IntakeSubsystem.Commands.setVoltage(IntakeConstants.INTAKE_VOLTAGE),
            new Delay(0.25),
            IntakeSubsystem.Commands.setVoltage(IntakeConstants.IDLE_VOLTAGE),
            new SetSuperstructure(Superstructures.CUBE_HIGH),
            new Delay(0.5),
            IntakeSubsystem.Commands.setVoltage(IntakeConstants.OUTTAKE_VOLTAGE),
            new Delay(0.25),
            IntakeSubsystem.Commands.setVoltage(IntakeConstants.IDLE_VOLTAGE),
            new SetSuperstructure(Superstructures.HOME_POSITION)));
    autoChooser.addOption(
        "HighCubeMobility",
        new SequentialCommandGroup(
            IntakeSubsystem.Commands.setVoltage(IntakeConstants.INTAKE_VOLTAGE),
            new Delay(0.25),
            IntakeSubsystem.Commands.setVoltage(IntakeConstants.IDLE_VOLTAGE),
            new SetSuperstructure(Superstructures.CUBE_HIGH),
            new Delay(0.5),
            IntakeSubsystem.Commands.setVoltage(IntakeConstants.OUTTAKE_VOLTAGE),
            new Delay(0.25),
            IntakeSubsystem.Commands.setVoltage(IntakeConstants.IDLE_VOLTAGE),
            new SetSuperstructure(Superstructures.HOME_POSITION),
            new DumbDriveTrajectory(0, -1, 0, 1)));
  }

  private void initializeLogging() {
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    if (isReal()) {
      File sda1 = new File(Logging.sda1Dir);
      if (sda1.exists() && sda1.isDirectory()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter(Logging.sda1Dir));
      }
    }
    Logger.getInstance().start();
  }

  private void initializeSubsystems() {
    swerveDrive =
        new ChassisSubsystem(
            new ChassisIOMXP(),
            new ModuleIOSparkMAX(DriveConstants.FRONT_LEFT),
            new ModuleIOSparkMAX(DriveConstants.FRONT_RIGHT),
            new ModuleIOSparkMAX(DriveConstants.BACK_LEFT),
            new ModuleIOSparkMAX(DriveConstants.BACK_RIGHT));

    arm = new ArmSubsystem(new ArmIOSparkMAX());
    pivot = new PivotSubsystem(new PivotIOFalcon());
    intake = new IntakeSubsystem(new IntakeIOSparkMAXPWM());
    leds = new LEDController();
  }

  private void createSwerveCommands() {
    driver.x().onTrue(new InstantCommand(() -> motionMode = MotionMode.LOCKDOWN));
    driver.back().onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));

    driver
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.HEADING_CONTROLLER;
                  HeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(0));
                }));

    driver
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.HEADING_CONTROLLER;
                  HeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(90));
                }));

    driver
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.HEADING_CONTROLLER;
                  HeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(180));
                }));

    driver
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.HEADING_CONTROLLER;
                  HeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(270));
                }));
  }

  private void createOperatorCommands() {
    operator.a().onTrue(new SetSuperstructure(Superstructures.CUBE_MID));
    operator.b().onTrue(new SetSuperstructure(Superstructures.CUBE_HIGH));
    operator.y().onTrue(new SetSuperstructure(Superstructures.GROUND));
    operator.x().onTrue(new SetSuperstructure(Superstructures.HOME_POSITION));
    operator.povUp().onTrue(new SetSuperstructure(Superstructures.CONE_MID));
    operator.povRight().onTrue(new SetSuperstructure(Superstructures.CONE_HIGH));
    operator.povDown().onTrue(new SetSuperstructure(Superstructures.SINGLE_SUBSTATION));
    operator.povLeft().onTrue(new SetSuperstructure(Superstructures.DOUBLE_SUBSTATION));

    operator
        .leftBumper()
        .onTrue(IntakeSubsystem.Commands.setVoltage(IntakeConstants.INTAKE_VOLTAGE));

    operator
        .leftBumper()
        .onFalse(IntakeSubsystem.Commands.setVoltage(IntakeConstants.IDLE_VOLTAGE));

    operator
        .rightBumper()
        .onTrue(IntakeSubsystem.Commands.setVoltage(IntakeConstants.OUTTAKE_VOLTAGE));

    operator
        .rightBumper()
        .onFalse(IntakeSubsystem.Commands.setVoltage(IntakeConstants.IDLE_VOLTAGE));

    // I am sorry
    operator
        .leftTrigger(0.5)
        .and(operator.rightTrigger(0.5).negate())
        .onTrue(
            new InstantCommand(
                () -> {
                  Robot.leds.setMode(LEDController.LEDMode.CUBE);
                }));
    operator
        .rightTrigger(0.5)
        .and(operator.rightTrigger(0.5).negate())
        .onTrue(
            new InstantCommand(
                () -> {
                  Robot.leds.setMode(LEDController.LEDMode.CONE);
                }));
    operator
        .leftTrigger(0.5)
        .and(operator.rightTrigger(0.5))
        .onTrue(
            new InstantCommand(
                () -> {
                  Robot.leds.setMode(LEDController.LEDMode.DEOCRATIVE);
                }));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (driver.getRightX() > 0.5) {
      motionMode = MotionMode.FULL_DRIVE;
    }
  }

  @Override
  public void disabledInit() {
    leds.setMode(LEDController.LEDMode.ALLIANCE);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoCommand = autoChooser.get();
    leds.setMode(LEDController.LEDMode.DEOCRATIVE);

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    motionMode = MotionMode.FULL_DRIVE;
    leds.setMode(LEDController.LEDMode.DEOCRATIVE);

    Robot.arm.dontMove();
    Robot.pivot.dontMove();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public String goFast() {
    return "nyoooooooooom";
  }
}
