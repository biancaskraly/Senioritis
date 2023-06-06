// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm.ArmIOSparkMAX;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.ChassisIOMXP;
import frc.robot.subsystems.Chassis.ChassisSubsystem;
import frc.robot.subsystems.Chassis.Modules.ModuleIOSparkMAX;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Pivot.PivotIOFalcon;
import frc.robot.subsystems.Pivot.PivotSubsystem;
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

    autoChooser.addDefaultOption("Blank", new SequentialCommandGroup());

    createSwerveCommands();

    createOperatorCommands();

    LEDStripInit();
  }

  public void LEDStripInit() {
    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    // throw some pretty lights on there
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values for our Red
      m_ledBuffer.setHSV(i, 0, 100, 94);
    }
  }

  private void initializeLogging() {
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    if (isReal()) {
      File sda1 = new File(Constants.Logging.sda1Dir);
      if (sda1.exists() && sda1.isDirectory()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter(Constants.Logging.sda1Dir));
      }
    }
    Logger.getInstance().start();
  }

  private void initializeSubsystems() {
    swerveDrive =
        new ChassisSubsystem(
            new ChassisIOMXP(),
            new ModuleIOSparkMAX(Constants.DriveConstants.FRONT_LEFT),
            new ModuleIOSparkMAX(Constants.DriveConstants.FRONT_RIGHT),
            new ModuleIOSparkMAX(Constants.DriveConstants.BACK_LEFT),
            new ModuleIOSparkMAX(Constants.DriveConstants.BACK_RIGHT));

    arm = new ArmSubsystem(new ArmIOSparkMAX());
    pivot = new PivotSubsystem(new PivotIOFalcon());
    // intake = new IntakeSubsystem(new IntakeIOSparkMAXPWM());
  }

  private void createSwerveCommands() {
    driver.x().onTrue(new InstantCommand(() -> motionMode = MotionMode.LOCKDOWN));

    // driver
    //     .povUp()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               motionMode = MotionMode.HEADING_CONTROLLER;
    //               HeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(0));
    //             }));

    // driver
    //     .povLeft()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               motionMode = MotionMode.HEADING_CONTROLLER;
    //               HeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(90));
    //             }));

    // driver
    //     .povDown()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               motionMode = MotionMode.HEADING_CONTROLLER;
    //               HeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(180));
    //             }));

    // driver
    //     .povRight()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               motionMode = MotionMode.HEADING_CONTROLLER;
    //               HeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(270));
    //             }));
  }

  private void createOperatorCommands() {
    operator
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  Robot.pivot.setTargetAngle(16);
                }));
    operator
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  Robot.pivot.setTargetAngle(Robot.pivot.getTargetAngle() - 1);
                }));
    operator
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  Robot.pivot.setTargetAngle(90);
                }));

    operator
        .leftBumper()
        .onTrue(IntakeSubsystem.Commands.setVoltage(Constants.IntakeConstants.INTAKE_VOLTAGE));

    operator
        .leftBumper()
        .onFalse(IntakeSubsystem.Commands.setVoltage(Constants.IntakeConstants.IDLE_VOLTAGE));

    operator
        .rightBumper()
        .onTrue(IntakeSubsystem.Commands.setVoltage(Constants.IntakeConstants.OUTTAKE_VOLTAGE));

    operator
        .rightBumper()
        .onFalse(IntakeSubsystem.Commands.setVoltage(Constants.IntakeConstants.IDLE_VOLTAGE));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (driver.getRightX() > 0.5) {
      motionMode = MotionMode.FULL_DRIVE;
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    motionMode = MotionMode.TRAJECTORY;
    autoCommand = autoChooser.get();

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
