// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.ModuleInfo;
import frc.robot.utilities.ModuleInfo.SwerveModuleName;
import frc.robot.utilities.PIDFFGains;
import frc.robot.utilities.SuperstructureConfig;
import lombok.experimental.UtilityClass;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@UtilityClass
public final class Constants {
  public static final boolean TUNING_MODE = true;
  public static final int zero = 0; // in case you need a zero :)
  public static final double π = Math.PI;
  public static final double DOUBLE_PLACEHOLDER = zero;
  public static final int INT_PLACEHOLDER = zero;
  public static final double TUNE_MODULES_DRIVE_SPEED = Units.feetToMeters(3);
  public static final int CAN_TIMEOUT_MS = 200;

  @UtilityClass
  public static final class Logging {
    public static final String sda1Dir = "/media/sda1";
    public static final String sda2Dir = "/media/sda2";
  }

  @UtilityClass
  public static final class RobotMap {
    public static final int DRIVER_PORT = zero;
    public static final int OPERATOR_PORT = 1;

    public static final int GYRO_PORT = 0;

    public static final int ARM = 14;
    public static final int PIVOT_RIGHT = 15;
    public static final int PIVOT_LEFT = 16;
    public static final int INTAKE = 0;

    // Absolute Encoders
    public static final int PIVOT_ENCODER = 1;
    public static final int ARM_ENCODER = 0;

    // LED stuff (very important)
    public static final int LED_PORT = 8;
    public static final int LED_COUNT = 20;
  }

  @UtilityClass
  public static class ArmConstants {
    public static final double GEAR_RATIO = 17.1875;
    public static final int CURRENT_LIMIT = 30;
    public static final double MAX_HEIGHT = 32;
    public static final double MIN_HEIGHT = 1;
    public static final double ENCODER_OFFSET = 0;
    public static final double DIST_PER_ROTATION = -10.5;
    public static final double RETRACTED_POSITION = 5;
    public static final double EPSILON = 5;

    public static final double MAX_PID_OUTPUT_VOLTS = 8;
    public static final double MAX_OUTPUT_VOLTS = 12;

    public static final PIDFFGains GAINS =
        PIDFFGains.builder("ArmController")
            .kP(3)
            .kD(0.00)
            .kI(0.00)
            .kS(1.00)
            .kG(4.00)
            .tolerance(0.5)
            .build();
  }

  @UtilityClass
  public static class PivotConstants {
    public static final double GEAR_RATIO = 22400.0 / 171.0;
    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(true, 20, 50, .25);
    public static final double MAX_ANGLE = 90;
    public static final double MIN_ANGLE = 10;
    public static final double ENCODER_OFFSET = 109;
    public static final double EPSILON = 5;

    public static final double MAX_OUTPUT_VOLTS = 4;

    public static final PIDFFGains GAINS =
        PIDFFGains.builder("PivotController")
            .kP(0.15)
            .kD(0.01)
            .kG(0.20)
            .kS(0.67)
            .armDegFF()
            .tolerance(0.5)
            .build();
  }

  @UtilityClass
  public static class IntakeConstants {
    public static final double IDLE_VOLTAGE = 0;
    public static final double INTAKE_VOLTAGE = 5;
    public static final double OUTTAKE_VOLTAGE = -5;
  }

  @UtilityClass
  public static final class DriveConstants {
    public static final double K_JOYSTICK_TURN_DEADZONE = 0.04;
    public static final double WHEEL_DIAMETER = 4;
    public static final double DRIVE_GEAR_RATIO = 7.36;
    public static final double AZI_GEAR_RATIO = 13.71;
    public static final double DRIVE_DIST_PER_PULSE =
        (1.0 / DRIVE_GEAR_RATIO) * Units.inchesToMeters(WHEEL_DIAMETER) * Math.PI;
    public static final double AZI_DIST_PER_PULSE = (1.0 / AZI_GEAR_RATIO) * 360;
    public static final double MAX_SWERVE_VEL = Units.feetToMeters(100);
    public static final double MAX_SWERVE_AZI = Math.PI;
    public static final double MAX_SWERVE_ACCEL = Units.feetToMeters(5);
    public static final double MAX_ROTATIONAL_SPEED_RAD_PER_SEC = 1;

    public static final int DRIVE_CURRENT_LIMIT = 50;
    public static final int AZI_CURRENT_LIMIT = 20;

    public static final double K_MODULE_DISTANCE_FROM_CENTER = 9.66;

    private static final Translation2d FRONT_LEFT_LOCATION =
        new Translation2d(
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d FRONT_RIGHT_LOCATION =
        new Translation2d(
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d BACK_LEFT_LOCATION =
        new Translation2d(
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d BACK_RIGHT_LOCATION =
        new Translation2d(
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);

    private static final double BUMPERLESS_ROBOT_LENGTH = Units.inchesToMeters(24);
    private static final double BUMPERLESS_ROBOT_WIDTH = Units.inchesToMeters(24);
    private static final double BUMPER_THICKNESS = Units.inchesToMeters(3);

    public static final double FULL_ROBOT_WIDTH = BUMPERLESS_ROBOT_WIDTH + BUMPER_THICKNESS * 2;
    public static final double FULL_ROBOT_LENGTH = BUMPERLESS_ROBOT_LENGTH + BUMPER_THICKNESS * 2;

    public static final double HEADING_CONTROLLER_DRIVER_CHANGE_RATE = 4;
    public static final PIDFFGains K_HEADING_CONTROLLER_GAINS =
        // PIDFFGains.builder("Heading Controller").kP(12).kS(3).kD(0.35).tolerance(1).build();
        PIDFFGains.builder("Heading Controller")
            .kP(DOUBLE_PLACEHOLDER)
            .kS(DOUBLE_PLACEHOLDER)
            .kD(DOUBLE_PLACEHOLDER)
            .tolerance(1)
            .build();
    public static final ModuleInfo FRONT_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_FRONT_LEFT_AZIMUTH_GAINS)
            .driveCANId(7)
            .aziCANId(8)
            .CANCoder(30)
            .offset(-259.5)
            .location(FRONT_LEFT_LOCATION)
            .build();

    public static final ModuleInfo FRONT_RIGHT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_FRONT_RIGHT_AZIMUTH_GAINS)
            .driveCANId(1)
            .aziCANId(2)
            .CANCoder(31)
            .offset(-273.0)
            .location(FRONT_RIGHT_LOCATION)
            .build();

    public static final ModuleInfo BACK_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_BACK_LEFT_AZIMUTH_GAINS)
            .driveCANId(6)
            .aziCANId(5)
            .CANCoder(28)
            .offset(-217.7)
            .location(BACK_LEFT_LOCATION)
            .build();

    public static final ModuleInfo BACK_RIGHT =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_BACK_RIGHT_AZIMUTH_GAINS)
            .driveCANId(3)
            .aziCANId(4)
            .CANCoder(29)
            .offset(-322.3)
            .location(BACK_RIGHT_LOCATION)
            .build();

    @UtilityClass
    public static final class Gains {
      public static final PIDFFGains K_DEFAULT_AZIMUTH_GAINS =
          PIDFFGains.builder("BackRight/Default Azimuth").kP(0.06).tolerance(0.75).build();

      public static final PIDFFGains K_DEFAULT_DRIVING_GAINS =
          PIDFFGains.builder("BackRight/Default Driving").kP(1.0).kS(0.15).kD(0.05).kV(0).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_X =
          PIDFFGains.builder("Trajectory Controller X-Axis").kP(7).kD(0.0).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_Y =
          PIDFFGains.builder("Trajectory Controller Y-Axis").kP(7).kD(0.0).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_ROTATION =
          PIDFFGains.builder("Trajectory Controller Rotation").kP(2.5).kD(0.0).build();

      public static final PIDFFGains K_FRONT_LEFT_AZIMUTH_GAINS = K_DEFAULT_AZIMUTH_GAINS;
      public static final PIDFFGains K_FRONT_RIGHT_AZIMUTH_GAINS = K_DEFAULT_AZIMUTH_GAINS;
      public static final PIDFFGains K_BACK_LEFT_AZIMUTH_GAINS = K_DEFAULT_AZIMUTH_GAINS;
      public static final PIDFFGains K_BACK_RIGHT_AZIMUTH_GAINS = K_DEFAULT_AZIMUTH_GAINS;
    }
  }

  @UtilityClass
  public static class Superstructures {
    // Handles ground intake and hybrid nodes
    public static final SuperstructureConfig GROUND =
        SuperstructureConfig.builder().pivotPosition(10).armHeight(5).build();
    public static final SuperstructureConfig HOME_POSITION =
        SuperstructureConfig.builder().pivotPosition(90).armHeight(5).build();
    public static final SuperstructureConfig SINGLE_SUBSTATION =
        SuperstructureConfig.builder().pivotPosition(40).armHeight(5).build();
    // Untested
    public static final SuperstructureConfig DOUBLE_SUBSTATION =
        SuperstructureConfig.builder().pivotPosition(40).armHeight(5).build();

    public static final SuperstructureConfig CUBE_MID =
        SuperstructureConfig.builder().pivotPosition(45).armHeight(5).build();
    public static final SuperstructureConfig CUBE_HIGH =
        SuperstructureConfig.builder().pivotPosition(50).armHeight(25).build();
    public static final SuperstructureConfig CONE_MID =
        SuperstructureConfig.builder().pivotPosition(50).armHeight(10).build();
    // Untested
    public static final SuperstructureConfig CONE_HIGH =
        SuperstructureConfig.builder().pivotPosition(55).armHeight(30).build();
  }
}
