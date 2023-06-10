package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.commands.DumbDriveTrajectory;
import frc.robot.subsystems.Chassis.ChassisSubsystem;
import org.littletonrobotics.junction.Logger;

public class MotionHandler {

  public enum MotionMode {
    FULL_DRIVE,
    HEADING_CONTROLLER,
    TRAJECTORY,
    TRAJECTORY_DUMB,
    LOCKDOWN,
    NULL
  }

  /**
   * Calculates SwerveModuleState objects using the heading controller.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveHeadingController() {
    double speedFactor = (Robot.driver.getLeftTriggerAxis() > 0.25) ? 0.33 : 1.0;

    double xSpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;
    double ySpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * DriveConstants.MAX_SWERVE_VEL * ChassisSubsystem.allianceFlipper,
                ySpeed * DriveConstants.MAX_SWERVE_VEL * ChassisSubsystem.allianceFlipper,
                Units.degreesToRadians(HeadingController.getInstance().update()) * speedFactor,
                Robot.swerveDrive.getYaw()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SWERVE_VEL);

    return swerveModuleStates;
  }

  /**
   * Calculates SwerveModuleState objects using pure driver control.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveFullControl() {
    double xSpeed =
        MathUtil.applyDeadband(Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);
    double ySpeed =
        MathUtil.applyDeadband(Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);
    double rSpeed =
        MathUtil.applyDeadband(Robot.driver.getRightX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);

    xSpeed *= DriveConstants.MAX_SWERVE_VEL * ChassisSubsystem.allianceFlipper;
    ySpeed *= DriveConstants.MAX_SWERVE_VEL * ChassisSubsystem.allianceFlipper;

    Rotation2d yaw = Robot.swerveDrive.getYaw();

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * yaw.getCos() + ySpeed * yaw.getSin(),
                xSpeed * yaw.getSin() + ySpeed * yaw.getCos(),
                rSpeed * DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC,
                Rotation2d.fromDegrees(0)));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SWERVE_VEL);

    return swerveModuleStates;
  }

  public static SwerveModuleState[] driveTrajectoryDumb() {
    // X and Y are purposefully swapped because in the actual
    // controls, the Y axis of the stick controls the "xSpeed"
    // and vice-versa
    double xSpeed = DumbDriveTrajectory.getYSpeed();
    double ySpeed = DumbDriveTrajectory.getXSpeed();
    double rSpeed = DumbDriveTrajectory.getRSpeed();

    xSpeed *= DriveConstants.MAX_SWERVE_VEL * ChassisSubsystem.allianceFlipper;
    ySpeed *= DriveConstants.MAX_SWERVE_VEL * ChassisSubsystem.allianceFlipper;

    Rotation2d yaw = Robot.swerveDrive.getYaw();

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * yaw.getCos() + ySpeed * yaw.getSin(),
                xSpeed * yaw.getSin() + ySpeed * yaw.getCos(),
                rSpeed * DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC,
                Rotation2d.fromDegrees(0)));

    Logger.getInstance().recordOutput("foo", yaw.getCos());

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SWERVE_VEL);

    return swerveModuleStates;
  }

  /**
   * Calculates swerveModuleStates using the current trajectory.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveTrajectory() {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(TrajectoryController.getInstance().update());

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SWERVE_VEL);

    return swerveModuleStates;
  }

  /**
   * Sets the robot to an unmoving lockdown configuration which is difficult to push.
   *
   * @return The lockdown array of swerveModuleStates.
   */
  public static SwerveModuleState[] lockdown() {
    SwerveModuleState[] swerveModuleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(45))
        };

    return swerveModuleStates;
  }
}
