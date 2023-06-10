package frc.robot.subsystems.Chassis;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis.Modules.Module;
import frc.robot.subsystems.Chassis.Modules.ModuleIO;
import frc.robot.utilities.MotionHandler;
import org.littletonrobotics.junction.Logger;

public class ChassisSubsystem extends SubsystemBase {

  ChassisIO io;
  public final ChassisInputsAutoLogged inputs = new ChassisInputsAutoLogged();

  private final Module frontLeft;
  private final Module frontRight;
  private final Module backLeft;
  private final Module backRight;

  private final SwerveDriveOdometry odometry;
  private Pose2d simOdometryPose;

  private LinearFilter filteredRoll = LinearFilter.singlePoleIIR(0.08, 0.02);
  public double filteredRollVal = 0;

  public static double allianceFlipper = 1;
  public static Rotation2d resetGyroVal = null;

  /**
   * Creates a new SwerveSubsystem (swerve drive) object.
   *
   * @param swerveIO The IO layer of the swerve drive. Change this to change which gyro you're using
   *     (SwerveModuleIOPigeon2 vs SwerveModuleIOSim)
   * @param frontLeft The IO layer for the front left swerve module. Change this to change which
   *     motor controller you're using (SwerveModuleIOSim vs SwerveModuleIOSparkMAX)
   * @param frontRight The IO layer for the front right swerve module.
   * @param backLeft The IO layer for the back left swerve module.
   * @param backRight The IO layer for the back left swerve module.
   */
  public ChassisSubsystem(
      ChassisIO swerveIO,
      ModuleIO frontLeft,
      ModuleIO frontRight,
      ModuleIO backLeft,
      ModuleIO backRight) {
    this.frontLeft = new Module(frontLeft, Constants.DriveConstants.FRONT_LEFT);
    this.frontRight = new Module(frontRight, Constants.DriveConstants.FRONT_RIGHT);
    this.backLeft = new Module(backLeft, Constants.DriveConstants.BACK_LEFT);
    this.backRight = new Module(backRight, Constants.DriveConstants.BACK_RIGHT);
    io = swerveIO;
    io.updateInputs(inputs);

    odometry =
        new SwerveDriveOdometry(
            DriveConstants.KINEMATICS,
            Rotation2d.fromDegrees(inputs.gyroYawPosition),
            new SwerveModulePosition[] {
              this.frontLeft.getPosition(),
              this.frontRight.getPosition(),
              this.backLeft.getPosition(),
              this.backRight.getPosition()
            },
            new Pose2d());

    simOdometryPose = odometry.getPoseMeters();
    seed();
  }

  public void zeroGyro() {
    io.zeroGyro();
  }

  /**
   * Sets the gyro to the given rotation.
   *
   * @param rotation The rotation to reset the gyro to.
   */
  public void resetGyro(Rotation2d rotation) {
    io.resetGyro(rotation);
  }

  /**
   * Resets the SwerveDriveOdometry to the given pose.
   *
   * @param pose The desired pose.
   */
  public void resetOdometry(Pose2d pose) {
    Logger.getInstance().recordOutput("Reset odometry to ", pose);
    odometry.resetPosition(
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);

    simOdometryPose = pose;
  }

  /**
   * Returns the current pose of the robot.
   *
   * @return The position of the robot on the field.
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(inputs.gyroYawPosition);
  }

  public Pose2d getPose() {
    if (Robot.isReal()) {
      return odometry.getPoseMeters();
    } else {
      return simOdometryPose;
    }
  }

  public double getTotalCurrentDraw() {
    return frontLeft.getTotalCurrentDraw()
        + frontRight.getTotalCurrentDraw()
        + backLeft.getTotalCurrentDraw()
        + backRight.getTotalCurrentDraw();
  }

  /**
   * Sets the desired states of the swerve modules.
   *
   * @param swerveModuleStates The array of desired swerveModuleStates. Ensure they are ordered the
   *     same way in this array as they are instantiated into SwerveDriveKinematics.
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public boolean gyroPitchHasChanged() {
    return inputs.gyroPitchPosition == inputs.previousgyroPitchPosition;
  }

  /**
   * Returns the average velocity of the swerve modules.
   *
   * @return The average velocity at which all the swerve modules are moving.
   */
  public double getAverageVelocity() {
    return (frontLeft.getMeasuredState().speedMetersPerSecond
            + frontRight.getMeasuredState().speedMetersPerSecond
            + backLeft.getMeasuredState().speedMetersPerSecond
            + backRight.getMeasuredState().speedMetersPerSecond)
        / 4;
  }

  // Only used for characterization
  public void applyVoltageForCharacterization(double volts) {
    frontLeft.applyVoltageForCharacterization(volts);
    frontRight.applyVoltageForCharacterization(volts);
    backLeft.applyVoltageForCharacterization(volts);
    backRight.applyVoltageForCharacterization(volts);
  }

  /**
   * Updates the odometry of the robot using the swerve module states and the gyro reading. Should
   * be run in periodic() or during every code loop to maintain accuracy.
   */
  public void updateOdometry() {

    odometry.update(
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });

    if (Robot.isSimulation()) {
      SwerveModuleState[] measuredStates =
          new SwerveModuleState[] {
            frontLeft.getMeasuredState(),
            frontRight.getMeasuredState(),
            backLeft.getMeasuredState(),
            backRight.getMeasuredState()
          };
      ChassisSpeeds speeds = Constants.DriveConstants.KINEMATICS.toChassisSpeeds(measuredStates);
      simOdometryPose =
          simOdometryPose.exp(
              new Twist2d(
                  speeds.vxMetersPerSecond * .02,
                  speeds.vyMetersPerSecond * .02,
                  speeds.omegaRadiansPerSecond * .02));

      inputs.gyroYawPosition = simOdometryPose.getRotation().getDegrees();
    }
  }

  public void seed() {
    frontLeft.seed();
    frontRight.seed();
    backLeft.seed();
    backRight.seed();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateOdometry();
    filteredRollVal = filteredRoll.calculate(inputs.gyroRollPosition);
    Logger.getInstance().recordOutput("Swerve/Filtered roll", filteredRollVal);

    switch (Robot.motionMode) {
      case FULL_DRIVE:
        setModuleStates(MotionHandler.driveFullControl());
        break;
      case HEADING_CONTROLLER:
        setModuleStates(MotionHandler.driveHeadingController());
        break;
      case LOCKDOWN:
        setModuleStates(MotionHandler.lockdown());
        break;
      case TRAJECTORY:
        setModuleStates(MotionHandler.driveTrajectory());
        break;
      case TRAJECTORY_DUMB:
        setModuleStates(MotionHandler.driveTrajectoryDumb());
        break;
      default:
        break;
    }

    SwerveModuleState[] measuredModuleStates =
        new SwerveModuleState[] {
          frontLeft.getMeasuredState(),
          frontRight.getMeasuredState(),
          backLeft.getMeasuredState(),
          backRight.getMeasuredState()
        };
    SwerveModuleState[] desiredModuleStates =
        new SwerveModuleState[] {
          frontLeft.getDesiredState(),
          frontRight.getDesiredState(),
          backLeft.getDesiredState(),
          backRight.getDesiredState()
        };

    Logger.getInstance().recordOutput("Swerve/Measured Module States", measuredModuleStates);
    Logger.getInstance().recordOutput("Swerve/Desired Module States", desiredModuleStates);

    Logger.getInstance().processInputs("Swerve/Chassis", inputs);
    Logger.getInstance()
        .recordOutput(
            "Swerve/Odometry Pose",
            new double[] {
              getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()
            });
    Logger.getInstance().recordOutput("Swerve/MotionMode", Robot.motionMode.name());

    ChassisSpeeds currentChassisSpeeds =
        DriveConstants.KINEMATICS.toChassisSpeeds(measuredModuleStates);
    ChassisSpeeds targetChassisSpeeds =
        DriveConstants.KINEMATICS.toChassisSpeeds(desiredModuleStates);

    Logger.getInstance()
        .recordOutput(
            "Swerve/Chassis Speeds/Current/X mps", currentChassisSpeeds.vxMetersPerSecond);
    Logger.getInstance()
        .recordOutput(
            "Swerve/Chassis Speeds/Current/Y mps", currentChassisSpeeds.vyMetersPerSecond);
    Logger.getInstance()
        .recordOutput(
            "Swerve/Chassis Speeds/Current/R radps", currentChassisSpeeds.omegaRadiansPerSecond);

    Logger.getInstance()
        .recordOutput("Swerve/Chassis Speeds/Desired/X mps", targetChassisSpeeds.vxMetersPerSecond);
    Logger.getInstance()
        .recordOutput("Swerve/Chassis Speeds/Desired/Y mps", targetChassisSpeeds.vyMetersPerSecond);
    Logger.getInstance()
        .recordOutput(
            "Swerve/Chassis Speeds/Desired/R radps", targetChassisSpeeds.omegaRadiansPerSecond);
  }

  public static class Commands {}
}
