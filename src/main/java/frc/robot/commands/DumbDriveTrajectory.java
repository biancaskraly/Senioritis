package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utilities.MotionHandler.MotionMode;

public class DumbDriveTrajectory extends CommandBase {

  private final double xSpeed, ySpeed, rSpeed, duration;
  private Timer timer;
  private static double static_xSpeed, static_ySpeed, static_rSpeed;

  public DumbDriveTrajectory(double xSpeed, double ySpeed, double rSpeed, double duration) {
    addRequirements(Robot.swerveDrive);
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rSpeed = rSpeed;
    this.duration = duration;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    Robot.motionMode = MotionMode.TRAJECTORY_DUMB;
    static_xSpeed = xSpeed;
    static_ySpeed = ySpeed;
    static_rSpeed = rSpeed;
    timer.restart();
  }

  public void end(boolean interrupted) {
    static_xSpeed = 0;
    static_ySpeed = 0;
    static_rSpeed = 0;
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }

  public static double getXSpeed() {
    return static_xSpeed;
  }

  public static double getYSpeed() {
    return static_ySpeed;
  }

  public static double getRSpeed() {
    return static_rSpeed;
  }
}
