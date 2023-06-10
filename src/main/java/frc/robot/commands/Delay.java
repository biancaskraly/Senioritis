package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Delay extends CommandBase {
  private final double duration;
  private Timer timer;

  public Delay(double duration) {
    this.duration = duration;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
