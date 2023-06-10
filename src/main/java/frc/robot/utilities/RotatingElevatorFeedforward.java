package frc.robot.utilities;

public class RotatingElevatorFeedforward {

  private static final double DEG2RADD = Math.PI / 180.0;

  public final double ks;
  public final double kg;
  public final double kv;

  public RotatingElevatorFeedforward(double ks, double kg, double kv) {
    this.ks = ks;
    this.kg = kg;
    this.kv = kv;
  }

  public double calculate(double elevatorAngleDegrees, double positionError, double velocity) {
    return ks * Math.signum(positionError)
        + kg * Math.sin(elevatorAngleDegrees * DEG2RADD)
        + kv * velocity;
  }
}
