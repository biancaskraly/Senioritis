package frc.robot.utilities;

public class ArmFeedforwardDeg {

  private static final double DEG2RADD = Math.PI / 180.0;

  public final double ks;
  public final double kg;
  public final double kv;

  public ArmFeedforwardDeg(double ks, double kg, double kv) {
    this.ks = ks;
    this.kg = kg;
    this.kv = kv;
  }

  public double calculate(
      double positionDegrees, double positionError, double velocityDegreesPerSec) {
    return ks * Math.signum(velocityDegreesPerSec)
        + kg * Math.cos(positionDegrees * DEG2RADD)
        + kv * velocityDegreesPerSec;
  }
}
