package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;

public class PIDFFController extends PIDController {
  private AbstractFeedForward feedforward;
  private double lastMeasurement;

  public PIDFFController(PIDFFGains gains) {
    super(gains.kP.get(), gains.kI.get(), gains.kD.get());

    feedforward = gains.createAbstractFeedForward();
    setTolerance(gains.tolerance.get());
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    setSetpoint(setpoint);
    return calculate(measurement);
  }

  @Override
  public double calculate(double measurement) {
    // Assumes 50 Hz cycle time and that this is called regularly
    double velocity = (measurement - lastMeasurement) / 0.02;
    double value = super.calculate(measurement);
    if (feedforward != null) {
      value += feedforward.calculate(measurement, getPositionError(), velocity);
    }
    lastMeasurement = measurement;
    return value;
  }
}
