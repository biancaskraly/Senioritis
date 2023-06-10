package frc.robot.utilities;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public abstract class AbstractFeedForward {
  public abstract double calculate(double position, double positionError, double velocity);

  public static class AbstractNull extends AbstractFeedForward {
    public double calculate(double position, double positionError, double velocity) {
      return 0;
    }
  }

  public static class AbstractSimpleFeedForward extends AbstractFeedForward {
    private final SimpleMotorFeedforward ff;

    public AbstractSimpleFeedForward(SimpleMotorFeedforward feedforward) {
      this.ff = feedforward;
    }

    @Override
    public double calculate(double position, double positionError, double velocity) {
      return ff.calculate(velocity);
    }
  }

  public static class AbstractArmFeedForward extends AbstractFeedForward {
    private final ArmFeedforward ff;

    public AbstractArmFeedForward(ArmFeedforward feedforward) {
      this.ff = feedforward;
    }

    @Override
    public double calculate(double position, double positionError, double velocity) {
      return ff.calculate(position, velocity);
    }
  }

  public static class AbstractArmDegFeedForward extends AbstractFeedForward {
    private final ArmFeedforwardDeg ff;

    public AbstractArmDegFeedForward(ArmFeedforwardDeg feedforward) {
      this.ff = feedforward;
    }

    @Override
    public double calculate(double position, double positionError, double velocity) {
      return ff.calculate(position, positionError, velocity);
    }
  }

  public static class AbstractElevatorFeedForward extends AbstractFeedForward {
    private final ElevatorFeedforward ff;

    public AbstractElevatorFeedForward(ElevatorFeedforward feedforward) {
      this.ff = feedforward;
    }

    @Override
    public double calculate(double position, double positionError, double velocity) {
      return ff.calculate(velocity);
    }
  }
}
