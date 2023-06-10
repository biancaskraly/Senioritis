package frc.robot.subsystems.Pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Robot;
import frc.robot.utilities.ArmFeedforwardDeg;
import frc.robot.utilities.SuperstructureConfig;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {

  private PivotIO io;
  private PivotInputsAutoLogged inputs;
  private final ProfiledPIDController controller;
  private final ArmFeedforwardDeg feedforward;
  private double lastPosition;
  private double targetAngle = 90;

  public PivotSubsystem(PivotIO pivotIO) {
    io = pivotIO;
    controller = PivotConstants.GAINS.createProfiledPIDController(new Constraints(300, 300));
    feedforward = PivotConstants.GAINS.createArmDegFeedforward();
    inputs = new PivotInputsAutoLogged();
    io.updateInputs(inputs);
    io.seed(inputs);
  }

  public boolean isAtTarget() {
    return Math.abs(targetAngle - inputs.absoluteEncoderAngle) < PivotConstants.EPSILON;
  }

  public void dontMove() {
    targetAngle = inputs.absoluteEncoderAngle;
  }

  public void setTargetAngle(double newAngle) {
    if (newAngle < PivotConstants.MIN_ANGLE || newAngle > PivotConstants.MAX_ANGLE) {
      return;
    }
    targetAngle = newAngle;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getCurrentAngle() {
    return inputs.absoluteEncoderAngle;
  }

  @Override
  public void periodic() {
    double velocity = (inputs.absoluteEncoderAngle - lastPosition) / 0.02;
    io.updateInputs(inputs);
    double pidValue = controller.calculate(inputs.absoluteEncoderAngle, targetAngle);
    double feedforwardValue =
        feedforward.calculate(
            inputs.absoluteEncoderAngle, targetAngle - inputs.absoluteEncoderAngle, velocity);
    double output = pidValue + feedforwardValue;

    // Code to create a good way to create setpoints
    double v = Robot.operator.getLeftY();
    v = MathUtil.applyDeadband(v, 0.2);
    if (v != 0) {
      output = v * 6;
      targetAngle = inputs.absoluteEncoderAngle;
    }

    output =
        MathUtil.clamp(output, -PivotConstants.MAX_OUTPUT_VOLTS, PivotConstants.MAX_OUTPUT_VOLTS);
    io.setVoltage(output);

    Logger.getInstance().recordOutput("Pivot/Target Angle", targetAngle);
    Logger.getInstance().recordOutput("Pivot/Output", output);
    Logger.getInstance().recordOutput("Pivot/PIDOutput", pidValue);
    Logger.getInstance().recordOutput("Pivot/FFOutput", feedforwardValue);
    Logger.getInstance().recordOutput("Pivot/Velocity", velocity);
    Logger.getInstance()
        .recordOutput(
            "Pivot/Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "null");

    Logger.getInstance().processInputs("Pivot", inputs);
    lastPosition = inputs.absoluteEncoderAngle;
  }

  public static class Commands {
    public static Command setPosition(double angle) {
      return new InstantCommand(() -> Robot.pivot.setTargetAngle(angle));
    }

    public static Command setPosition(SuperstructureConfig config) {
      return setPosition(config.getPivotPosition());
    }

    public static Command setPositionAndWait(double angle) {
      return new RunCommand(() -> Robot.pivot.setTargetAngle(angle), Robot.pivot)
          .until(Robot.pivot::isAtTarget);
    }

    public static Command setPositionAndWait(SuperstructureConfig config) {
      return setPositionAndWait(config.getPivotPosition());
    }
  }
}
