package frc.robot.subsystems.Chassis;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;

public class ChassisIOMXP implements ChassisIO {

  private final AHRS gyro;

  public ChassisIOMXP() {
    gyro = new AHRS(SerialPort.Port.kMXP);
    gyro.calibrate();
  }

  @Override
  public void updateInputs(ChassisInputs inputs) {
    inputs.gyroCompassHeading = gyro.getYaw();
    inputs.gyroYawPosition = gyro.getYaw();
    inputs.gyroPitchPosition = gyro.getPitch();
    inputs.gyroRollPosition = gyro.getRoll();
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    gyro.zeroYaw();
    double offset = gyro.getAngleAdjustment() + rotation2d.getDegrees();
    gyro.setAngleAdjustment(offset);
  }

  @Override
  public void zeroGyro() {
    gyro.zeroYaw();
  }
}
