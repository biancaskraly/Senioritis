package frc.robot.subsystems.Chassis.Modules;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.utilities.ModuleInfo;

public class ModuleIOSparkMAX implements ModuleIO {

  CANSparkMax driver;
  CANSparkMax azimuth;
  WPI_CANCoder CANCoder;
  CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
  private final ModuleInfo information;

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private RelativeEncoder getAziEncoder() {
    return azimuth.getEncoder();
  }

  public void applyVoltageForCharacterization(double voltage) {
    driver.setVoltage(voltage);
  }

  public WPI_CANCoder getCANCoder() {
    return CANCoder;
  }

  public ModuleIOSparkMAX(ModuleInfo information) {
    this.information = information;
    CANCoder = new WPI_CANCoder(information.getCANCoder());
    CANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    CANCoderConfig.magnetOffsetDegrees = information.getOffset();
    CANCoder.configAllSettings(CANCoderConfig);
    CANCoder.configMagnetOffset(information.getOffset());
    driver = new CANSparkMax(this.information.getDriveCANId(), MotorType.kBrushless);
    azimuth = new CANSparkMax(this.information.getAziCANId(), MotorType.kBrushless);

    // driver.restoreFactoryDefaults();
    // azimuth.restoreFactoryDefaults();

    driver.setCANTimeout(Constants.CAN_TIMEOUT_MS);
    azimuth.setCANTimeout(Constants.CAN_TIMEOUT_MS);

    driver.setSmartCurrentLimit(Constants.DriveConstants.DRIVE_CURRENT_LIMIT);
    azimuth.setSmartCurrentLimit(Constants.DriveConstants.AZI_CURRENT_LIMIT);

    for (int i = 0; i < 30; i++) {
      azimuth.setInverted(true);
      driver.setInverted(true);
    }

    driver.setIdleMode(IdleMode.kCoast);
    azimuth.setIdleMode(IdleMode.kCoast);

    getDriveEncoder().setPositionConversionFactor(Constants.DriveConstants.DRIVE_DIST_PER_PULSE);

    getDriveEncoder()
        .setVelocityConversionFactor((Constants.DriveConstants.DRIVE_DIST_PER_PULSE / 60));

    getAziEncoder().setPositionConversionFactor(Constants.DriveConstants.AZI_DIST_PER_PULSE);
    getAziEncoder().setVelocityConversionFactor(Constants.DriveConstants.AZI_DIST_PER_PULSE / 60);

    seed();
  }

  public void seed() {
    getAziEncoder().setPosition(getCANCoder().getAbsolutePosition());
  }

  public double simplifyDegrees(double degrees) {
    double result = degrees;

    while (result > 360) {
      result -= 360;
    }
    while (result <= 0) {
      result += 360;
    }
    return result;
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {

    inputs.aziAbsoluteEncoderDegrees = CANCoder.getAbsolutePosition();
    inputs.aziAbsoluteEncoderRawDegrees =
        CANCoder.getAbsolutePosition() - CANCoder.configGetMagnetOffset();

    inputs.aziOutputVolts = azimuth.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.aziTempCelcius = azimuth.getMotorTemperature();
    inputs.aziCurrentDrawAmps = azimuth.getOutputCurrent();
    inputs.aziEncoderPositionDeg = getAziEncoder().getPosition();
    inputs.aziEncoderSimplifiedPositionDeg = simplifyDegrees(getAziEncoder().getPosition());
    inputs.aziEncoderVelocityDegPerSecond = getAziEncoder().getVelocity();

    inputs.driveEncoderPositionMetres = getDriveEncoder().getPosition();
    inputs.driveEncoderVelocityMetresPerSecond = getDriveEncoder().getVelocity();
    inputs.driveOutputVolts = driver.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentDrawAmps = driver.getOutputCurrent();
    inputs.driveTempCelcius = driver.getMotorTemperature();
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    azimuth.setVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    driver.setVoltage(driveVolts);
  }
}
