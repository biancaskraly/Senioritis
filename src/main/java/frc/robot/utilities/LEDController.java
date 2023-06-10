package frc.robot.utilities;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class LEDController {

  private LEDMode mode = LEDMode.ALLIANCE;
  private int animationCycle;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  public LEDController() {
    m_led = new AddressableLED(Constants.RobotMap.LED_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.RobotMap.LED_COUNT);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.start();
  }

  public void periodic() {
    animationCycle++;
    switch (mode) {
      case ALLIANCE:
        if (!DriverStation.isDSAttached()) {
          setSolidColor(0, 0, 100);
        } else {
          Alliance alliance = DriverStation.getAlliance();
          if (alliance == Alliance.Red) {
            setSolidColor(0, 100, 100);
          } else if (alliance == Alliance.Blue) {
            setSolidColor(240, 100, 100);
          } else {
            // Error - Orange
            setSolidColor(30, 100, 100);
          }
        }
        break;
      case CONE:
        setSolidColor(60, 100, 100);
        break;
      case CUBE:
        setSolidColor(300, 100, 100);
        break;
      case DEOCRATIVE:
        decorativePeriodic();
        break;
      default:
        break;
    }
    m_led.setData(m_ledBuffer);
  }

  private void setSolidColor(double hue, double saturation, double value) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, (int) (hue * 0.5), (int) (saturation * 2.55), (int) (value * 2.55));
    }
  }

  private void decorativePeriodic() {
    // Arcane magic
    final int c = 6;
    final int m = 3;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      boolean f = false;
      int j = animationCycle - i;
      f |= j < 0;
      int k = j / c;
      f |= k % m == 2;
      int s = 255 - ((k % m) * 255);
      int l = j % c;
      f |= l > 3;
      int v = 255 - l * 63;
      if (f) s = v = 0;
      m_ledBuffer.setHSV(i, 0, s, v);
    }
  }

  public void setMode(LEDMode mode) {
    this.mode = mode;
    animationCycle = 0;
  }

  public static enum LEDMode {
    ALLIANCE,
    DEOCRATIVE,
    CONE,
    CUBE
  }
}
