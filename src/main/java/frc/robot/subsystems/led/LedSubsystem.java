package frc.robot.subsystems.led;

// WPILib
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
// CTRE Phoenix CANdle LED Library
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

// Project constants
import frc.robot.Constants.LedConstants;

public class LedSubsystem extends SubsystemBase {

    // LED range constants
    private static final int DEVICE_LED_START = 0;
    private static final int DEVICE_LED_COUNT = 8;
    private static final int STRIP_LED_START = 8;
    private static final int STRIP_LED_COUNT = 123;

    private final CANdle m_candle = new CANdle(LedConstants.CANID, "rio");
    private final int m_candleChannel = 0;

    private boolean m_last5V = false;
    private boolean m_animDirection = true;
    private boolean m_stripInManualColorMode = false;

    private Animation m_toAnimate;

    public enum DeviceLEDState {
        OFF,
        RED,
        GREEN,
        BLUE,
        ORANGE;
    }

    public enum StripAnimationState {
        Fire,
        Larson,
        Rainbow,
        Strobe;
    }

    private DeviceLEDState m_currentDeviceState;
    private StripAnimationState m_currentStripState;

    public LedSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.GRB;
        config.brightnessScalar = 0.1;
        config.vBatOutputMode = VBatOutputMode.Modulated;

        m_candle.configAllSettings(config, 100);

        this.setDeviceState(DeviceLEDState.ORANGE);
        this.setStripState(StripAnimationState.Fire);
    }

    public void toggle5VOverride() {
        System.out.println("5V Override State is: " + m_last5V);
        m_candle.configV5Enabled(m_last5V);
        m_last5V = !m_last5V;
    }

    public void toggleAnimDirection() {
        m_animDirection = !m_animDirection;
    }

    public int getMaximumAnimationCount() {
        return m_candle.getMaxSimultaneousAnimationCount();
    }

    public double getVbat() {
        return m_candle.getBusVoltage();
    }

    public double get5V() {
        return m_candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return m_candle.getCurrent();
    }

    public double getTemperature() {
        return m_candle.getTemperature();
    }

    public void configBrightness(double percent) {
        m_candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        m_candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        m_candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        m_candle.configStatusLedState(offWhenActive, 0);
    }

    public void setDeviceState(DeviceLEDState state) {
        m_currentDeviceState = state;

        switch (state) {
            case OFF:
                m_candle.setLEDs(0, 0, 0, 0, DEVICE_LED_START, DEVICE_LED_COUNT);
                break;
            case RED:
                m_candle.setLEDs(255, 0, 0, 0, DEVICE_LED_START, DEVICE_LED_COUNT);
                break;
            case GREEN:
                m_candle.setLEDs(0, 255, 0, 0, DEVICE_LED_START, DEVICE_LED_COUNT);
                break;
            case BLUE:
                m_candle.setLEDs(0, 0, 255, 0, DEVICE_LED_START, DEVICE_LED_COUNT);
                break;
            case ORANGE:
                m_candle.setLEDs(255, 165, 0, 0, DEVICE_LED_START, DEVICE_LED_COUNT);
                break;
        }

        System.out.println("Device LED set to: " + state.name());
    }

    public void setStripState(StripAnimationState type) {
        m_stripInManualColorMode = false;
        m_currentStripState = type;

        switch (type) {
            case Fire:
                m_toAnimate = new FireAnimation(1, 0.15, STRIP_LED_COUNT, 0.3, 0, m_animDirection, STRIP_LED_START);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(255, 95, 31, 0, 0.25, STRIP_LED_COUNT, BounceMode.Front, 3,
                        STRIP_LED_START);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.7, STRIP_LED_COUNT, m_animDirection, STRIP_LED_START);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, STRIP_LED_COUNT, STRIP_LED_START);
                break;
        }

        m_candle.clearAnimation(m_candleChannel);
        m_candle.animate(m_toAnimate, m_candleChannel);

        System.out.println("Strip animation set to: " + type.name());
    }

    public void setStripColor(int r, int g, int b) {
        if (!m_stripInManualColorMode) {
            m_candle.clearAnimation(m_candleChannel);
        }
        m_stripInManualColorMode = true;
        m_currentStripState = null;

        m_candle.setLEDs(r, g, b, 0, STRIP_LED_START, STRIP_LED_COUNT);
        System.out.println("Strip color set to RGB(" + r + ", " + g + ", " + b + ")");
    }
}