package frc.robot.lib;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * The main class for the CANduit library. This class manages the CAN communication and GPIO pin
 * allocation for the CANduit device.
 */
public class CANduit {
  private Map<Integer, Channel> channels = new HashMap<>();
  private final CAN can;
  private final boolean connected;

  @Logged(name = "Modes", importance = Importance.CRITICAL)
  private final int[] modes = new int[8];

  private static final int CLASS_SET_MODE = 1;
  private static final int CLASS_DIO = 2;
  private static final int CLASS_BCAST_STATUS = 20;
  private static final int CLASS_BCAST_PWM_VALUE = 21;

  private static final int API_BCAST_STATUS_STATES = 0;
  private static final int API_BCAST_STATUS_RESETS = 1;
  private static final int API_BCAST_STATUS_MODES = 2;

  private static final byte MODE_NONE = 0;
  private static final byte MODE_DIO_IN = 1;
  private static final byte MODE_DIO_OUT = 2;
  private static final byte MODE_PWM_IN = 3;

  private abstract class Channel implements AutoCloseable {
    protected final CANduit canduit;
    protected final int gpio;
    protected final byte mode;
    protected final boolean disabled;
    protected long timestamp;
    protected final CANData data = new CANData();
    private byte configuredMode = 0;

    protected Channel(CANduit canduit, int gpio, byte mode, boolean disabled) {
      this.canduit = canduit;
      this.gpio = gpio;
      this.mode = mode;
      this.disabled = disabled;
      canduit.acquireGPIO(gpio, this);

      if (!disabled) {
        canduit.configureGPIO(gpio, mode);
      }
    }

    /** Updates the input. Must be invoked in the robot periodic. */
    protected abstract void update();

    /**
     * Attempts to configure the channel according to its GPIO and mode if it has not been
     * configured yet. Use {@link #isConfigured()} to test the configuration state.
     *
     * @param checkConfiguration indicates that configuration should be checked via blocking call
     * @return true if the channel has been configured successfully.
     */
    public boolean configure(boolean checkConfiguration) {
      if (disabled) {
        return false;
      }

      if (isConfigured()) {
        return true;
      }

      canduit.configureGPIO(gpio, mode);

      if (checkConfiguration) {
        // 10 ms timeout
        final var configuredMode = canduit.readConfiguredMode(gpio, 0.01);
        if (configuredMode.isPresent()) {
          return configuredMode.get() == mode;
        }

        return false;
      }

      return true;
    }

    private void syncConfiguredMode(byte mode) {
      configuredMode = mode;
    }

    /** Returns timestamp of the most recent value. */
    public long getTimestamp() {
      return timestamp;
    }

    /** Releases the channel. */
    public void close() {
      canduit.releaseGPIO(gpio);
    }

    /** Returns an indicator that the channel has been successfully configured. */
    public boolean isConfigured() {
      return mode == configuredMode;
    }

    /** Returns an indicator that the channel is disabled. */
    public boolean isDisabled() {
      return disabled;
    }
  }

  private abstract class DigitalIO extends Channel {
    protected static final byte[] k0 = new byte[] {0b00000000};
    protected static final byte[] k1 = new byte[] {0b00000001};

    protected boolean value;

    private DigitalIO(CANduit canduit, int gpio, byte mode, boolean disabled) {
      super(canduit, gpio, mode, disabled);
    }

    @Override
    protected void update() {
      // [!] CLASS_BCAST_STATUS returns the status for ALL pins and always uses API index 0
      if (canduit.readDataLatest(API_BCAST_STATUS_STATES, CLASS_BCAST_STATUS, data)
          && data.length > 0) {
        // byte 0: GPIO state - bits 0 through 7 correspond to GPIO 0 through 7
        value = (data.data[0] & (0b00000001 << gpio)) != 0;
        timestamp = data.timestamp;
      }
    }

    /** Returns most recent value received. */
    public boolean get() {
      return value;
    }
  }

  public class DigitalInput extends DigitalIO {
    private DigitalInput(CANduit canduit, int gpio, boolean disabled) {
      super(canduit, gpio, MODE_DIO_IN, disabled);
    }
  }

  public class DigitalOutput extends DigitalIO {
    private DigitalOutput(CANduit canduit, int gpio, boolean disabled) {
      super(canduit, gpio, MODE_DIO_OUT, disabled);
    }

    /** Sets the output. */
    public void set(boolean value) {
      canduit.writeData(gpio, CLASS_DIO, value ? k1 : k0);
    }
  }

  public class PWMInput extends Channel {
    protected int period = -1;
    protected int width = -1;
    protected int generation = -1;

    private PWMInput(CANduit canduit, int gpio, boolean disabled) {
      super(canduit, gpio, MODE_PWM_IN, disabled);
    }

    @Override
    protected void update() {
      if (canduit.readDataLatest(gpio, CLASS_BCAST_PWM_VALUE, data) && data.length == 8) {
        final var bb = ByteBuffer.wrap(data.data).order(ByteOrder.LITTLE_ENDIAN);
        width = bb.getInt();
        period = bb.getInt();

        // width and period are 28-bit values, with top 4 bits capturing the generation
        // unsigned shift to extract generation bits
        final var lo = width >>> 28;
        final var hi = period >>> 28;
        generation = lo | (hi << 4);

        width &= 0xFFFFFFF;
        period &= 0xFFFFFFF;

        timestamp = data.timestamp;
      }
    }

    /**
     * Returns most recent period received (in nanoseconds).
     *
     * <p>Returns -1 if no valid value is present.
     */
    public int getPeriod() {
      return period;
    }

    /**
     * Returns most recent pulse width received (in nanoseconds). Returns -1 if no valid value is
     * present.
     */
    public int getPulseWidth() {
      return width;
    }

    /** Returns most recent generation number. */
    public int generation() {
      return generation;
    }

    /** Determines whether valid reading is present. */
    public boolean hasValue() {
      return period > 0 && width >= 0;
    }
  }

  public static class ResetCounters {
    private int powerOn;
    private int brownout;
    private int other;
    private int disable;

    /** Returns number of power-on resets. */
    @Logged(name = "PowerOn", importance = Importance.DEBUG)
    public int powerOn() {
      return powerOn;
    }

    /** Returns number of brownout/glitch resets. */
    @Logged(name = "Brownout", importance = Importance.DEBUG)
    public int brownout() {
      return brownout;
    }

    /** Returns number of other reason resets. */
    @Logged(name = "Other", importance = Importance.DEBUG)
    public int other() {
      return other;
    }

    /** Returns number of disable broadcast resets. */
    @Logged(name = "Disable", importance = Importance.DEBUG)
    public int disable() {
      return disable;
    }
  }

  /** Gets the reset counters. */
  @Logged(name = "ResetCounters", importance = Importance.DEBUG)
  public final ResetCounters resetCounters = new ResetCounters();

  /**
   * Initializes a CANduit device.
   *
   * @param canID The CAN ID of the CANduit device. Negative identifier can be used to initialized
   *     the device in disconnected mode.
   */
  public CANduit(int canID) {
    if (canID >= 0) {
      this.can = new CAN(canID);
      this.connected = true;
    } else {
      this.can = null;
      this.connected = false;
    }
  }

  /** Acquires the specified GPIO and exposes it as a digital input. */
  public DigitalInput createDigitalInput(int gpio) {
    return createDigitalInput(gpio, false);
  }

  /** Acquires the specified GPIO and exposes it as a digital input. */
  public DigitalInput createDigitalInput(int gpio, boolean disabled) {
    return new DigitalInput(this, gpio, disabled);
  }

  /** Acquires the specified GPIO and exposes it as a digital output. */
  public DigitalOutput createDigitalOutput(int gpio) {
    return createDigitalOutput(gpio, false);
  }

  /** Acquires the specified GPIO and exposes it as a digital output. */
  public DigitalOutput createDigitalOutput(int gpio, boolean disabled) {
    return new DigitalOutput(this, gpio, disabled);
  }

  /** Acquires the specified GPIO and exposes it as a generic PWM input. */
  public PWMInput createPWMInput(int gpio) {
    return createPWMInput(gpio, false);
  }

  /** Acquires the specified GPIO and exposes it as a generic PWM input. */
  public PWMInput createPWMInput(int gpio, boolean disabled) {
    return new PWMInput(this, gpio, disabled);
  }

  /** Updates all registered input channels. Must be invoked in the robot periodic. */
  public void update() {
    updateModes();
    updateResets();

    for (var channel : channels.values()) {
      channel.update();
    }
  }

  private final CANData packetModes = new CANData();
  private final CANData packetResets = new CANData();

  /** Updates configured modes on GPIO pins. */
  private void updateModes() {
    if (readDataLatest(API_BCAST_STATUS_MODES, CLASS_BCAST_STATUS, packetModes)
        && packetModes.length == 8) {
      for (var channel : channels.values()) {
        channel.syncConfiguredMode(packetModes.data[channel.gpio]);
      }

      // store modes for telemetry
      for (var i = 0; i < 8; ++i) {
        modes[i] = packetModes.data[i];
      }
    }
  }

  /** Updates various reset counters. */
  private void updateResets() {
    if (readDataLatest(API_BCAST_STATUS_RESETS, CLASS_BCAST_STATUS, packetResets)
        && packetResets.length == 8) {
      resetCounters.powerOn = packetResets.data[0] & 0xFF;
      resetCounters.brownout = packetResets.data[1] & 0xFF;
      resetCounters.other = packetResets.data[2] & 0xFF;
      resetCounters.disable = packetResets.data[3] & 0xFF;
    }
  }

  /** Acquires GPIO. */
  private void acquireGPIO(int gpio, Channel channel) {
    if (gpio < 0 || gpio > 7) {
      throw new IllegalArgumentException("GPIO pin must be between 0 and 7.");
    }

    if (channels.containsKey(gpio)) {
      throw new IllegalArgumentException("GPIO pin " + gpio + " in use.");
    }

    channels.put(gpio, channel);
  }

  /** Releases GPIO. */
  private void releaseGPIO(int gpio) {
    channels.remove(gpio);
    configureGPIO(gpio, MODE_NONE);
  }

  /** Configures GPIO mode. */
  private void configureGPIO(int gpio, byte mode) {
    if (!connected) return;
    final byte[] data = {mode};
    writeData(gpio, CLASS_SET_MODE, data);
  }

  /** Reads low-level data from the CAN. Returns latest packet. */
  private boolean readDataLatest(int gpio, int apiClass, CANData data) {
    if (!connected) return false;
    return can.readPacketLatest(apiClass << 4 | gpio, data);
  }

  /** Writes low-level data to the CAN. */
  private void writeData(int gpio, int apiClass, byte[] data) {
    if (!connected) return;
    can.writePacket(data, apiClass << 4 | gpio);
  }

  /** Requests configured GPIO mode. This is a blocking call. */
  private Optional<Byte> readConfiguredMode(int gpio, double timeoutSeconds) {
    if (!connected) return Optional.empty();

    final var apiId = CLASS_SET_MODE << 4 | gpio;
    can.writeRTRFrame(1, apiId);
    final var timer = new Timer();
    timer.start();

    final var data = new CANData();

    // blocking poll until timeout
    while (timer.get() < timeoutSeconds) {
      if (can.readPacketNew(apiId, data)) {
        return Optional.of(data.data[0]);
      }

      Timer.delay(0.001); // 1ms
    }

    return Optional.empty();
  }
}
