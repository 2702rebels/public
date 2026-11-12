package frc.robot.lib;

import edu.wpi.first.wpilibj.DriverStation;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.function.Consumer;
import java.util.function.Function;

public class UdpListenerThread<T> extends Thread {
  private final int port;
  private final int mtu;
  private final Consumer<T> packetConsumer;
  private final Function<ByteBuffer, T> unpack;

  public UdpListenerThread(
      int port, int mtu, Consumer<T> packetConsumer, Function<ByteBuffer, T> unpack) {
    this.port = port;
    this.mtu = mtu;
    this.packetConsumer = packetConsumer;
    this.unpack = unpack;

    setName("UDP Listener [" + port + "]");
    setDaemon(true);
  }

  @Override
  public void run() {
    DatagramSocket socket = null;
    try {
      socket = new DatagramSocket(port);
      final var buffer = new byte[mtu];
      final var packet = new DatagramPacket(buffer, buffer.length);

      System.out.println(getName() + " started");
      while (true) {
        // blocks until next datagram is received
        socket.receive(packet);
        final var data = packet.getData();
        final var bb = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);

        packetConsumer.accept(unpack.apply(bb));
      }
    } catch (Exception e) {
      DriverStation.reportError(getName() + ": " + e.getMessage(), e.getStackTrace());
    } finally {
      if (socket != null) {
        socket.close();
      }
      System.out.println(getName() + " terminated");
    }
  }
}
