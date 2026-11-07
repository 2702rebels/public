package frc.robot.lib;

import java.nio.ByteBuffer;

public interface StructExt {
  /**
   * Deserializes a boolean array from a raw struct serialized ByteBuffer starting at the current
   * position. Will increment the ByteBuffer position by size * kSizeBool bytes. Will not otherwise
   * modify the ByteBuffer (e.g. byte order will not be changed).
   *
   * @param bb ByteBuffer
   * @param size Size of the array
   * @return Boolean array
   */
  static boolean[] unpackBooleanArray(ByteBuffer bb, int size) {
    final var arr = new boolean[size];
    for (var i = 0; i < size; ++i) {
      arr[i] = bb.get() == 1;
    }
    return arr;
  }

  /**
   * Puts array contents to a ByteBuffer starting at the current position. Will increment the
   * ByteBuffer position by size * kSizeBool bytes. Will not otherwise modify the ByteBuffer (e.g.
   * byte order will not be changed).
   *
   * @param bb ByteBuffer
   * @param arr Array to serialize
   */
  static void packArray(ByteBuffer bb, boolean[] arr) {
    for (boolean value : arr) {
      bb.put(value ? (byte) 1 : (byte) 0);
    }
  }
}
