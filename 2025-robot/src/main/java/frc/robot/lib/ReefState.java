package frc.robot.lib;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import java.util.BitSet;

/**
 * A mutable state representation of the reef scoring elements and target.
 *
 * <p>Coral array indices start from the face closest to the alliance wall on the left side and go
 * counterclockwise, corresponding to labels A to L in the game manual.
 *
 * <p>Algae array indices start from the face closest to the alliance wall and go counterclockwise,
 * corresponding to sides AB, CD, EF, GH, IJ, KL.
 */
public class ReefState implements StructSerializable {
  /* Algae presence. */
  public BitSet algae;

  /** Number of corals scored at level 1. */
  public byte coralsL1;

  /** Coral positions scored at level 2. */
  public BitSet coralsL2;

  /** Coral positions scored at level 3. */
  public BitSet coralsL3;

  /** Coral positions scored at level 4. */
  public BitSet coralsL4;

  /**
   * Scoring target position encoded as "(level - 1) * 12 + index" or -1 if no target is specified.
   * Indices between 0 and 11 are reserved, 12 to 23 represent corals at level 2, and so on.
   */
  public byte target;

  /** Indicates that target is locked by the operator. */
  public boolean locked;

  /** Selected reef level for scoring (1, 2, 3, 4) */
  public byte level;

  /** Determines whether target position is marked as scored. */
  public boolean isTargetScored() {
    if (this.target < 12) {
      return false;
    }

    final var level = (this.target / 12);
    final var index = this.target % 12;
    switch (level) {
      case 1:
        return coralsL2.get(index);
      case 2:
        return coralsL3.get(index);
      case 3:
        return coralsL4.get(index);
    }
    return false;
  }

  /** Determines whether position is marked as scored. */
  public boolean isPositionScored(int level, int index) {
    switch (level) {
      case 2:
        return coralsL2.get(index);
      case 3:
        return coralsL3.get(index);
      case 4:
        return coralsL4.get(index);
    }
    return false;
  }

  /** Marks target position as as scored. */
  public void markTargetScored(int target) {
    if (target < 12) {
      return;
    }

    final var level = target / 12;
    final var index = target % 12;

    switch (level) {
      case 2:
        coralsL2.set(index);
        break;
      case 3:
        coralsL3.set(index);
        break;
      case 4:
        coralsL4.set(index);
        break;
    }
  }

  public ReefState() {
    // algae is preloaded
    algae = new BitSet(6);
    algae.set(0, 6);

    coralsL1 = 0;
    coralsL2 = new BitSet(12);
    coralsL3 = new BitSet(12);
    coralsL4 = new BitSet(12);
    target = -1;
    locked = false;
    level = 0;
  }

  public ReefState(
      BitSet algae,
      byte coralsL1,
      BitSet coralsL2,
      BitSet coralsL3,
      BitSet coralsL4,
      byte target,
      boolean locked,
      byte level) {
    this.algae = algae;
    this.coralsL1 = coralsL1;
    this.coralsL2 = coralsL2;
    this.coralsL3 = coralsL3;
    this.coralsL4 = coralsL4;
    this.target = target;
    this.locked = locked;
    this.level = level;
  }

  /** Returns bitset for the specified level index (1...3). */
  public BitSet getCorals(int levelIndex) {
    switch (levelIndex) {
      case 1:
        return coralsL2;
      case 2:
        return coralsL3;
      case 3:
        return coralsL4;
      default:
        throw new IndexOutOfBoundsException();
    }
  }

  /** Resets this instance to the default field state. */
  public void reset() {
    algae.set(0, 6);
    coralsL1 = 0;
    coralsL2.clear();
    coralsL3.clear();
    coralsL4.clear();
    target = -1;
    locked = false;
    level = 0;
  }

  /** ReefState struct for serialization. */
  public static final ReefStateStruct struct = new ReefStateStruct();

  public static final class ReefStateStruct implements Struct<ReefState> {
    @Override
    public Class<ReefState> getTypeClass() {
      return ReefState.class;
    }

    @Override
    public String getTypeName() {
      return "ReefState";
    }

    @Override
    public int getSize() {
      return kSizeInt16 * 3 + kSizeInt8 * 4 + kSizeBool;
    }

    @Override
    public String getSchema() {
      return "uint8 algae;uint8 coralsL1;uint16 coralsL2;uint16 coralsL3;uint16 coralsL4;int8 target;bool locked;uint8 level;";
    }

    @Override
    public ReefState unpack(ByteBuffer bb) {
      return new ReefState(
          BitSet.valueOf(bb.slice(0, 1)),
          bb.get(1),
          BitSet.valueOf(bb.slice(2, 2)),
          BitSet.valueOf(bb.slice(4, 2)),
          BitSet.valueOf(bb.slice(6, 2)),
          bb.get(8),
          bb.get(9) == 1,
          bb.get(10));
    }

    @Override
    public void pack(ByteBuffer bb, ReefState value) {
      bb.put(toByteArray(value.algae, 6));
      bb.put(value.coralsL1);
      bb.put(toByteArray(value.coralsL2, 12));
      bb.put(toByteArray(value.coralsL3, 12));
      bb.put(toByteArray(value.coralsL4, 12));
      bb.put(value.target);
      bb.put(value.locked ? (byte) 1 : (byte) 0);
      bb.put(value.level);
    }

    /**
     * Returns little-endian byte representation of the bitset with the fixed length regardless of
     * the number of bits actually set.
     *
     * <p>Note that built-in BitSet.toByteArray method always packs the bitset into the smallest
     * possible representation, i.e. bitset with all false bits is represented by an empty array,
     * which is not what we want here.
     */
    private static byte[] toByteArray(BitSet bitset, int length) {
      final var bytes = new byte[(length >>> 3) + 1];

      byte b = 0;
      for (var i = 0; i < length; ++i) {
        final var bit = i % 8;
        if (bitset.get(i)) {
          b |= (1 << bit);
        }

        // little-endian representation for multi-byte
        if (bit == 7) {
          bytes[i / 8] = b;
          b = 0;
        }
      }

      if (b != 0) {
        bytes[bytes.length - 1] = b;
      }

      return bytes;
    }
  }
}
