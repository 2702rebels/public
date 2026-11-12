package frc.robot.lib;

import java.util.ArrayList;
import java.util.ConcurrentModificationException;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

/**
 * Implements a fixed capacity timestamped queue. However no attempt is made to keep queue
 * timestamped ordered, the order is based purely on insertions.
 */
public class TimestampedCircularBuffer<T> implements Iterable<TimestampedValue<T>> {
  private final long[] timestamps;
  private final List<T> values;
  private final int capacity;
  private int head = 0; // points to the next slot to be inserted/discarded
  private boolean full = false;

  class TimestampedCircularBufferIterator<E> implements Iterator<TimestampedValue<E>> {
    private int current;
    private int head;
    private TimestampedCircularBuffer<E> buffer;

    public TimestampedCircularBufferIterator(TimestampedCircularBuffer<E> buffer) {
      this.buffer = buffer;
      head = buffer.head;
      current = buffer.head - 1;
    }

    private void guardConcurrency() {
      if (buffer.head != head) {
        throw new ConcurrentModificationException("Queue has been modified while iterating");
      }
    }

    public boolean hasNext() throws ConcurrentModificationException {
      guardConcurrency();
      return buffer.full ? buffer.head - current <= buffer.capacity : current >= 0;
    }

    public TimestampedValue<E> next() throws ConcurrentModificationException {
      guardConcurrency();
      final var tv = buffer.getAt(current);
      --current;
      return tv;
    }

    public void remove() {
      throw new UnsupportedOperationException();
    }
  }

  private TimestampedValue<T> getAt(int index) {
    final var translated = index >= 0 ? index : index + capacity;
    return new TimestampedValue<T>(timestamps[translated], values.get(translated));
  }

  public TimestampedCircularBuffer(int capacity) {
    this.capacity = capacity;
    values = new ArrayList<>(capacity);
    for (var i = 0; i < capacity; ++i) {
      values.add(null);
    }

    timestamps = new long[capacity];
  }

  /** Enqueues an element, automatically discards the oldest value if queue is at capacity. */
  public void enqueue(long timestamp, T value) {
    timestamps[head] = timestamp;
    values.set(head, value);

    ++head;
    if (head >= capacity) {
      full = true;
      head = 0;
    }
  }

  /** Returns number of elements in the queue. */
  public int size() {
    return full ? capacity : head;
  }

  /** Returns an iterator that enumerates elements starting from the most recently inserted. */
  public Iterator<TimestampedValue<T>> iterator() {
    return new TimestampedCircularBufferIterator<T>(this);
  }

  /** Returns the newest (most recently) recorded timestamp. */
  public long getHeadTimestamp() {
    if (full) {
      return timestamps[head == 0 ? capacity - 1 : head - 1];
    } else {
      return head > 0 ? timestamps[head - 1] : 0;
    }
  }

  /** Returns the oldest recorded timestamp. */
  public long getTailTimestamp() {
    return full ? timestamps[head] : timestamps[0];
  }

  /** Returns value from the queue with the timestamp closest to the requested one. */
  public Optional<T> findAt(long timestamp) {
    final var size = size();
    if (size == 0) {
      return Optional.empty();
    }

    if (size == 1) {
      return Optional.of(values.get(0));
    }

    // determine newest and oldest timestamps in the queue based on the insertion order
    final var newestIndex = head == 0 ? capacity - 1 : head - 1;
    final var oldestIndex = full ? head : 0;

    if (timestamp <= timestamps[oldestIndex]) {
      return Optional.of(values.get(oldestIndex));
    }

    if (timestamp >= timestamps[newestIndex]) {
      return Optional.of(values.get(newestIndex));
    }

    // binary search, transform indices to [0, capacity) for ease of manipulation
    final var offset = full ? oldestIndex : 0;
    var i = 0; // oldest
    var j = full ? capacity : head; // newest
    var k = 0;

    while (i < j) {
      k = (i + j) / 2;

      final var kIndex = (k + offset) % capacity;

      final var t = timestamps[kIndex];
      if (timestamp == t) {
        return Optional.of(values.get(kIndex));
      }

      if (timestamp < t) {
        if (k > 0 && timestamp > timestamps[(k - 1 + offset) % capacity]) {
          return Optional.of(closest((k - 1 + offset) % capacity, kIndex, timestamp));
        }

        j = k;
      } else {
        if (k < size - 1 && timestamp < timestamps[(k + 1 + offset) % capacity]) {
          return Optional.of(closest(kIndex, (k + 1 + offset) % capacity, timestamp));
        }

        i = k + 1;
      }
    }

    return Optional.of(values.get((k + offset) % capacity));
  }

  private T closest(int i1, int i2, long timestamp) {
    final var t1 = timestamps[i1];
    final var t2 = timestamps[i2];
    return Math.abs(timestamp - t1) < Math.abs(timestamp - t2) ? values.get(i1) : values.get(i2);
  }
}
