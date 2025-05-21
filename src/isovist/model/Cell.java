package isovist.model;

public class Cell {
  public static final byte REACHABLE = 0b0000_0001;
  public static final byte OBSTACLE = 0b0000_0010;
  public static final byte UNREACHABLE = 0b0000_0100;
  public static final byte START = 0b0000_1000;

  public static final byte FLOODED = 0b0001_0000;

  private final byte byteValue;

  public Cell() {
    this((byte) 0);
  }

  public Cell(byte byteValue, byte... flags) {
    for (byte flag : flags)
      byteValue |= flag;
    this.byteValue = byteValue;
  }

  public Cell with(byte... flags) {
    return new Cell(this.byteValue, flags);
  }

  public Cell without(byte... flags) {
    byte newByteValue = byteValue;
    for (byte flag : flags)
      newByteValue = (byte) (newByteValue & ~flag);
    return new Cell(newByteValue);
  }

  public boolean hasAll(byte... flags) {
    for (byte flag : flags) {
      if ((byteValue & flag) == 0)
        return false;
    }
    return true;
  }

  public boolean hasAny(byte... flags) {
    for (byte flag : flags) {
      if ((byteValue & flag) > 0)
        return true;
    }
    return false;
  }

  public boolean hasNone(byte... flags) {
    return !hasAny(flags);
  }

  public byte toByte() {
    return byteValue;
  }

  @Override
  public String toString() {
    return String.format("%8s", Integer.toBinaryString(toByte())).replace(' ', '0');
  }
}
