package isovist.model;

public class Coordinate {
  private final int x;
  private final int y;

  public Coordinate(int x, int y) {
    this.x = x;
    this.y = y;
  }

  public int getX() {
    return x;
  }

  public int getY() {
    return y;
  }

  public Coordinate offset(int dx, int dy) {
    return new Coordinate(getX() + dx, getY() + dy);
  }

  public Coordinate north() {
    return offset(0, 1);
  }

  public Coordinate west() {
    return offset(-1, 0);
  }

  public Coordinate south() {
    return offset(0, -1);
  }

  public Coordinate east() {
    return offset(1, 0);
  }

  public Coordinate[] neighbors() {
    return new Coordinate[] { north(), south(), west(), east() };
  }

  public Coordinate[] diagonalNeighbors() {
    return new Coordinate[] {
        offset(1, 1),
        offset(1, -1),
        offset(-1, -1),
        offset(-1, 1)
    };
  }

  @Override
  public String toString() {
    return String.format("(%d/%d)", getX(), getY());
  }
}
