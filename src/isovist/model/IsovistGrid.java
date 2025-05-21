package isovist.model;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Predicate;

import basics.grid.Grid;
import robotinterface.Robot;
import robotinterface.debug.DebugPainterOverlay;

public class IsovistGrid {

  private Grid<Cell> storage = new Grid<Cell>(Grid.X_POSNEG | Grid.Y_POSNEG);
  private double size = 25;

  public double getGridSize() {
    return size;
  }

  public void paint(DebugPainterOverlay ovl) {
    ovl.clear();

    // Lines
    for (int x = storage.getMinX(); x <= storage.getMaxX() + 1; ++x) {
      ovl.drawLine(x * size, storage.getMinY() * size, x * size, (storage.getMaxY() + 1) * size, 0, 0, 255, 64);
    }
    for (int y = storage.getMinY(); y <= storage.getMaxY() + 1; ++y) {
      ovl.drawLine(storage.getMinX() * size, y * size, (storage.getMaxX() + 1) * size, y * size, 0, 0, 255, 64);
    }

    // Cells
    for (int x = storage.getMinX(); x <= storage.getMaxX(); ++x) {
      for (int y = storage.getMinY(); y <= storage.getMaxY(); ++y) {
        double x_ = x * size + size / 2;
        double y_ = y * size + size / 2;
        Cell cell = get(x, y);

        if (cell.hasAll(Cell.REACHABLE)) {
          ovl.fillSquare(x_, y_, size, 0, 255, 0, 64);
        }

        if (cell.hasAll(Cell.OBSTACLE)) {
          ovl.drawCross(x_, y_, size, 255, 0, 0, 64);
        }

        if (cell.hasAll(Cell.UNREACHABLE)) {
          ovl.fillCircle(x_, y_, size / 2, 255, 0, 0, 128);
        }

        if (cell.hasAll(Cell.START)) {
          ovl.drawText(x_, y_, "s", true, false, size / 2, 0, 0, 0, 128);
        }

        if (cell.hasAll(Cell.FLOODED)) {
          ovl.fillCircle(x_, y_, size / 4, 0, 0, 255, 255);
        }
      }
    }
  }

  // Flood-fill processing //{{{
  // ----------------------------------------------------------------------------

  public synchronized Coordinate floodSearchNearestCell(
      Coordinate start,
      Predicate<Coordinate> searchPredicate,
      Predicate<Coordinate> floodPredicate) {

    // Clear previous flooding markers
    storage.processRect(storage.getMinX(), storage.getMinY(), storage.getMaxX(), storage.getMaxY(),
        (__, x, y) -> set(x, y, get(x, y).without(Cell.FLOODED)));

    // Start flooding from current position, radially outwards
    Queue<Coordinate> q = new LinkedList<>();
    q.add(start);

    while (!q.isEmpty()) {
      Coordinate coord = q.remove();

      // Check if flooding should continue after this cell
      // This has to be done BEFORE testing the current cell to make sure,
      // the cell hasn't been created yet (this might be an abort criteria)
      boolean continueFlooding = floodPredicate.test(coord);

      // Check if the current cell would satisfy the predicate
      if (searchPredicate.test(coord))
        return coord;

      // Continue flooding
      if (safeGet(coord).hasNone(Cell.FLOODED) && continueFlooding) {
        set(coord, safeGet(coord).with(Cell.FLOODED));
        q.add(coord.north());
        q.add(coord.east());
        q.add(coord.south());
        q.add(coord.west());
      }
    }

    // Flooding ended, nothing found (read: cleaning done, area is fully enclosed)
    return null;
  }

	public void markAllReachable(Coordinate start) {
		floodSearchNearestCell(start, _ -> false, coord -> get(coord).hasNone(Cell.OBSTACLE, Cell.UNREACHABLE));
		storage.processRect(
			storage.getMinX(), storage.getMinY(),
			storage.getMaxX(), storage.getMaxY(),
			this::markFloodedReachable
		);
	}

	private void markFloodedReachable(Grid<Cell> grid, int x, int y) {
		Cell c = get(x, y);
		if (c.hasAll(Cell.FLOODED))
			set(x, y, c.with(Cell.REACHABLE));
	}
  // }}}

  // Getters and Setters //{{{
  // ----------------------------------------------------------------------------

  public synchronized boolean contains(int x, int y) {
    return x <= storage.getMaxX() && x >= storage.getMinX() && y <= storage.getMaxY() && y >= storage.getMinY();
  }

  public boolean contains(Coordinate coord) {
    return contains(coord.getX(), coord.getY());
  }

  public synchronized void set(int x, int y, Cell cell) {
    // Update grid boundaries
    // if (x > storage.getMaxX())
    //   storage.getMaxX() = x;
    // if (x < storage.getMinX())
    //   storage.getMinX() = x;
    // if (y > storage.getMaxY())
    //   storage.getMaxY() = y;
    // if (y < storage.getMinY())
    //   storage.getMinY() = y;
    //
    storage.set(x, y, cell);
  }

  public void set(Coordinate coord, Cell cell) {
    set(coord.getX(), coord.getY(), cell);
  }

  public void set(double[] pos, Cell cell) {
    set(worldToGrid(pos), cell);
  }

  public void set(Cell cell) {
    set(Robot.motionSubsystem.estimateCurrentPosition(), cell);
  }

  public void set(int[] pos, Cell cell) {
    set(pos[0], pos[1], cell);
  }

  public synchronized Cell safeGet(int x, int y) {
    Cell cell = storage.get(x, y);
		// Robot.debugOut.println("OBJECT: " + cell);

		if (cell == null) {
			cell = new Cell();
			storage.set(x, y, cell);
		}
		return cell;
  }

  public Cell safeGet(Coordinate coord) {
    return safeGet(coord.getX(), coord.getY());
  }

  public synchronized Cell getOrNull(int x, int y) {
    if (!contains(x, y))
      return null;
    return safeGet(x, y);
  }

  public Cell getOrNull(Coordinate coord) {
    return getOrNull(coord.getX(), coord.getY());
  }

  public synchronized Cell get(int x, int y) {
    // Position is outside the grid -> create cell and expand grid
    if (!contains(x, y)) {
      Cell cell = new Cell();
      set(x, y, cell);
      return cell;
    }

    // Get cell from grid storage
    return safeGet(x, y);
  }

  public Cell get(Coordinate coord) {
    return get(coord.getX(), coord.getY());
  }

  public Cell get(double[] pos) {
    return get(worldToGrid(pos));
  }
  // }}}

  // Conversion helpers //{{{
  // ----------------------------------------------------------------------------

  public Coordinate worldToGrid(double x, double y) {
    int x_ = x < 0 ? (int) (x / size) - 1 : (int) (x / size);
    int y_ = y < 0 ? (int) (y / size) - 1 : (int) (y / size);
    return new Coordinate(x_, y_);
  }

  public Coordinate worldToGrid(double[] pos) {
    return worldToGrid(pos[0], pos[1]);
  }

  public double[] gridToWorld(int x, int y) {
    return new double[] {
        x * size + size / 2,
        y * size + size / 2,
    };
  }

  public double[] gridToWorld(Coordinate pos) {
    return gridToWorld(pos.getX(), pos.getY());
  }
  // }}}

}
