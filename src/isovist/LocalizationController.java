package isovist;

import basics.math.Geom;
import basics.points.PointCloudCreator2D;
import basics.points.PointList2D;
import java.util.ArrayList;
import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.RobotGeomUtil;
import robotinterface.Time;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.lss.LidarSlamWorldModelPoint;
import robotinterface.lss.LidarSubsystem;
import robotinterface.lss.LidarSubsystemListenerSlam;
import robotinterface.lss.ObservedLidarPointSlam;
import robotinterface.mss.AsyncMotionMessage;
import robotinterface.mss.AsyncMotionMessageBundle;
import robotinterface.mss.MotionSubsystem;
import robotinterface.mss.MotionSubsystemListener;
import robotlib.driver.Driver;
import robotlib.driver.RegulatedAheadDriver;
import robotlib.nav.Pos2PosRouting;
import robotlib.nav.grid.Grid_Astar;
import robotlib.nav.grid.Pos2PosRoutingGrid;
import robotlib.navtraj.NavTrajPlanning;
import robotlib.navtraj.NavTrajSplitPlanning;
import robotlib.navtraj.RouteTraj;
import robotlib.traj.TrajectoryPlanner;
import robotlib.traj.longrange.LongrangePlanner;
import robotlib.traj.longrange.LongrangeProfile;
import robotlib.traj.seq.Maneuver;
import robotlib.traj.seq.TrajectorySequence;
import robotlib.worldmodel.ObstacleContainer;
import robotlib.worldmodel.ObstaclePoint;

import java.util.ArrayList;

import basics.points.PointCloudCreator2D;
import basics.points.PointList2D;
import basics.points.Point;
import basics.points.container.GridPointCloud2D;
import basics.points.container.ArrayPointList;
import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.Time;
import robotinterface.debug.DebugPainterOverlay;
import robotinterface.lss.LidarSubsystem;
import robotinterface.lss.LidarSubsystemListenerSlam;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.lss.LidarSubsystemListenerRaw;
import robotinterface.lss.LidarPackageRaw;
import robotinterface.lss.ObservedLidarPointRaw;
import robotinterface.mss.AsyncMotionMessageBundle;
import robotinterface.mss.MotionSubsystemListener;
import robotlib.driver.Driver;
import robotlib.worldmodel.ObstacleContainer;
import robotlib.worldmodel.ObstaclePoint;

import isovist.model.IsovistGrid;
import isovist.model.Cell;
import isovist.model.Isovist;

import isovist.util.Serialization;
import org.w3c.dom.Document;

public class LocalizationController
  extends RobotController
  implements
    // MotionSubsystemListener,
    LidarSubsystemListenerRaw
    // LidarSubsystemListenerSlam {
{
	String filepath;
	DebugPainterOverlay overlay = Robot.debugPainter.getOverlay("Position Estimate");

	double posX, posY, posAng;
	double us;
	boolean tactile;

	IsovistGrid isovistGrid;

  public LocalizationController()
	{
		//   Robot.motionSubsystem.registerMotionListener(this); // Receive MMS messages
		//   if (Robot.lidarSubsystem == null)
		// {
		//     Robot.debugOut.println("No Lidar Subsystem available - I cannot see anything!");
		//     return;
		//   }

    try
		{
      Robot.lidarSubsystem.setTiming(LidarSubsystem.EQUIDISTANT, 500);
      Robot.lidarSubsystem.registerLidarListenerRaw(this); // Receive raw Lidar points
    }
		catch (UnsupportedOperationException e)
		{
      Robot.debugOut.println("Lidar Subsystem does not provide raw points");
    }
		//
		//   try
		// {
		//     Robot.lidarSubsystem.registerLidarListenerSlam(this); // Receive corr. Lidar points
		//     Robot.lidarSubsystem.setMSSCorrection(true); // Lidar-SLAM correct position
		//   }
		// catch (UnsupportedOperationException e)
		// {
		//     Robot.debugOut.println("Lidar Subsystem does not provide SLAM-corrected points");
		//   }
  }


  public String getDescription()
	{
    return "Self-localization using a pre-computed isovist-based grid. The configuration is the file path.";
  }

  public boolean requiresConfiguration()
	{
    return true;
  }

  public void configure(String params) throws IllegalArgumentException
	{
		this.filepath = params;
  }

  public void run() throws Exception {
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println("This Robot Controller requires a Lidar Subsystem");
      return;
    }
    try
		{
      Robot.lidarSubsystem.resetWorldModel();
    }
		catch (UnsupportedOperationException e)
		{
      Robot.debugOut.println("Lidar Subsystem does not use a World Model");
    }
    Robot.lidarSubsystem.startup();

		// Load isovist grid
		Document document = Serialization.readDocumentFromFile(filepath);
		isovistGrid = Serialization.isovistGridFromDocument(document);

		// Debug painting
		isovistGrid.paint(Robot.debugPainter.getOverlay("Grid"));

		// stop();
		// if (1==1) return;

    // Demo init
    Robot.motionSubsystem.sendCommand("stoprule T,U50");
    // Robot.motionSubsystem.sendCommand("fore 50");

		// Time.sleep(5000);
    // Robot.motionSubsystem.sendCommand("curve -100 500");
    Robot.motionSubsystem.sendCommand("rotate -65");
		Time.sleep(1000);
    Robot.motionSubsystem.sendCommand("fore 600");
    // Do something reasonable
    while (isRunning()) {
      // ...
      Time.sleep(1000);
      // Do not consume all the CPU power
      // ...
    }
  }

  public void pause() throws Exception
	{
    if (Robot.lidarSubsystem == null)
		{
      Robot.debugOut.println("This Robot Controller requires a Lidar Subsystem");
      return;
    }
    Robot.lidarSubsystem.shutdown();
    Robot.motionSubsystem.sendCommand("stop");
  }

  public void stop() throws Exception
	{
    if (Robot.lidarSubsystem == null)
		{
      Robot.debugOut.println("This Robot Controller requires a Lidar Subsystem");
      return;
    }
    Robot.lidarSubsystem.shutdown();
    Robot.motionSubsystem.sendCommand("stop");
  }

  public void mssResponse(ArrayList<String> messages, int responseType) throws Exception
	{
    if (MotionSubsystemListener.isFailureResponse(responseType))
			Robot.debugOut.println("Failure response " + messages.get(0));
  }

  public void mssAsyncMessages(ArrayList<String> messages, AsyncMotionMessageBundle bundle) throws Exception
	{
    if (bundle.containsPos())
		{
      posX = bundle.getDouble(AsyncMotionMessage.X);
      // Current pos x
      posY = bundle.getDouble(AsyncMotionMessage.Y);
      // Current pos y
			posAng = bundle.getDouble(AsyncMotionMessage.ANG);
      // Current angle
    }
    if (bundle.containsType(AsyncMotionMessage.TACTIL))
			tactile = bundle.getBoolean(AsyncMotionMessage.TACTIL);
    if (bundle.containsType(AsyncMotionMessage.US))
			us = bundle.getDouble(AsyncMotionMessage.US);
    // US distance
    if (bundle.containsType(AsyncMotionMessage.COLLISION_TACTIL)) {
      // ...
      // Taktil collision detected!
    }
    if (bundle.containsType(AsyncMotionMessage.COLLISION_US)) {
      // ...
      // US collision detected!
    }
  }

	public static int LOST_TRACE_THRESHOLD = 10;
	public static int POINT_HISTORY_COUNT = 5;
	public static double LAST_POINT_DIST = 150;
	private PointList2D<Point> lastPoints = new ArrayPointList(POINT_HISTORY_COUNT);
	private int lastTimeSinceAdd = 0;
	public void observedLidarPointsRaw(LidarPackageRaw lidarPackageRaw)
	{
		ArrayList<ObservedLidarPointRaw> lidarPoints = lidarPackageRaw.observedPoints;
		PointList2D<Point> points = new ArrayPointList(lidarPoints.size());

		Robot.debugOut.println("Pos X:" + lidarPackageRaw.observationPosX);
		Robot.debugOut.println("Pox Y:" + lidarPackageRaw.observationPosY);
		Robot.debugOut.println("-----");

		DebugPainterOverlay pointOverlay = Robot.debugPainter.getOverlay("Raw LiDAR Points");
		pointOverlay.clear();
		for (ObservedLidarPointRaw lidarPoint : lidarPoints) {
			if (!lidarPoint.isValid()) continue;
			double theta = (lidarPoint.lidarAngle / 360) * 2 * Math.PI;
			double y = Math.cos(theta) * lidarPoint.lidarDistance;
			double x = Math.sin(theta) * lidarPoint.lidarDistance;
			points.add(new Point(x, y));
			// System.out.println("Point at " + lidarPoint.lidarAngle + ": " + new Point(x, y).toString());
			// System.out.println("original dist: " + lidarPoint.lidarDistance);
			// System.out.println("---");

			pointOverlay.fillCircle(
				x + lidarPackageRaw.observationPosX,
				y + lidarPackageRaw.observationPosY,
				10,
				255, 0, 0, 255
			);
		}
		pointOverlay.paint();

		Isovist isovist = isovistGrid.findBestMatchingIsovist(points);
		double[] isovistPos = isovist.getPos();

		// Reset last point trace when there was no point added for some time (= trace got lost)
		if (lastTimeSinceAdd >= LOST_TRACE_THRESHOLD)
			lastPoints = new ArrayPointList(POINT_HISTORY_COUNT);

		// Add new point to the trace
		Point newPoint = new Point(isovistPos[0], isovistPos[1]);
		if (lastPoints.size() == POINT_HISTORY_COUNT) {
			lastPoints.sort((p1, p2) -> newPoint.distanceTo2D(p1) > newPoint.distanceTo2D(p2) ? 1 : -1);

			// Point is near -> use to strengthen isovist pos
			if (newPoint.distanceTo2D(lastPoints.get(0)) < LAST_POINT_DIST) {
				lastTimeSinceAdd = 0;
				lastPoints.remove(POINT_HISTORY_COUNT-1);
				lastPoints.add(newPoint);
			} else {
				lastTimeSinceAdd += 1;
			}
		} else {
			lastPoints.add(newPoint);
		}

		double[] sum = new double[2];
		DebugPainterOverlay po = Robot.debugPainter.getOverlay("Last Points");
		po.clear();
		for (Point p : lastPoints) {
			po.fillCircle(p.getX(), p.getY(), 10, 0, 0, 0, 255);
			sum[0] = sum[0] + p.getX();
			sum[1] = sum[1] + p.getY();
		}

		Point p = new Point(sum[0]/lastPoints.size(), sum[1]/lastPoints.size());

		po.fillCircle(p.getX(), p.getY(), 20, 255, 0, 0, 255);
		po.paint();

		overlay.clear();
		overlay.drawCross(isovistPos[0], isovistPos[1], 50, 0, 0, 255, 255);
		overlay.drawArrow(
			isovistPos[0],
			isovistPos[1],
			lidarPackageRaw.observationPosX,
			lidarPackageRaw.observationPosY,
			20,
			255, 0, 255, 255
		);
		overlay.paint();

		isovist.paint(Robot.debugPainter.getOverlay("Matched Isovist"), "0000FF");

		///

		// DebugPainterOverlay isovistOverlay = Robot.debugPainter.getOverlay("Grid Isovist");
		// Cell c = isovistGrid.get(isovistGrid.worldToGrid(Robot.motionSubsystem.estimateCurrentPosition()));
		// c.getIsovist().paint(isovistOverlay, "00FF00");
	}
}
