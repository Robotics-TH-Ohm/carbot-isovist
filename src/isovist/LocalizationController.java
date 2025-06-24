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
    LidarSubsystemListenerRaw,
    // LidarSubsystemListenerSlam {
{
	String filepath;

	DebugPainterOverlay overlay = Robot.debugPainter.getOverlay("Robot");

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
		isovistGrid.paint(Robot.debugPainter.getOverlay("GRID"));
		overlay.clear();
		overlay.fillCircle(500, 500, 50, 255, 0, 0, 200);
		overlay.paint();

		Time.sleep(20000);

    // Demo init
    Robot.motionSubsystem.sendCommand("stoprule T,U50");
    Robot.motionSubsystem.sendCommand("fore 50");

		Time.sleep(5000);
    Robot.motionSubsystem.sendCommand("fore 400");
    // Do something reasonable
    while (isRunning()) {
      // ...
      Time.sleep(100);
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
    if (bundle.containsType(AsyncMotionMessage.TACTIL)) tactile =
      bundle.getBoolean(AsyncMotionMessage.TACTIL); // Taktil value
    if (bundle.containsType(AsyncMotionMessage.US)) us = bundle.getDouble(
      AsyncMotionMessage.US
    );
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

  // public void observedLidarPointsRaw(LidarPackageRaw lidarPackageRaw)
  //   throws Exception {
  //   // ...
  //   // Process Raw Scan
  // }
	
  public void observedLidarPointsSlam(LidarPackageSlam lidarPackageSlam)
    throws Exception {

		// Idk if this is right
		posX = lidarPackageSlam.observationPosX;
		posY = lidarPackageSlam.observationPosY;
		// overlay.clear();
		// overlay.drawCross(posX, posY, 20, 0, 0, 0, 100);

		// ArrayList<ObservedLidarPointSlam> points = lidarPackageSlam.observedPoints;
		// ArrayPointList<Point> pointList = new ArrayPointList<>(points.size());
		// for (ObservedLidarPointSlam p : points) {
		// 	if (Double.isNaN(p.x)) continue;
		// 	pointList.add(new Point(p.x, p.y));
		// }
		//
		// GridPointCloud2D lidarCloud = new GridPointCloud2D(10, pointList);
		// calcPoints(lidarCloud, "LiDAR");


		/// TESTING

		// double[] pos = new double[] { posX, posY };
		// Isovist i = new Isovist(Isovist.samplePointsFromLidar(lidarPackageSlam), pos);
		// i.paint(Robot.debugPainter.getOverlay("LiDAR Isovist"), "0000FF");

		// Isovist j = new Isovist(Isovist.samplePointsFromCloud(cloud, pos), pos);
		// i.paint(Robot.debugPainter.getOverlay("Grid Isovist"), "00FF00");
	
		/// END TESTING


		// Paint seen lidar points -- THIS WORKS
		//   DebugPainterOverlay ovl = Robot.debugPainter.getOverlay("Isovist Points");
		// ovl.clear();
		// for (int i = 0; i < points.size(); i++) {
		// 	ObservedLidarPointSlam point = points.get(i);
		//
		// 	if (!Double.isNaN(point.x)) {
		// 		// Regular point!
		// 		ovl.fillCircle(point.x, point.y, 5, 0, 0, 0, 255);
		// 	} else {
		// 		// No point in this direction, assume maximum distance
		// 		double dx = posX + Math.cos(point.lidarAngleRaw) * MAX_DIST;
		// 		double dy = posY + Math.sin(point.lidarAngleRaw) * MAX_DIST;
		//
		// 		int xxx = point.isReliable() ? 0 : 255;
		// 		ovl.fillCircle(dx, dy, 5, xxx, 0, 0, 255);
		// 	}
		//
		// }
		//   ovl.paint();

		// for (ObservedLidarPointSlam p : points)
		// 	Robot.debugOut.println(p.lidarAngleRaw + " | " + " - " + p.freeSpaceAngle() + " | " + p.x + "/" + p.y);
		// Robot.debugOut.println("--------------------");
		// stop();

		// Paint Isovist rays
		// ovl = Robot.debugPainter.getOverlay("Isovist Rays");
		// ovl.clear();
		// for (int i = 0; i < points.size(); i++) {
		// 	ObservedLidarPointSlam point = points.get(i);
		//  ovl.drawLine(lidarPackageSlam.observationPosX, lidarPackageSlam.observationPosY, point.x, point.y, 0, 255, 0, 100);
		// }
		// ovl.paint();


		// ovl = Robot.debugPainter.getOverlay("Isovist Ploygon");
		// ovl.clear();
		// ovl.fillPoly(lidarPackageSlam.getPositionAsArray(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY), 0, 0, 255, 100);
		// ovl.paint();

		////////////////////////////////////////////////////////////////////////////////

		// calcPoints(cloud, "GRID");
  }

	public void observedLidarPointsRaw(LidarPackageRaw lidarPackageRaw)
	{
		ArrayList<ObservedLidarPointRaw> lidarPoints = lidarPackageRaw.observedPoints;
		PointList2D<Point> points = new ArrayPointList(lidarPoints.size());

		Robot.debugOut.println("Angle:" + lidarPackageRaw.observationAngle);
		Robot.debugOut.println("Pos X:" + lidarPackageRaw.observationPosX);
		Robot.debugOut.println("Pox Y:" + lidarPackageRaw.observationPosY);
		Robot.debugOut.println("-----");

		for (ObservedLidarPointRaw lidarPoint : lidarPoints) {
			if (!lidarPoint.isValid()) continue;
			double theta = (lidarPoint.lidarAngle / 360) * 2 * Math.PI;
			double y = Math.cos(theta) * lidarPoint.lidarDistance;
			double x = Math.sin(theta) * lidarPoint.lidarDistance;
			points.add(new Point(x, y));
		}


		///

		double[] isovistPos = isovistGrid.estimatePosition(points);
		overlay.clear();
		overlay.drawCross(isovistPos[0], isovistPos[1], 50, 0, 0, 255, 255);
		overlay.paint();

		///

		// DebugPainterOverlay isovistOverlay = Robot.debugPainter.getOverlay("Grid Isovist");
		// Cell c = isovistGrid.get(isovistGrid.worldToGrid(Robot.motionSubsystem.estimateCurrentPosition()));
		// c.getIsovist().paint(isovistOverlay, "00FF00");
	}

	int MAX_DIST = 600;
	int RAY_COUNT = 400;
	double RADIUS = 5;

	PointList2D<Point> calcPoints(GridPointCloud2D cloud, String name) {
		PointList2D<Point> gridPs = new ArrayPointList(360);
		
		DebugPainterOverlay ovl = Robot.debugPainter.getOverlay("calcPoints(" + name + ")");
		ovl.clear();
		ovl.fillCircle(posX, posY, 50, 0, 0, 0, 50);

		for (int i = 0; i < RAY_COUNT; ++i) {
			double theta = ((float)i / RAY_COUNT) * 2 * Math.PI;
			double dx = posX + Math.cos(theta) * MAX_DIST;
			double dy = posY + Math.sin(theta) * MAX_DIST;

			// BG line
			ovl.drawLine(posX, posY, dx, dy, 0, 0, 0, 10);
			
			PointList2D<Point> ps = cloud.getInsideBeam(posX, posY, dx, dy, RADIUS);
			Point p = null;

			//for (Point p : ps)
			//	ovl.fillCircle(p.getX(), p.getY(), 10, 0, 0, 255, 255);

			if (ps.size() > 0) {
				// Point found in beam!
				p = ps.getKNearest(posX, posY, Double.POSITIVE_INFINITY, 1).get(0);
			} else {
				// No point, assume maximum viewing distance
				p = new Point(dx, dy);
			}

			gridPs.add(p);
			ovl.drawLine(posX, posY, p.getX(), p.getY(), 0, 0, 0, 100);
			// for (Point p : ps)
			// 	ovl.fillCircle(p.getX(), p.getY(), 5, 0, 255, 0, 255);
		}

		for (Point p : gridPs) {
			ovl.fillCircle(p.getX(), p.getY(), RADIUS, 0, 255, 0, 255);
		}
		ovl.paint();

		ovl = Robot.debugPainter.getOverlay("calcPoints(" + name + ") | POLYGON");
		ovl.clear();
		ovl.fillPoly(gridPs.getAll2D(), 0, 255, 0, 100);
		ovl.paint();

		return gridPs;
	}
}
