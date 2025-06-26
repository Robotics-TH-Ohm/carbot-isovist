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

	public void observedLidarPointsRaw(LidarPackageRaw lidarPackageRaw)
	{
		ArrayList<ObservedLidarPointRaw> lidarPoints = lidarPackageRaw.observedPoints;
		PointList2D<Point> points = new ArrayPointList(lidarPoints.size());

		Robot.debugOut.println("Angle:" + lidarPackageRaw.observationAngle);
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
