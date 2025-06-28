package robotcontroller;

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

import robotcontroller.model.IsovistGrid;
import robotcontroller.model.Cell;
import robotcontroller.model.Isovist;

import robotcontroller.util.Serialization;
import org.w3c.dom.Document;



public class MappingController
  extends RobotController
  implements MotionSubsystemListener
{
	String filepath;
	double posX, posY, posAng;
	boolean isPositionValid = false;

	GridPointCloud2D cloud;
	IsovistGrid isovistGrid;

  public MappingController()
	{
    Robot.motionSubsystem.registerMotionListener(this);
    if (Robot.lidarSubsystem == null)
		{
      Robot.debugOut.println("No Lidar Subsystem available - I cannot see anything!");
      return;
    }
  }

  private void paintObstacles(double[][] obstacles, String overlayStr)
	{
    DebugPainterOverlay ovl = Robot.debugPainter.getOverlay(overlayStr);
    ovl.clear();
    for (int i = 0; i < obstacles.length; i++)
      ovl.fillCircle(obstacles[i][0], obstacles[i][1], 5, 200, 0, 0, 255);
    ovl.paint();
  }

  public String getDescription()
	{
    return "Maps the entire environment and saves it as an isovist-based grid. The configuration is the file path.";
  }

  public boolean requiresConfiguration()
	{
    return true;
  }

  public void configure(String params) throws IllegalArgumentException
	{
		this.filepath = params;
  }

  public void run() throws Exception
	{
    if (Robot.lidarSubsystem == null)
		{
      Robot.debugOut.println("This Robot Controller requires a Lidar Subsystem");
      return;
    }

		// Make robot move to get the correct position
    Robot.motionSubsystem.sendCommand("stoprule T,U50");
    Robot.motionSubsystem.sendCommand("fore 100");

		// Wait for the initial position
		while (!isPositionValid)
		{
			Robot.debugOut.println("Waiting for position...");
			Time.sleep(1000);
		}
    Robot.motionSubsystem.sendCommand("stop");
		Robot.debugOut.println("Got position: " + posX + "/" + posY + "@" + posAng);
		Robot.debugOut.println("Calculating isovist grid...");

		// HACK: This doesn't work in real life
		double[][] obstacles = Robot.lidarSubsystem.getAllObstacles();
		cloud = new GridPointCloud2D(10, obstacles, Point.class);
		paintObstacles(cloud.getAll2D(), "All Obstacle Points");

		// Init isovist grid and mark all obstacle cells
		isovistGrid = new IsovistGrid();
		for (double[] o : obstacles)
			isovistGrid.set(o, new Cell(Cell.OBSTACLE));

		// Calculate isovists for all reachable cells based on the point cloud
		isovistGrid.markAllReachable(isovistGrid.worldToGrid(posX, posY));
		isovistGrid.calculateIsovists(cloud);
		isovistGrid.paint(Robot.debugPainter.getOverlay("Isovist Grid"));

		// Serialize to file
		Document doc = Serialization.isovistGridToDocument(isovistGrid);
		Serialization.documentToFile(doc, filepath);

		// Nothing more to do
		stop();
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
      posX = bundle.getDouble(AsyncMotionMessage.X); // Current pos x
      posY = bundle.getDouble(AsyncMotionMessage.Y); // Current pos y
      posAng = bundle.getDouble(AsyncMotionMessage.ANG); // Current angle
			isPositionValid = true;
    }
  }
}
