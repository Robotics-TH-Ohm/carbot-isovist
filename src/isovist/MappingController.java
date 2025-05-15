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
import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.Time;
import robotinterface.debug.DebugPainterOverlay;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.lss.LidarSubsystem;
import robotinterface.lss.LidarSubsystemListenerSlam;
import robotinterface.mss.AsyncMotionMessageBundle;
import robotinterface.mss.MotionSubsystemListener;
import robotlib.driver.Driver;
import robotlib.worldmodel.ObstacleContainer;
import robotlib.worldmodel.ObstaclePoint;


public class MappingController
  extends RobotController
  implements
    MotionSubsystemListener,
    // LidarSubsystemListenerRaw,
    LidarSubsystemListenerSlam {

	double posX, posY, posAng;
	double us;
	boolean tactile;

  public MappingController() {
    Robot.motionSubsystem.registerMotionListener(this);
    // Receive MMS messages
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println(
        "No Lidar Subsystem available - I cannot see anything!"
      );
      return;
    }
    try {
      Robot.lidarSubsystem.setTiming(LidarSubsystem.EQUIDISTANT, 500);
      // Robot.lidarSubsystem.registerLidarListenerRaw(this); // Receive raw Lidar points
    } catch (UnsupportedOperationException e) {
      Robot.debugOut.println("Lidar Subsystem does not provide raw points");
    }

    try {
      Robot.lidarSubsystem.registerLidarListenerSlam(this); // Receive corr. Lidar points
      Robot.lidarSubsystem.setMSSCorrection(true);
      // Lidar-SLAM correct position
    } catch (UnsupportedOperationException e) {
      Robot.debugOut.println(
        "Lidar Subsystem does not provide SLAM-corrected points"
      );
    }
  }

  public String getDescription() {
    return "Just a demo controller (Lidar)";
  }

  public boolean requiresConfiguration() {
    return false;
  }

  // This controller needs config
  public void configure(String params) throws IllegalArgumentException {
    // ...
    // Accept a configuration and store it
  }

  public void run() throws Exception {
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println(
        "This Robot Controller requires a Lidar Subsystem"
      );
      return;
    }
    try {
      Robot.lidarSubsystem.resetWorldModel();
    } catch (UnsupportedOperationException e) {
      Robot.debugOut.println("Lidar Subsystem does not use a World Model");
    }
    Robot.lidarSubsystem.startup();
    Robot.motionSubsystem.sendCommand("stoprule T,U50");
    // Demo init
    Robot.motionSubsystem.sendCommand("fore 100");
    // Do something reasonable
    while (isRunning()) {
      // ...
      Time.sleep(100);
      // Do not consume all the CPU power
      // ...
    }
  }

  public void pause() throws Exception {
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println(
        "This Robot Controller requires a Lidar Subsystem"
      );
      return;
    }
    Robot.lidarSubsystem.shutdown();
    Robot.motionSubsystem.sendCommand("stop");
  }

  public void stop() throws Exception {
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println(
        "This Robot Controller requires a Lidar Subsystem"
      );
      return;
    }
    Robot.lidarSubsystem.shutdown();
    Robot.motionSubsystem.sendCommand("stop");
  }

  // ...
  // Stop everything (e.g. motors)
  // and clear state
  public void mssResponse(ArrayList<String> messages, int responseType)
    throws Exception {
    if (
      MotionSubsystemListener.isFailureResponse(responseType)
    ) Robot.debugOut.println("Failure response " + messages.get(0));
  }

  public void mssAsyncMessages(
    ArrayList<String> messages,
    AsyncMotionMessageBundle bundle
  ) throws Exception {
    if (bundle.containsPos()) {
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
    // ...
    // Process SLAM-corrected Scan
  }
}
