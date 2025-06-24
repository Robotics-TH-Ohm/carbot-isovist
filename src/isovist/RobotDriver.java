package isovist;

import basics.points.PointList2D;
import java.util.ArrayList;
import robotinterface.Robot;
import robotinterface.RobotGeomUtil;
import robotinterface.Time;
import robotinterface.debug.DebugPainterOverlay;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.mss.AsyncMotionMessage;
import robotinterface.mss.AsyncMotionMessageBundle;
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

public class RobotDriver {

  private NavTrajPlanning navTraj = null; // Navigation und Bahnplanung in einer Combi-Instanz
  private Driver driver = null; // Bahnregulation
  private ObstacleContainer obstaclesLidar = null; // Shared obstacle container
  private DebugPainterOverlay ovl = null;

  private double posXMSS = Double.NaN; // Letzte Position X über asynchrone MSS-Meldung
  private double posYMSS = Double.NaN; // Letzte Position Y über asynchrone MSS-Meldung
  private double posAngMSS = Double.NaN; // Letzter Winkel über asynchrone MSS-Meldung

  private double targetX = Double.NaN; // Ziel-Position X
  private double targetY = Double.NaN; // Ziel-Position Y

  private boolean tactileObstacles = false;

  public double getX() {
    return posXMSS;
  }

  public double getY() {
    return posYMSS;
  }

  public double getAngle() {
    return posAngMSS;
  }

  public RobotDriver(ObstacleContainer obstaclesLidar, DebugPainterOverlay ovl) {
    this.obstaclesLidar = obstaclesLidar;
    this.ovl = ovl;
  }

  public boolean canDriveTo(double targetX, double targetY) {
    try {
      TrajectorySequence ts = computeRouteAndTrajectories(targetX, targetY);
      return ts != null;
    } catch (Exception e) {
      return false;
    }
  }

  public void driveTo(double targetX, double targetY) throws Exception {
    TrajectorySequence trajectorySequence = computeRouteAndTrajectories(targetX, targetY);
    if (trajectorySequence == null) {
      this.targetX = Double.NaN;
      this.targetY = Double.NaN;

      throw new Exception("can't find route to target");
    }

    // Paint trajectories and target
    ovl.clear();
    ovl.fillCircle(targetX, targetY, 10, 0, 0, 0, 255);
    if (trajectorySequence.length() > 50)
      trajectorySequence.paint(ovl, 255, 255, 0, 255);

    // Remeber target coordinates (to know when we have arrived) and start driving
    // new trajectory sequence
    this.targetX = targetX;
    this.targetY = targetY;
    driver.drive(trajectorySequence);
  }

  public void waitForTarget() throws Exception {
    if (driver.isHalted()) { // Es gibt keine Trajectorie, z.B. es wurde erkannt, dass die aktuelle
                             // Trajektorien nicht mehr befahrbar ist -> neu Routen

      // Drive backwards when tactile collision occurs to prevent being stuck inside
      // the newly added tactile obstacle points
      if (driver.lastHaltReason() == Driver.HALT_REASON_COLLISION_TACTILE) {
        Robot.motionSubsystem.sendCommand("back 25");
        Time.sleep(1000);
      }

      // Driver halted because we have reached the target
      if (targetReached()) {
        this.targetX = Double.NaN;
        this.targetY = Double.NaN;
        return;
      }

      // Driver halted because the previous route became invalid -> calculate new
      // trajectories
      Robot.debugOut.println("route became invalid, calculating new");
      driveTo(targetX, targetY);
    }
  }

  public void setTactileObstacles(boolean enable) {
    this.tactileObstacles = enable;
  }

  // Alles für das Fahren einrichten
  // - Navigation (Pos2PosRouting)
  // - Bahnplanung (TrajectoryPlanner)
  // - Navigation und Bahnplanung in einer Combi-Instanz (NavTrajPlanning)
  // - Bahnregulation (Driver)
  public void instantiateMotionPlanning() {
    TrajectoryPlanner trajectoryPlannerWithBackdriving = new LongrangePlanner(
        LongrangeProfile
            .createBestPracticeProfile()
            .setPlanningFlagsWithTargetAngle(
                Maneuver.FLAGS_ALLOW_BACKDRIVING |
                    Maneuver.FLAGS_ARCS_LESS_180_DEGREES |
                    Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL,
                Maneuver.FLAGS_ALLOW_BACKDRIVING |
                    Maneuver.FLAGS_ARCS_LESS_180_DEGREES |
                    Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL)
            .setPlanningFlagsWithoutTargetAngle(
                Maneuver.FLAGS_ALLOW_BACKDRIVING |
                    Maneuver.FLAGS_ARCS_LESS_180_DEGREES |
                    Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL,
                Maneuver.FLAGS_ALLOW_BACKDRIVING |
                    Maneuver.FLAGS_ARCS_LESS_180_DEGREES |
                    Maneuver.FLAGS_ALL_WITHOUT_TARGET_ANGLE_WO_CL)
            .setSplitDist(300.0d)
            .setArcarcProfile(
                new double[] { 1.01d, 3.0d, 5.0d },
                new double[] { 1.5d, 1.0d, 1.0d })
            .setIntArcsProfile(
                new double[] { 1.01d, 3.0d, 5.0d },
                new double[] { 1.5d, 1.0d, 1.0d })
            .setWingArcProfile(
                new double[] { 1.01d, 4.0d, 8.0d },
                new double[] { 1.5d, 1.0d, 1.0d })
            .setSnakeProfile(
                new double[] { 1.01d, 1.7d },
                new double[] { 1.5d, 1.0d })
            .setDubinsArcsProfile(
                new double[] { 1.01d, 1.5d, 3.0d },
                new double[] { 1.5d, 1.3d, 1.0d })
            .setTurninplaceData(1, 1) // constantTurnInPlaceCosts,relativeTurnInPlaceCosts
            .setBackdrivingData(500, 5) // constantBackwardCosts,relativeBackwardCosts
            .setChangeDrivingDirectionData(
                1, // changeDrivingDirectionCosts
                Robot.headingDistance * 3.0d) // changeDrivingDirectionMinObstacleDist
            .setAlgoFlags(
                LongrangePlanner.FLAGS_TRAJ_ALGO_ANGLE_ALL_INCLUDING_REVERSE |
                    LongrangePlanner.FLAGS_TRAJ_ALGO_VITERBI)
            .setName("Longrange(WITH back)"),
        null);

    Pos2PosRouting p2p = new Pos2PosRoutingGrid(
        1.0d * Robot.robotWidth / 2d, // obstacleBuffer
        5, // cellSize
        Grid_Astar.FLAGS_STEPWIDTH5 |
            Grid_Astar.FLAGS_DONT_CUT_CORNER |
            Grid_Astar.FLAGS_REMOVE_COLLINEAR_AND_BYPASSES |
            Grid_Astar.FLAGS_CONSIDER_ADDITIONAL_COSTS, // A*-Flags
        2.0d * Robot.robotWidth / 2d, // additionalCostsBuffer
        2.0d, // maxAdditionalCostsFactor
        3.0d // acceptedShortcutCosts
    );

    navTraj = new NavTrajSplitPlanning(p2p, trajectoryPlannerWithBackdriving);

    driver = new RegulatedAheadDriver(
        new LongrangePlanner(
            LongrangeProfile
                .createBestPracticeProfile()
                .setMaxTrajectoryStretch(2.0d)
                .setPlanningFlagsWithTargetAngle(
                    Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL &
                        ~Maneuver.FLAGS_SNAKE &
                        ~Maneuver.FLAGS_SNAKE2 | // Snake ausklammern, da die zu "Trudeln" führen
                        Maneuver.FLAGS_ALLOW_BACKDRIVING | // ORIGINAL: WAS ENABLED
                        Maneuver.FLAGS_NO_CHANGING_BACK_FORE |
                        Maneuver.FLAGS_ARCS_LESS_180_DEGREES |
                        Maneuver.FLAGS_ALLOW_TURN_IN_PLACE) // flags_single_trajectory
                .setArcarcProfile(
                    new double[] { 1.02d, 3.0d, 5.0d },
                    new double[] { 2.5d, 1.0d, 1.0d })
                .setIntArcsProfile(
                    new double[] { 1.02d, 3.0d, 5.0d },
                    new double[] { 2.5d, 1.0d, 1.0d })
                .setWingArcProfile(
                    new double[] { 1.02d, 4.0d, 8.0d },
                    new double[] { 2.5d, 1.0d, 1.0d })
                .setSnakeProfile(
                    new double[] { 1.02d, 1.7d },
                    new double[] { 1.5d, 1.0d }) // Wird eigentlich nicht gebraucht, da oben ausgeschlossen
                .setDubinsArcsProfile(
                    new double[] { 1.02d, 1.5d, 3.0d },
                    new double[] { 1.5d, 1.3d, 1.0d })
                .setTurninplaceData(1, 1) // constantTurnInPlaceCosts,relativeTurnInPlaceCosts,
                .setBackdrivingData(500, 5) // constantBackwardCosts,relativeBackwardCosts
                .setChangeDrivingDirectionData(
                    1, // 50, // changeDrivingDirectionCosts, ORIGINAL: 50
                    Double.NaN) // changeDrivingDirectionMinObstacleDist
                .setAlgoFlags(LongrangePlanner.FLAGS_TRAJ_ALGO_VITERBI), // Keine FLAGS_TRAJ_ALGO_ANGLE_... notwendig,
                                                                         // da immer nur Routen mit 2 Punkten erfragt
                                                                         // werden
            null),
        RegulatedAheadDriver.REGULATION_FLAGS_BESTPRACTICE,
        60, // aheadDist,
        90, // rotateSpeed,
        22, // travelSpeedArc,
        30, // travelSpeedArcFast,
        5, // travelSpeedArcBack,
        50, // minFastArcRadius,
        50, // minFastArcLength,
        25, // travelSpeedLinear,
        30, // travelSpeedLinearFast,
        10, // travelSpeedLinearBack,
        50, // minFastLinearLength
        false);
    driver.setTrajectoryEvaluator(navTraj);
  }

  private TrajectorySequence computeRouteAndTrajectories(double targetX, double targetY) throws Exception {
    RouteTraj routeTraj = navTraj.getRouteTraj(
        posXMSS,
        posYMSS,
        RobotGeomUtil.mssAngle2NavAngle(posAngMSS),
        Double.NaN,
        targetX,
        targetY,
        Double.NaN,
        Double.NaN,
        ObstacleContainer.getObstactles2D(obstaclesLidar));

    if (routeTraj.getRoute() == null) {
      Robot.debugOut.println("No route found!");
      return null;
    }

    TrajectorySequence trajSeq = routeTraj.getTrajectories();
    if (trajSeq == null) {
      Robot.debugOut.println("No trajectories found");
    }

    return trajSeq;
  }

  public boolean targetReached() {
    // No target to drive to -> has to have been reached
    if (Double.isNaN(targetX) || Double.isNaN(targetY))
      return true;

    // Update MSS positions
    double[] pos = Robot.motionSubsystem.estimateCurrentPosition();
    posXMSS = pos[0];
    posYMSS = pos[1];
    posAngMSS = pos[2];

    // 5cm Distanz zum Ziel ist erlaubt
    return (Math.hypot(posXMSS - targetX, posYMSS - targetY) < 5);
  }

  // Methods from framework

  public PointList2D<ObstaclePoint> observedLidarPointsSlam(LidarPackageSlam lidarPackageSlam) throws Exception {
    if (driver != null && !driver.isHalted() && navTraj.hasRouted()) {
      // Lidar-Punkte in die Hindernis-Karte eintragen

      PointList2D<ObstaclePoint> newObstacles = obstaclesLidar.addLidarPoints(
          lidarPackageSlam,
          1.0d);

      // Neu gefundene Hindernisse an die Navigations-Instanz melden

      if (!navTraj.addObstacles(newObstacles.getAll2D())) { // Hinzufügen der neuen Punkte, bei false ging das nicht, da
                                                            // z.B. der grid zu klein war
        driver.halt(
            Driver.HALT_REASON_NAVIGATION,
            "Cannot add new obstacles to navigation world model"); // Bei Fehlschlag neu-Routing initiieren
        return newObstacles;
      }

      // Schauen, ob die aktuelle Bahn noch hindenisfrei befahrbar ist

      TrajectorySequence plannedTraj = driver.plannedTrajectories();
      if (plannedTraj != null) {
        double[] collision = navTraj.trajectoryCollision(plannedTraj);

        if (collision != null) { // Die aktuelle Trajektorie oder die restliche Sequenz ist nicht frei
          driver.halt(
              Driver.HALT_REASON_NAVIGATION,
              "Planned route goes through obstacle");
        }
      }
      return newObstacles;
    }
    return null;
  }

  public void mssAsyncMessages(
      ArrayList<String> messages,
      AsyncMotionMessageBundle bundle) throws Exception {

    // Update current pose
    if (bundle.containsPos()) {
      posXMSS = bundle.getDouble(AsyncMotionMessage.X);
      posYMSS = bundle.getDouble(AsyncMotionMessage.Y);
      posAngMSS = bundle.getDouble(AsyncMotionMessage.ANG);
    }

    // Handle if stopped
    if (bundle.containsType(AsyncMotionMessage.STOPPED)) {
      driver.mssStoppedReceived();
    }

    // Tactile collision detected -> halt
    if (bundle.containsType(AsyncMotionMessage.COLLISION_TACTIL)) {
      // Only save tacitle collisions to map if explicitly enabled to prevent being
      // stuck in them (routeTraj can't route out of obstacles)
      if (tactileObstacles) {
        double[] pos = Robot.motionSubsystem.estimateCurrentPosition();
        obstaclesLidar.addTactilePoints(pos[0], pos[1], pos[2]);
      }

      Robot.debugOut.println("TACTILE COLLISION!");
      driver.halt(Driver.HALT_REASON_COLLISION_TACTILE, "Tactile collision detected");
    }
  }

  // Proxy methods

  public void halt(int haltReason) throws Exception {
    this.driver.halt(haltReason);
  }

  public void halt(int haltReason, java.lang.String haltReasonDescription) throws Exception {
    this.driver.halt(haltReason, haltReasonDescription);
  }
}
