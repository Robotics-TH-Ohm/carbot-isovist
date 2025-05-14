package isovist;

import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.RobotGeomUtil;
import robotinterface.Time;

import robotinterface.mss.MotionSubsystem;
import robotinterface.mss.MotionSubsystemListener;
import robotinterface.mss.AsyncMotionMessage;
import robotinterface.mss.AsyncMotionMessageBundle;

import robotinterface.lss.LidarSubsystem;
import robotinterface.lss.LidarSubsystemListenerSlam;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.lss.ObservedLidarPointSlam;
import robotinterface.lss.LidarSlamWorldModelPoint;

import robotlib.driver.Driver;
import robotlib.driver.RegulatedAheadDriver;

import robotlib.traj.TrajectoryPlanner;
import robotlib.traj.longrange.LongrangeProfile;
import robotlib.traj.longrange.LongrangePlanner;
import robotlib.traj.seq.Maneuver;
import robotlib.traj.seq.TrajectorySequence;

import robotlib.nav.Pos2PosRouting;
import robotlib.nav.grid.Grid_Astar;
import robotlib.nav.grid.Pos2PosRoutingGrid;

import robotlib.navtraj.NavTrajPlanning;
import robotlib.navtraj.NavTrajSplitPlanning;
import robotlib.navtraj.RouteTraj;

import robotlib.worldmodel.ObstacleContainer;
import robotlib.worldmodel.ObstaclePoint;

import basics.math.Geom;
import basics.points.PointCloudCreator2D;
import basics.points.PointList2D;

import java.util.ArrayList;

public class DriveDemo extends RobotController implements MotionSubsystemListener,LidarSubsystemListenerSlam {

    private NavTrajPlanning navTraj=null;     // Navigation und Bahnplanung in einer Combi-Instanz
    private Driver driver=null;               // Bahnregulation


    private double posXMSS=Double.NaN;        // Letzte Position X über asynchrone MSS-Meldung
    private double posYMSS=Double.NaN;        // Letzte Position Y über asynchrone MSS-Meldung
    private double posAngMSS=Double.NaN;      // Letzter Winkel über asynchrone MSS-Meldung

    private double targetX=Double.NaN;        // Ziel-Position X
    private double targetY=Double.NaN;        // Ziel-Position Y

    private ObstacleContainer obstaclesLidar=null;   // Container, der alle bisher gefundenen Hindernispunkte enthält

    public DriveDemo() {
        Robot.motionSubsystem.registerMotionListener(this);

        // Ohne Lidar kann diese Anwendung nicht laufen -> dann sofort beenden

        if (Robot.lidarSubsystem==null) {
            Robot.debugOut.println("No Lidar Subsystem available - I cannot see anything!");
            return;
        }


        // Lidar-SLAM-Mechanismus einrichten 

        try {
            Robot.lidarSubsystem.setTiming(LidarSubsystem.EQUIDISTANT,1000);
            Robot.lidarSubsystem.registerLidarListenerSlam(this);
            Robot.lidarSubsystem.setMSSCorrection(true);
        }
        catch (UnsupportedOperationException e) {
            Robot.debugOut.println("Lidar Subsystem does not provide SLAM-correction");
        }


        // Hindernis-Karte einrichten

        obstaclesLidar=new ObstacleContainer(
                                          PointCloudCreator2D.TYPE_GRID,10.0d,
                                          ObstacleContainer.ADD_MODE_FUSION_WEIGHT,
                                          3.0d);  // FUSION_MAX_LIDAR_DIST
    }


    // Alles für das Fahren einrichten
    // - Navigation (Pos2PosRouting)
    // - Bahnplanung (TrajectoryPlanner)
    // - Navigation und Bahnplanung in einer Combi-Instanz (NavTrajPlanning)
    // - Bahnregulation (Driver)
    //


    private void instantiateMotionPlanning() {

        TrajectoryPlanner trajectoryPlannerWithBackdriving=new LongrangePlanner(
                                             LongrangeProfile.createBestPracticeProfile()
                                             .setPlanningFlagsWithTargetAngle(Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_ARCS_UPTO_360_DEGREES | Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL,    Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_ARCS_UPTO_360_DEGREES | Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL)
                                             .setPlanningFlagsWithoutTargetAngle(Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_ARCS_UPTO_360_DEGREES | Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL, Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_ARCS_UPTO_360_DEGREES | Maneuver.FLAGS_ALL_WITHOUT_TARGET_ANGLE_WO_CL)
                                             .setSplitDist(300.0d)
                                             .setArcarcProfile(new double[]{1.01d,3.0d,5.0d},new double[]{1.5d,1.0d,1.0d})
                                             .setIntArcsProfile(new double[]{1.01d,3.0d,5.0d},new double[]{1.5d,1.0d,1.0d})
                                             .setWingArcProfile(new double[]{1.01d,4.0d,8.0d},new double[]{1.5d,1.0d,1.0d})
                                             .setSnakeProfile(new double[]{1.01d,1.7d},new double[]{1.5d,1.0d})
                                             .setDubinsArcsProfile(new double[]{1.01d,1.5d,3.0d},new double[]{1.5d,1.3d,1.0d})
                                             .setTurninplaceData(500,2)      // constantTurnInPlaceCosts,relativeTurnInPlaceCosts
                                             .setBackdrivingData(100,5)      // constantBackwardCosts,relativeBackwardCosts
                                             .setChangeDrivingDirectionData(50,      // changeDrivingDirectionCosts
                                                       Robot.headingDistance*3.0d)   // changeDrivingDirectionMinObstacleDist        
                                             .setAlgoFlags(LongrangePlanner.FLAGS_TRAJ_ALGO_ANGLE_ALL_INCLUDING_REVERSE | LongrangePlanner.FLAGS_TRAJ_ALGO_VITERBI)
                                             .setName("Longrange(WITH back)"),
                                             null
                                            );


        Pos2PosRouting p2p=new Pos2PosRoutingGrid(
                                           1.1d*Robot.robotWidth/2d,                                  // obstacleBuffer
                                           10,                                                        // cellSize
                                           Grid_Astar.FLAGS_STEPWIDTH5 | Grid_Astar.FLAGS_DONT_CUT_CORNER 
                                           | Grid_Astar.FLAGS_REMOVE_COLLINEAR_AND_BYPASSES | Grid_Astar.FLAGS_CONSIDER_ADDITIONAL_COSTS,   // A*-Flags
                                           2.0d*Robot.robotWidth/2d,                                  // additionalCostsBuffer
                                           2.0d,                                                      // maxAdditionalCostsFactor 
                                           3.0d                                                       // acceptedShortcutCosts
                                          );

        navTraj=new NavTrajSplitPlanning(p2p,trajectoryPlannerWithBackdriving);

        driver=new RegulatedAheadDriver(
                                         new LongrangePlanner(
                                                LongrangeProfile.createBestPracticeProfile()
                                                    .setMaxTrajectoryStretch(2.0d)
                                                    .setPlanningFlagsWithTargetAngle(
                                                                       Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL  & ~Maneuver.FLAGS_SNAKE & ~Maneuver.FLAGS_SNAKE2     // Snake ausklammern, da die zu "Trudeln" führen
                                                                       | Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_NO_CHANGING_BACK_FORE 
                                                                       | Maneuver.FLAGS_ARCS_LESS_180_DEGREES
                                                                       | Maneuver.FLAGS_ALLOW_TURN_IN_PLACE
                                                                     )  // flags_single_trajectory
                                                    .setArcarcProfile(new double[]{1.02d,3.0d,5.0d},new double[]{2.5d,1.0d,1.0d})
                                                    .setIntArcsProfile(new double[]{1.02d,3.0d,5.0d},new double[]{2.5d,1.0d,1.0d})
                                                    .setWingArcProfile(new double[]{1.02d,4.0d,8.0d},new double[]{2.5d,1.0d,1.0d})  
                                                    .setSnakeProfile(new double[]{1.02d,1.7d},new double[]{1.5d,1.0d})  // Wird eigentlich nicht gebraucht, da oben ausgeschlossen
                                                    .setDubinsArcsProfile(new double[]{1.02d,1.5d,3.0d},new double[]{1.5d,1.3d,1.0d})
                                                    .setTurninplaceData(500,2)      // constantTurnInPlaceCosts,relativeTurnInPlaceCosts,
                                                    .setBackdrivingData(100,5)      // constantBackwardCosts,relativeBackwardCosts
                                                    .setChangeDrivingDirectionData(50,   // changeDrivingDirectionCosts
                                                           Double.NaN)                   // changeDrivingDirectionMinObstacleDist                                                    
                                                    .setAlgoFlags(LongrangePlanner.FLAGS_TRAJ_ALGO_VITERBI),    // Keine FLAGS_TRAJ_ALGO_ANGLE_... notwendig, da immer nur Routen mit 2 Punkten erfragt werden
                                                    null
                                         ),
                                         RegulatedAheadDriver.REGULATION_FLAGS_BESTPRACTICE,
                                         60, // aheadDist,
                                         90, // rotateSpeed,
                                         22, // travelSpeedArc,
                                         30, // travelSpeedArcFast,
                                         5,  // travelSpeedArcBack,
                                         50, // minFastArcRadius,
                                         50, // minFastArcLength,
                                         25, // travelSpeedLinear,
                                         30, // travelSpeedLinearFast,
                                         10, // travelSpeedLinearBack,
                                         50, // minFastLinearLength
                                         false
                                        );
        driver.setTrajectoryEvaluator(navTraj);
    }


@Override  
    public String getDescription() {
        return "Robot controller "+getClass().getName()+" navigation/trajectoryplanning/driving demo (no configuration accepted)";
    }


@Override  
    public boolean requiresConfiguration() {
        return false;
    }


@Override  
    public void run() throws Exception {

        Robot.debugOut.println("Instantiate Motion Planning...");

        instantiateMotionPlanning();   // So aufwändig, dass in eigene Methode ausgelagert

        Robot.debugOut.println("Motion Planning instantiated");

        Robot.debugOut.println("Startup Robot facilities...");


        // Lidar-System einrichten und starten 

        if (Robot.lidarSubsystem==null) {
            Robot.debugOut.println("Navigation without Lidar Subsystem not possible!");
            return;
        }

        try {
            Robot.lidarSubsystem.resetWorldModel();
        }
        catch (UnsupportedOperationException e) {
            Robot.debugOut.println("Lidar Subsystem does not use a World Model");
        }

        Robot.lidarSubsystem.startup();


        // MSS konfigurieren

        Robot.motionSubsystem.sendCommand("stoprule T");
        Robot.motionSubsystem.sendCommand("rotaterule T");


        Robot.debugOut.println("Robot facilities started");


        Time.sleep(2000);    // 2s warten, damit 
                             // - der Lidar hochgefahren ist (Rotation hat Endgeschwindigkeit)
                             // - es ein paar Lidar-Scans gibt und damit die Scans schon richtig ausgerichtet auf der Karte liegen
                             // - das MSS die ersten Positionen gemeldet hat

        // Demo-Ziel
        targetX=100;
        targetY=100;

        TrajectorySequence firstRouteTraj=computeRouteAndTrajectories();
        if (firstRouteTraj==null) return;

        driver.drive(firstRouteTraj);

        while (isRunning() && !targetReached()) {
            Time.sleep(100);

            if (driver.isHalted()) {  // Es gibt keine Trajectorie, z.B. es wurde erkannt, dass die aktuelle Trajektorien nicht mehr befahrbar ist -> neu Routen

                if (targetReached()) continue;

                // Neue Bahn zum Ziel berechnen
                TrajectorySequence routeTraj=computeRouteAndTrajectories();

                if (routeTraj==null) 
                    return;

                // Driver die Bahn fahren lassen

                driver.drive(routeTraj);
            }
        }
    }


@Override 
    public void pause() throws Exception {
        if (Robot.lidarSubsystem!=null) {
            Robot.lidarSubsystem.shutdown();
        }
        if (driver!=null)
            driver.halt(Driver.HALT_REASON_CONTROLLER_PAUSE);
    }


@Override  
    public void stop() throws Exception {
        if (Robot.lidarSubsystem!=null) {
            Robot.lidarSubsystem.shutdown();
        }

        if (driver!=null)
            driver.halt(Driver.HALT_REASON_CONTROLLER_STOP);

        obstaclesLidar.clear();
    }


@Override  
    public void mssResponse(ArrayList<String> messages,int responseType) throws Exception {
        if (MotionSubsystemListener.isFailureResponse(responseType))
             Robot.debugOut.println("Failure response "+messages.get(0));
    }


@Override  
    public void mssAsyncMessages(ArrayList<String> messages,AsyncMotionMessageBundle bundle) throws Exception {
        if (bundle.containsPos()) {
            posXMSS=bundle.getDouble(AsyncMotionMessage.X);
            posYMSS=bundle.getDouble(AsyncMotionMessage.Y);
            posAngMSS=bundle.getDouble(AsyncMotionMessage.ANG);           
        }

        if (bundle.containsType(AsyncMotionMessage.STOPPED)) {
            driver.mssStoppedReceived();
        }
    }


@Override  
    public void observedLidarPointsSlam(LidarPackageSlam lidarPackageSlam) throws Exception {
        if (!lidarPackageSlam.isSuccessful())  // Lidar Scan war nicht erfolgreich -> Nichts tun
            return;

        if (driver!=null && !driver.isHalted() && navTraj.hasRouted()) {

            // Lidar-Punkte in die Hindernis-Karte eintragen

            PointList2D<ObstaclePoint> newObstacles=obstaclesLidar.addLidarPoints(lidarPackageSlam,1.0d);


            // Neu gefundene Hindernisse an die Navigations-Instanz melden

            if (!navTraj.addObstacles(newObstacles.getAll2D())) {   // Hinzufügen der neuen Punkte, bei false ging das nicht, da z.B. der grid zu klein war
                driver.halt(Driver.HALT_REASON_NAVIGATION,"Cannot add new obstacles to navigation world model");  // Bei Fehlschlag neu-Routing initiieren
                return;       
            }


            // Schauen, ob die aktuelle Bahn noch hindenisfrei befahrbar ist

            TrajectorySequence plannedTraj=driver.plannedTrajectories();
            if (plannedTraj!=null) {
                double[] collision=navTraj.trajectoryCollision(plannedTraj);

                if (collision!=null) {  // Die aktuelle Trajektorie oder die restliche Sequenz ist nicht frei
                    driver.halt(Driver.HALT_REASON_NAVIGATION,"Planned route goes through obstacle");
                }
            }
        }
    }


    // Route und Bahn planen
    private TrajectorySequence computeRouteAndTrajectories() throws Exception {

        RouteTraj routeTraj=navTraj.getRouteTraj(posXMSS,posYMSS,RobotGeomUtil.mssAngle2NavAngle(posAngMSS),Double.NaN,
                                                 targetX,targetY,Double.NaN,Double.NaN,
                                                 ObstacleContainer.getObstactles2D(obstaclesLidar));

        if (routeTraj.getRoute()==null) {
            Robot.debugOut.println("No route found!");
            return null;
        }

        TrajectorySequence trajSeq=routeTraj.getTrajectories();
        if (trajSeq==null) {
            Robot.debugOut.println("No trajectories found");
        }

        return trajSeq;
    }


    private boolean targetReached() {
        return (Math.hypot(posXMSS-targetX,posYMSS-targetY)<5);   // 5cm Distanz zum Ziel ist erlaubt
    }

}
