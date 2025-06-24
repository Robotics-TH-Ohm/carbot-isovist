package isovist.model;

import java.util.ArrayList;

import robotinterface.Robot;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.lss.ObservedLidarPointSlam;
import robotinterface.debug.DebugPainterOverlay;

import basics.util.ColorUtil;
import basics.points.Point;
import basics.points.PointList2D;
import basics.points.PointCloud2D;
import basics.points.container.GridPointCloud2D;
import basics.points.container.ArrayPointList;

import isovist.model.features.Feature;
import isovist.model.features.FeatureUtils;

public class Isovist {
	private double[] pos;
	private PointList2D<Point> points;
	private double[] featureVec = new double[features.length];

	public Isovist(PointList2D<Point> points, double[] pos) {
		this.pos = pos;
		this.points = points;
		calculateFeatures();
	}

	public Isovist(double[] featureVec) {
		this.featureVec = featureVec;
	}

	public void paint(DebugPainterOverlay overlay, String hexColor) {
		int[] rgb = ColorUtil.hex2col(hexColor);
		overlay.clear();
		overlay.fillPoly(points.getAll2D(), rgb[0], rgb[1], rgb[2], 100);
		overlay.paint();
	}

	public static final Feature[] features  = new Feature[] {
		Feature.area,
		Feature.perimeter,
		Feature.compactness,
		Feature.drift
	};

	private void calculateFeatures() {
		for (int i = 0; i < features.length; ++i) {
			Feature feat = features[i];
			featureVec[i] = feat.calculate(points, pos);
		}
	}

	public void normalizeFeatures(double[] min, double[] max) {
		for (int i = 0; i < featureVec.length; ++i) {
			featureVec[i] = FeatureUtils.normalize(featureVec[i], min[i], max[i]);
		}
	}

	public double[] getFeatures() {
		return this.featureVec;
	}

	public double[] getPos() {
		return this.pos;
	}

	public double distanceTo(Isovist isovist) {
		if (featureVec.length != isovist.featureVec.length) return -1;

		double sum = 0;
		for (int i = 0; i < featureVec.length; ++i) {
			sum += Math.pow(featureVec[i] - isovist.featureVec[i], 2);
		}
		return Math.sqrt(sum);
	}

	////////////////////////////////////////////////////////////////////////////////

	public static final int MAX_DIST = 600;
	public static final int RAY_COUNT = 400;
	public static final int RADIUS = 8;

	public static PointList2D<Point> samplePointsFromCloud(PointCloud2D<Point> cloud, double[] pos) {
		PointList2D<Point> points = new ArrayPointList(RAY_COUNT);
		
		for (int i = 0; i < RAY_COUNT; ++i) {
			double theta = ((float)i / RAY_COUNT) * 2 * Math.PI;
			double dx = pos[0] + Math.cos(theta) * MAX_DIST;
			double dy = pos[1] + Math.sin(theta) * MAX_DIST;

			PointList2D<Point> ps = cloud.getInsideBeam(pos[0], pos[1], dx, dy, RADIUS);
			Point p = null;

			if (ps.size() > 0) {
				// Point found in beam!
				p = ps.getKNearest(pos[0], pos[1], Double.POSITIVE_INFINITY, 1).get(0);
			} else {
				// No point, assume maximum viewing distance
				p = new Point(dx, dy);
			}

			points.add(p);
		}

		return points;
	}

	public static PointList2D<Point> samplePointsFromLidar(LidarPackageSlam lidarPackage) {
		ArrayList<ObservedLidarPointSlam> lidarPoints = lidarPackage.observedPoints;

		PointList2D<Point> points = new ArrayPointList<>(lidarPoints.size());
		for (ObservedLidarPointSlam p : lidarPoints)
			points.addIfNotNull(p.isValid() ? new Point(p.x, p.y) : null);

		GridPointCloud2D<Point> lidarCloud = new GridPointCloud2D(10, points);
		return samplePointsFromCloud(lidarCloud, new double[] { lidarPackage.observationPosX, lidarPackage.observationPosY });
	}
}
