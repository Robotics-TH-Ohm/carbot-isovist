package robotcontroller.model.features;

import basics.points.Point;
import basics.points.PointList2D;

public class FeatureUtils {
	public static final double EPS = 1e-5;

	public static Point centroid(PointList2D<Point> points, double area) {
		if (area < EPS || points.size() < 3) return null;

		double sumX = 0, sumY = 0;

		for (int i = 0; i < points.size(); ++i) {
			Point curr = points.get(i);
			Point next = Feature.getNext(points, i);
			double product = curr.getX() * next.getY() - next.getX() * curr.getY();

			sumX += (curr.getX() + next.getX()) * product;
			sumY += (curr.getY() + next.getY()) * product;
		}

		return new Point(sumX / (6 * area), sumY / (6 * area));
	}

	public static double normalize(double value, double min, double max) {
		double denom = max - min;
		return Math.abs(denom) < EPS ? 0 : (value - min) / denom;
	}

	public static boolean isEPS(double v) { return v < EPS; }
	public static double withEPS(double v) { return isEPS(v) ? Double.NaN : v; }
	public static double withEPSZero(double v) { return isEPS(v) ? 0 : v; }
	public static double clamp(double v, double min, double max) { return Math.max(min, Math.min(v, max)); }
}
