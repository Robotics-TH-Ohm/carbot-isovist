package isovist.model.features;

import java.util.function.*;
import basics.points.Point;
import basics.points.PointList2D;

public interface Feature {
	double calculate(PointList2D<Point> points, double[] pos);
	String getName();

	////////////////////////////////////////////////////////////////////////////////

	public static Point getPrev(PointList2D<Point> points, int i) {
		if (i == 0)
			return points.get(points.size() - 1);
		else
			return points.get(i - 1);
	}
	
	public static Point getNext(PointList2D<Point> points, int i) {
		if (i + 1 == points.size())
			return points.get(0);
		else
			return points.get(i + 1);
	}

	class DoubleFeature implements Feature
	{
		private BiFunction<PointList2D<Point>,double[],Double> func;
		private String name;

		public DoubleFeature(String name, BiFunction<PointList2D<Point>,double[],Double> func)
		{
			this.func = func;
			this.name = name;
		}

		public double calculate(PointList2D<Point> points, double[] pos) { return func.apply(points, pos); }
		public String getName() { return name; }
	}

	////////////////////////////////////////////////////////////////////////////////

	public static Feature area = new DoubleFeature("area", (points, pos) -> {
		if (points.size() < 3) return 0.0;

		double sum = 0;
		for (int i = 0; i < points.size(); ++i) {
			Point curr = points.get(i);
			Point next = getNext(points, i);
			Point prev = getPrev(points, i);

			sum += curr.getX() * (next.getY() - prev.getY());
		}

		return 0.5 * Math.abs(sum);
	});

	public static Feature perimeter = new DoubleFeature("perimeter", (points, pos) -> {
		if (points.size() < 2) return 0.0;

		double perimeter = 0;
		for (int i = 0; i < points.size(); ++i) {
			Point curr = points.get(i);
			Point next = getNext(points, i);
			
			perimeter += curr.distanceTo2D(next);
		}

		return perimeter;
	});

	public static Feature compactness = new DoubleFeature("compactness", (points, pos) -> {
		double perimeter = Feature.perimeter.calculate(points, pos);
		if (perimeter == 0) return 0.0;

		double area = Feature.area.calculate(points, pos);
		if (area == 0) return 0.0;

		return (4 * Math.PI * area) / (perimeter * perimeter);
	});

	public static Feature drift = new DoubleFeature("drift", (points, pos) -> {
		double area = Feature.area.calculate(points, pos);
		if (area == 0) return 0.0;

		Point centroid = FeatureUtils.centroid(points, area);
		if (centroid == null) return 0.0;

		return centroid.distanceTo2D(pos[0], pos[1]);
	});
}
