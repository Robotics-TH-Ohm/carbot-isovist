package robotcontroller.model.features;

import java.util.Arrays;

import robotinterface.debug.DebugPainterOverlay;
import robotinterface.Robot;
import basics.points.container.ArrayPointList;

import basics.points.Point;
import basics.points.PointList2D;

public class MomentCalculator {

	private static final double EPS = 1e-5;
	public static final int STEP = 16;

	public static double[] computeMomentValues(PointList2D<Point> points, double[] pos) {
		if (points.size() < 3) return null;
		double[] _pos = pos == null ? new double[] { 0, 0 } : pos;

		int tris = 0;
		double a1Sum = 0;
		double a2Sum = 0;
		double a3Sum = 0;

		DebugPainterOverlay overlay = Robot.debugPainter.getOverlay("Raw LiDAR Isovist");
		PointList2D<Point> paintPoints = new ArrayPointList<Point>(points.size() / STEP);
		overlay.clear();

		for (int i = 0; i < points.size(); i += STEP) {
			Point cur = points.get(i);
			Point next = (i + STEP >= points.size()) ? points.get(points.size() - i + STEP) : points.get(i + STEP);
			paintPoints.add(new Point(cur));

			double a = cur.distanceTo2D(_pos[0], _pos[1]);
			double b = next.distanceTo2D(_pos[0], _pos[1]);
			double c = cur.distanceTo2D(next);

			// Normalize triangle lengths
			double max = 600.0; // Math.max(Math.max(a, b), c);
			a /= max;
			b /= max;
			c /= max;

			if (a == 0 || b == 0 || c == 0) continue;

			double[] angles = computeAngles(a, b, c);
			double alpha = angles[0];
			double beta = angles[1];
			double gamma = angles[2];

			if (Double.isNaN(alpha) || Double.isNaN(beta) || Double.isNaN(gamma)) continue;

			double _a1 = computeA1(a, b, c, gamma);
			double _a2 = computeA2(a, b, c, alpha, beta, gamma);
			double _a3 = computeA3(a, b, c, alpha, beta, gamma);

			// DEBUGGING FOR TRIANGLES
			// System.out.println("Calculating triangle at index " + i);
			// System.out.println("Lengths: " + a + " | " + b + " | " + c);
			// System.out.println("Angles: " + Arrays.toString(angles));
			// System.out.println("cot(α): " + cot(angles[0]) + ", cot(β): " + cot(angles[1]));
			// System.out.println("cosec(α): " + cosec(angles[0]) + ", cosec(β): " + cosec(angles[1]));
			// System.out.println("a1: " + _a1 + " | a2: " + _a2 + " | a3: " + _a3);
			// System.out.println("A1: " + a1Sum + " | A2: " + a2Sum + " | A3: " + a3Sum);
			// System.out.println("-----");

			if (Double.isNaN(_a1) || Double.isNaN(_a2) || Double.isNaN(_a3)) continue;

			tris++;
			a1Sum += _a1;
			a2Sum += _a2;
			a3Sum += _a3;
		}

		// DEBUG: Draw triangle
		double[] paintPos = pos == null ? Robot.motionSubsystem.estimateCurrentPosition() : _pos;
		if (pos == null) {
			paintPoints.rotate0(-paintPos[2] * Math.PI/180);
			paintPoints.translate(paintPos[0], paintPos[1]);
		}
		for (Point p : paintPoints)
			overlay.drawLine(paintPos[0], paintPos[1], p.getX(), p.getY(), 0, 0, 0, 100);
		overlay.paint();

		// System.out.println("E_A1: " + a1Sum + " | E_A2: " + a2Sum + " | E_A3: " + a3Sum);
		double a1 = a1Sum / (2 * Math.PI);
		double a2 = a2Sum / (2 * Math.PI);
		double a3 = a3Sum / (2 * Math.PI);
		// System.out.println("A1: " + a1+ " | A2: " + a2+ " | A3: " + a3);

		double m1 = a1;
		double m2 = a2 - m1 * m1;
		double m3 = a3 - 3 * m1 * a2 + 2 * m1 * m1 * m1;

		double[] moments = new double[] { m1, m2, m3 };

		System.out.println("Moments: " + Arrays.toString(Arrays.stream(moments).mapToObj(v -> String.format("%.16f", v)).toArray())); // Adjust the precision as needed
		System.out.println("used tris: " + tris);
		System.out.println("===");

		return moments;
	}

	private static double[] computeAngles(double a, double b, double c) {
		double cosGamma = (a * a + b * b - c * c) / (2 * a * b);
		double cosAlpha = (b * b + c * c - a * a) / (2 * b * c);
		double cosBeta =	(a * a + c * c - b * b) / (2 * a * c);

		double gamma = Math.acos(clamp(cosGamma, -1.0, 1.0));
		double alpha = Math.acos(clamp(cosAlpha, -1.0, 1.0));
		double beta = Math.acos(clamp(cosBeta, -1.0, 1.0));

		return new double[] { alpha, beta, gamma };
	}

	private static double computeA1(double a, double b, double c, double gamma) {
		double _1st = a * b / c;
		double _2nd = Math.sin(gamma) / gamma;

		double cosGamma = Math.cos(gamma);
		double _3rdNum = (c + a - b * cosGamma) * (c + b - a * cosGamma);
		double _3rdDenom = a * b * Math.pow(Math.sin(gamma), 2);
		if (_3rdDenom < EPS || _3rdNum <= 0) return Double.NaN;
		double _3rd = Math.log(_3rdNum / _3rdDenom);

		return _1st * _2nd * _3rd;
	}

	private static double computeA2(double a, double b, double c, double alpha, double beta, double gamma) {
		double _1st = 1 / gamma;
		double _2nd = Math.pow(a * b * Math.sin(gamma) / c, 2);
		double cotAlpha = cot(alpha);
		double cotBeta = cot(beta);
		if (Double.isNaN(cotAlpha) || Double.isNaN(cotBeta)) return Double.NaN;
		double _3rd = cotAlpha + cotBeta;

		return _1st * _2nd * _3rd;
	}

	private static double computeA3(double a, double b, double c, double alpha, double beta, double gamma) {
		double _1st = 1 / (2 * gamma);
		double _2nd = Math.pow(a * b * Math.sin(gamma) / c, 3);

		double cosecAlpha = cosec(alpha);
		double cotAlpha = cot(alpha);
		double cosecBeta = cosec(beta);
		double cotBeta = cot(beta);

		if (Double.isNaN(cosecAlpha) || Double.isNaN(cosecBeta) ||
				Double.isNaN(cotAlpha) || Double.isNaN(cotBeta)) return Double.NaN;

		double _3rdLeft = cosecAlpha * cotAlpha + cosecBeta * cotBeta;
		double _3rdRight = Math.log((cosecAlpha + cotAlpha) * (cosecBeta + cotBeta));
		double _3rd = _3rdLeft + _3rdRight;

		return _1st * _2nd * _3rd;
	}

	private static double cot(double angle) {
		if (angle < 1e-3 || Math.abs(angle - Math.PI) < 1e-3) return Double.NaN;
		double tan = Math.tan(angle);
		return Math.abs(tan) < EPS ? Double.NaN : 1 / tan;
	}

	private static double cosec(double angle) {
		if (angle < 1e-3 || Math.abs(angle - Math.PI) < 1e-3) return Double.NaN;
		double sin = Math.sin(angle);
		return Math.abs(sin) < EPS ? Double.NaN : 1 / sin;
	}

	private static double clamp(double val, double min, double max) {
		return Math.max(min, Math.min(max, val));
	}
} 
