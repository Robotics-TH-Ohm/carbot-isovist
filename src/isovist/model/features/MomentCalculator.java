package isovist.model.features;

import robotinterface.debug.DebugPainterOverlay;
import robotinterface.Robot;

import basics.points.Point;
import basics.points.PointList2D;

public class MomentCalculator {

	private static final double EPS = 1e-5;

	public static double[] computeMomentValues(PointList2D<Point> points, double[] pos) {
		if (points.size() < 3) return null;

		double a1Sum = 0;
		double a2Sum = 0;
		double a3Sum = 0;

		int step = 25;
		DebugPainterOverlay overlay = Robot.debugPainter.getOverlay("Raw LiDAR Isovist");
		overlay.clear();

		for (int i = 0; i < points.size(); i += step) {
			Point cur = points.get(i);
			Point next = (i + step >= points.size()) ? points.get(points.size() - i - step) : points.get(i + step);

			double a = cur.distanceTo2D(pos[0], pos[1]);
			double b = next.distanceTo2D(pos[0], pos[1]);
			double c = cur.distanceTo2D(next);

			overlay.drawLine(0, 0, cur.getX(), cur.getY(), 0, 0, 0, 100);
			overlay.drawLine(0, 0, next.getX(), next.getY(), 255, 0, 0, 100);

			if (a == 0 || b == 0 || c == 0) continue;

			double[] angles = computeAngles(a, b, c);
			double alpha = angles[0];
			double beta = angles[1];
			double gamma = angles[2];

			if (Double.isNaN(alpha) || Double.isNaN(beta) || Double.isNaN(gamma)) continue;

			double angleSum = alpha + beta + gamma;
			if (Math.abs(angleSum - Math.PI) > 0.01) continue;

			Double _a1 = safeComputeA1(a, b, c, gamma);
			Double _a2 = safeComputeA2(a, b, c, alpha, beta, gamma);
			Double _a3 = safeComputeA3(a, b, c, alpha, beta, gamma);

			if (_a1 == null || _a2 == null || _a3 == null) continue;

			if (_a1 > 1e6 || _a2 > 1e6 || _a3 > 1e6) {
				System.out.printf("[DEBUG] Huge moments at i=%d\n", i);
				System.out.printf("Lengths: a=%.3f, b=%.3f, c=%.3f\n", a, b, c);
				System.out.printf("Angles: alpha=%.5f, beta=%.5f, gamma=%.5f\n", alpha, beta, gamma);
				System.out.printf("Moments: A1=%.5e, A2=%.5e, A3=%.5e\n\n", _a1, _a2, _a3);
			}

			a1Sum += _a1;
			a2Sum += _a2;
			a3Sum += _a3;
		}
		overlay.paint();

		double a1 = a1Sum / (2 * Math.PI);
		double a2 = a2Sum / (2 * Math.PI);
		double a3 = a3Sum / (2 * Math.PI);

		double m1 = a1;
		double m2 = a2 - m1 * m1;
		double m3 = a3 - 3 * m1 * a2 + 2 * m1 * m1 * m1;

		return new double[] { m1, m2, m3 };
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

	private static Double safeComputeA1(double a, double b, double c, double gamma) {
		try {
			double _1st = a * b / c;
			double _2nd = Math.sin(gamma) / gamma;

			double cosGamma = Math.cos(gamma);
			double _3rdNum = (c + a - b * cosGamma) * (c + b - a * cosGamma);
			double _3rdDenom = a * b * Math.pow(Math.sin(gamma), 2);

			if (_3rdDenom < EPS || _3rdNum <= 0) return null;
			double _3rd = Math.log(_3rdNum / _3rdDenom);

			double result = _1st * _2nd * _3rd;
			return (Math.abs(result) > 1e6) ? null : result;
		} catch (Exception e) {
			return null;
		}
	}

	private static Double safeComputeA2(double a, double b, double c, double alpha, double beta, double gamma) {
		try {
			double _1st = 1 / gamma;
			double _2nd = Math.pow(a * b * Math.sin(gamma) / c, 2);

			double cotAlpha = cot(alpha);
			double cotBeta = cot(beta);
			if (Double.isNaN(cotAlpha) || Double.isNaN(cotBeta)) return null;
			double _3rd = cotAlpha + cotBeta;

			double result = _1st * _2nd * _3rd;
			return (Math.abs(result) > 1e6) ? null : result;
		} catch (Exception e) {
			return null;
		}
	}

	private static Double safeComputeA3(double a, double b, double c, double alpha, double beta, double gamma) {
		try {
			double _1st = 1 / (2 * gamma);
			double _2nd = Math.pow(a * b * Math.sin(gamma) / c, 3);

			double cosecAlpha = cosec(alpha);
			double cotAlpha = cot(alpha);
			double cosecBeta = cosec(beta);
			double cotBeta = cot(beta);

			if (Double.isNaN(cosecAlpha) || Double.isNaN(cosecBeta) ||
					Double.isNaN(cotAlpha) || Double.isNaN(cotBeta)) return null;

			double sum1 = cosecAlpha + cotAlpha;
			double sum2 = cosecBeta + cotBeta;
			if (sum1 * sum2 <= 0) return null; // log invalid

			double _3rdLeft = cosecAlpha * cotAlpha + cosecBeta * cotBeta;
			double _3rdRight = Math.log(sum1 * sum2);
			double _3rd = _3rdLeft + _3rdRight;

			double result = _1st * _2nd * _3rd;
			return (Math.abs(result) > 1e6) ? null : result;
		} catch (Exception e) {
			return null;
		}
	}

	private static double cot(double angle) {
		double tan = Math.tan(angle);
		if (Math.abs(tan) < EPS) return Double.NaN;
		double result = 1 / tan;
		return Math.abs(result) > 1e5 ? Double.NaN : result;
	}

	private static double cosec(double angle) {
		double sin = Math.sin(angle);
		if (Math.abs(sin) < EPS) return Double.NaN;
		double result = 1 / sin;
		return Math.abs(result) > 1e5 ? Double.NaN : result;
	}

	private static double clamp(double val, double min, double max) {
		return Math.max(min, Math.min(max, val));
	}
} 
