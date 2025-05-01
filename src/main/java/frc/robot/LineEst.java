package frc.robot;

import java.util.ArrayList;
import java.util.function.Function;

// https://www.mathsisfun.com/data/least-squares-regression.html
public class LineEst {
  public record Line(double m, double b) {}

  public static Line estimate(ArrayList<Double> x, ArrayList<Double> y) {
    final double[] xArray = new double[x.size()];
    final double[] yArray = new double[y.size()];
    for (int i = 0; i < x.size(); i++) {
      xArray[i] = x.get(i);
    }
    for (int i = 0; i < y.size(); i++) {
      yArray[i] = y.get(i);
    }
    return estimate(xArray, yArray);
  }

  public static Line estimate(double[] x, double[] y) {
    assert x.length == y.length;
    final int N = x.length;

    double sumX = sum(x);
    double sumY = sum(y);
    double sumX2 = sum(xi -> xi * xi, x);
    double sumXY = 0;
    for (int i = 0; i < N; i++) {
      sumXY += x[i] * y[i];
    }

    double m = (N * sumXY - sumX * sumY) / (N * sumX2 - sumX * sumX);
    double b = (sumY - m * sumX) / N;

    return new Line(m, b);
  }

  private static double sum(Function<Double, Double> map, double[] array) {
    double sum = 0;
    for (double value : array) {
      sum += map.apply(value);
    }
    return sum;
  }

  private static double sum(double[] array) {
    return sum(x -> x, array);
  }
}
