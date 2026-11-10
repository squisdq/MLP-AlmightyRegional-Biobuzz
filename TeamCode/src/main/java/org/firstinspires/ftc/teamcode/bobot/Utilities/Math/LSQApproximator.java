package org.firstinspires.ftc.teamcode.bobot.Utilities.Math;

import java.util.ArrayList;
import java.util.List;

public class LSQApproximator {
    private final double[] coefficients;

    private LSQApproximator(double[] coefficients) {
        this.coefficients = coefficients;
    }

    public static Builder builder() {
        return new Builder();
    }

    public double get(double distance) {
        double result = 0;
        for (int i = coefficients.length - 1; i >= 0; i--) result = result * distance + coefficients[i];
        return result;
    }

    public static class Builder {
        private final List<double[]> points = new ArrayList<>();
        private int degree = 1;

        public Builder add(double distance, double tof) {
            points.add(new double[]{distance, tof});
            return this;
        }

        public Builder degree(int degree) {
            this.degree = degree;
            return this;
        }

        public LSQApproximator build() {
            int n = degree + 1;
            if (points.stream().mapToDouble(p -> p[0]).distinct().count() < n)
                throw new IllegalStateException("TofApproximator: not enough points");
            double[][] a = new double[n][n + 1];
            for (double[] p : points)
                for (int i = 0; i < n; i++) {
                    for (int j = 0; j < n; j++) a[i][j] += Math.pow(p[0], i + j);
                    a[i][n] += Math.pow(p[0], i) * p[1];
                }
            for (int i = 0; i < n; i++)
                for (int r = i + 1; r < n; r++) {
                    double f = a[r][i] / a[i][i];
                    for (int c = i; c <= n; c++) a[r][c] -= f * a[i][c];
                }
            double[] x = new double[n];
            for (int i = n - 1; i >= 0; i--) {
                double sum = a[i][n];
                for (int j = i + 1; j < n; j++) sum -= a[i][j] * x[j];
                x[i] = sum / a[i][i];
            }
            return new LSQApproximator(x);
        }
    }
}