package org.firstinspires.ftc.teamcode.bobot.Utilities.Math;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BilinearLut {
    private final double[] xs;
    private final double[] ys;
    private final double[][] zs;
    private final boolean extrapolate;

    private BilinearLut(double[] xs, double[] ys, double[][] zs, boolean extrapolate) {
        this.xs = xs;
        this.ys = ys;
        this.zs = zs;
        this.extrapolate = extrapolate;
    }

    public static Builder builder() {
        return new Builder();
    }

    public double get(double x, double y) {
        if (!extrapolate) {
            x = Math.max(xs[0], Math.min(xs[xs.length - 1], x));
            y = Math.max(ys[0], Math.min(ys[ys.length - 1], y));
        }
        int i = 0;
        while (i < xs.length - 2 && x > xs[i + 1]) i++;
        int j = 0;
        while (j < ys.length - 2 && y > ys[j + 1]) j++;
        double tx = (x - xs[i]) / (xs[i + 1] - xs[i]);
        double ty = (y - ys[j]) / (ys[j + 1] - ys[j]);
        double bottom = zs[j][i] + tx * (zs[j][i + 1] - zs[j][i]);
        double top = zs[j + 1][i] + tx * (zs[j + 1][i + 1] - zs[j + 1][i]);
        return bottom + ty * (top - bottom);
    }

    public static class Builder {
        private final List<double[]> points = new ArrayList<>();
        private boolean extrapolate = false;

        public Builder add(double x, double y, double z) {
            points.add(new double[]{x, y, z});
            return this;
        }

        public Builder allowExtrapolation() {
            extrapolate = true;
            return this;
        }

        public BilinearLut build() {
            double[] xs = points.stream().mapToDouble(p -> p[0]).distinct().sorted().toArray();
            double[] ys = points.stream().mapToDouble(p -> p[1]).distinct().sorted().toArray();
            if (xs.length < 2 || ys.length < 2 || points.size() != xs.length * ys.length)
                throw new IllegalStateException("BilinearLut: grid incomplete");
            double[][] zs = new double[ys.length][xs.length];
            for (double[] p : points)
                zs[Arrays.binarySearch(ys, p[1])][Arrays.binarySearch(xs, p[0])] = p[2];
            return new BilinearLut(xs, ys, zs, extrapolate);
        }
    }
}