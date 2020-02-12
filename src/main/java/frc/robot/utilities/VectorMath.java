/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

/**
 * Add your docs here.
 */
public class VectorMath {
    public static double[] add(double[] u, double[] v) {
        double[] r = new double[u.length];
        for (int i = 0; i < u.length; i++) {
            r[i] = u[i] + v[i];
        }
        return r;
    }
    public static double[] sub(double[] u, double[] v) {
        double[] r = new double[u.length];
        for (int i = 0; i < u.length; i++) {
            r[i] = u[i] - v[i];
        }
        return r;
    }
    public static double[] vector(double x, double y) {
        return new double[] {x,y};
    }
    public static double dot(double[] u, double[] v) {
        double r = 0;
        for (int i = 0; i < u.length; i++) r += u[i] * v[i];
        return r;
    }
    public static double length(double[] u) {
        return Math.sqrt(dot(u,u));
    }
    public static double[] smul(double s, double[] u) {
        double[] r = new double[u.length];
        for (int i = 0; i < u.length; i++) r[i] = s*u[i];
        return r;
    }
    public static double L1norm(double[] u) {
        double r = 0;
        for (int i = 0; i < u.length; i++) {
            double a = Math.abs(u[i]);
            if (a > r) r = a;
        }
        return r;
    }
    public static double[] normalize(double[] u) {
        return smul(1/length(u), u);
    }
    public static double[] rotate(double theta, double[] u) {
        theta = Math.toRadians(theta);
        double s = Math.sin(theta);
        double c = Math.cos(theta);
        return new double[] {c*u[0] - s*u[1], s*u[0]+c*u[1]};
    }
    public static double[][] getLine(double[] x0, double[] x1) {
        double[] v = sub(x1, x0);
        return new double[][] {x0, rotate(90, normalize(v))};
    }
    public static double distanceOffLine(double[][] line, double[] p) {
        return dot(sub(p, line[0]), line[1]);
    }

    public static double[] matrixmult(double[][] mat, double[] u) {
        double[] r = new double[mat.length];
        for (int i = 0; i < mat.length; i++) r[i] = dot(mat[i], u);
        return r;
    }

    public static double avg(double[] u) {
        double sum = 0;
        for (int i = 0; i < u.length; i++) sum += u[i];
        return sum/u.length;
    }

    public static double angle(double[] src, double[] dest) {
        double[] diff = sub(src, dest);
        return Math.atan2(diff[1], diff[0]);
    }

    public static double[] displacement(double dist, double angle) {
        angle = Math.toRadians(angle);
        return new double[] {dist*Math.cos(angle), dist*Math.sin(angle)};
    }

    public static double normalizeAngle(double angle, double cutpoint) {
        while (angle > cutpoint) angle -= 360;
        while (angle < cutpoint-360) angle += 360;
        return angle;
    }

    public static double max(double[] u) {
        double max = u[0];
        for (int i = 0; i < u.length; i++) {
            if (u[i] > max) max = u[i];
        }
        return max;
    }
    public static double min(double[] u) {
        double min = u[0];
        for (int i = 0; i < u.length; i++) {
            if (u[i] < min) min = u[i];
        }
        return min;
    }

    public static double mod(double x, double n) {
        while(x > n) x -= n;
        while(x < 0) x += n;
        return x;
    }
}
