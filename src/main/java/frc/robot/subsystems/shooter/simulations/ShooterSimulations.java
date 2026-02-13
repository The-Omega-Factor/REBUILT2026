package frc.robot.subsystems.shooter.simulations;

import java.util.ArrayList;
import java.util.List;


public class ShooterSimulations {
    static class Point {
        double x;
        double y;

        Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    static class PointAndSpeed {
        double x;
        double y;
        double v0;

        PointAndSpeed(double x, double y, double v0) {
            this.x = x;
            this.y = y;
            this.v0 = v0;
        }

        @Override
        public String toString() {
            return "(" + x + ", " + y + ", " + v0 + ")";
        }
    }

    public static List<Point> simulate(
        double mass,
        double dragCoefficient,
        double area,
        double v0, 
        double theta,
        double airDensity,
        double gravity,
        double dt 
    ) {
        theta = Math.toRadians(theta);

        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);

        double x = 0;
        double y = 0;

        double k = airDensity * dragCoefficient * area / (2 * mass);

        List<Point> trajectory = new ArrayList<>();
        trajectory.add(new Point(x, y));

        while (true) {

        double v = Math.sqrt(vx * vx + vy * vy);

        double ax = -k * v * vx;
        double ay = -gravity - k * v * vy;

        double prevX = x;
        double prevY = y;
        double prevVy = vy;

        vx += ax * dt;
        vy += ay * dt;

        x += vx * dt;
        y += vy * dt;

        // Check: crossing y = 4 meters on the way down
        if (prevY > 4 && y <= 4 && prevVy < 0) {

            // Linear interpolation for better accuracy
            double frac = (prevY - 4) / (prevY - y);

            double impactX = prevX + frac * (x - prevX);
            double impactY = 4;

            trajectory.add(new Point(impactX, impactY));
            break;
        }

        trajectory.add(new Point(x, y));

        // Safety: stop if it hits ground before reaching 4
            if (y < 0) {
                break;
            }
        }

        return trajectory;
    }

    public static void main(String[] args) {

        double m = 0.215;       // kg
        double Cd = 0.6;       // sphere
        double A = 0.01767;      // m^2
        double angle = 60;      // degrees

        double rho = 1.225;     // air density
        double g = 9.81;        // gravity
        double dt = 0.001;      // timestep

        List<PointAndSpeed> rawSpeedList = new ArrayList<PointAndSpeed>();
        double minimumSpeed = 0.0;

        for (double v0 = 0; v0 < 50; v0 += 0.01) {
            List<Point> path = simulate(m, Cd, A, v0, angle, rho, g, dt);

            double x = path.get(path.size() - 1).x;
            double y = path.get(path.size() - 1).y;

            rawSpeedList.add(new PointAndSpeed(x, y, v0));
        }

        for (int i = 0; i < rawSpeedList.size(); i += 50) {
            System.out.println(rawSpeedList.get(i).toString());
        }

        System.out.println(minimumSpeed);
        System.out.println();

        List<Point> path = simulate(m, Cd, A, 2, angle, rho, g, dt);

        for (int i = 0; i < path.size(); i += 10) {
            System.out.println(path.get(i).y);
        }
    }
}
