#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>

struct TrajectoryPoint {
    double x;
    double y;
};

struct TrajectoryConstraints {
    double x;
    double y;
    double heading;    // radians
    double curvature;  // optional (set NAN if unused)
};

struct Obstacle {
    double x;
    double y;
    double width;
};

class TrajectoryGenerator {
public:
    void setStart(const TrajectoryConstraints& start) { start_ = start; }
    void setEnd(const TrajectoryConstraints& end) { end_ = end; }
    void setObstacle(const Obstacle& obs) { obs_ = obs; has_obstacle_ = true; }

    std::vector<TrajectoryPoint> generate(int num_points = 200) {
        double x0 = start_.x, y0 = start_.y;
        double x1 = end_.x,   y1 = end_.y;

        // Adjust target endpoint if obstacle is present
        if (has_obstacle_ && obs_.x > x0 && obs_.x < x1) {
            if (y0 <= obs_.y) {
                y1 = obs_.y + obs_.width / 2.0 + safety_margin_;
            } else {
                y1 = obs_.y - (obs_.width / 2.0 + safety_margin_);
            }
        }

        double L = x1 - x0;

        // Slopes (first derivative at boundaries)
        double dy0 = std::tan(start_.heading);
        double dy1 = std::tan(end_.heading);

        // Curvatures -> convert to second derivative
        double ddy0 = 0.0;
        double ddy1 = 0.0;
        if (!std::isnan(start_.curvature)) {
            ddy0 = start_.curvature * std::pow(1 + dy0 * dy0, 1.5);
        }
        if (!std::isnan(end_.curvature)) {
            ddy1 = end_.curvature * std::pow(1 + dy1 * dy1, 1.5);
        }

        // Scale derivatives into polynomial constraints
        double p0 = y0;
        double p1 = y1;
        double v0 = dy0 * L;
        double v1 = dy1 * L;
        double a0 = ddy0 * L * L;
        double a1 = ddy1 * L * L;

        // Solve quintic polynomial coefficients
        double c0 = p0;
        double c1 = v0;
        double c2 = a0 / 2.0;

        double rhs1 = p1 - (c0 + c1 + c2);
        double rhs2 = v1 - (c1 + 2 * c2);
        double rhs3 = a1 - 2 * c2;

        double A[3][3] = {
            {1, 1, 1},
            {3, 4, 5},
            {6, 12, 20}
        };
        double B[3] = {rhs1, rhs2, rhs3};
        double C[3];

        // Gaussian elimination
        for (int i = 0; i < 3; i++) {
            double pivot = A[i][i];
            for (int j = 0; j < 3; j++) A[i][j] /= pivot;
            B[i] /= pivot;
            for (int k = 0; k < 3; k++) {
                if (k == i) continue;
                double factor = A[k][i];
                for (int j = 0; j < 3; j++) A[k][j] -= factor * A[i][j];
                B[k] -= factor * B[i];
            }
        }
        for (int i = 0; i < 3; i++) C[i] = B[i];

        double c3 = C[0];
        double c4 = C[1];
        double c5 = C[2];

        // Generate trajectory points
        std::vector<TrajectoryPoint> traj;
        traj.reserve(num_points + 1);
        for (int i = 0; i <= num_points; i++) {
            double s = static_cast<double>(i) / num_points;
            double y = c0 + c1 * s + c2 * s * s +
                       c3 * s * s * s + c4 * s * s * s * s +
                       c5 * s * s * s * s * s;
            double x = x0 + s * L;
            traj.push_back({x, y});
        }
        return traj;
    }

private:
    TrajectoryConstraints start_{}, end_{};
    Obstacle obs_{};
    bool has_obstacle_ = false;
    const double safety_margin_ = 0.5;
};

int main() {
    TrajectoryGenerator gen;

    // Start & End conditions with curvature
    TrajectoryConstraints start{0.0, 0.0, 0.2, 0.1};       // curvature = 0
    TrajectoryConstraints end{20.0, 0.0, 0.1, 0.0};        // curvature = 0
    gen.setStart(start);
    gen.setEnd(end);

    // Add obstacle
    Obstacle obs{10.0, 0.0, 4.2};
    gen.setObstacle(obs);

    auto traj = gen.generate(200);

    // Save CSV
    std::ofstream file("trajectory.csv");
    file << "x,y\n";
    for (const auto& p : traj) {
        file << p.x << "," << p.y << "\n";
    }
    file.close();

    std::cout << "Trajectory written to trajectory.csv\n";
    return 0;
}
