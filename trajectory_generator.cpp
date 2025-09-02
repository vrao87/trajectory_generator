#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct TrajectoryPoint {
    double x;
    double y;
};

// Wrapper for boundary conditions
struct TrajectoryConstraints {
    double x;
    double y;
    double heading;   // in radians
};

class QuinticTrajectory {
public:
    QuinticTrajectory(const TrajectoryConstraints& start,
                      const TrajectoryConstraints& end)
        : start_(start), end_(end) {}

    void addObstacle(double x, double y, double width) {
        obs_x_ = x;
        obs_y_ = y;
        obs_w_ = width;
        has_obstacle_ = true;
    }

    std::vector<TrajectoryPoint> generate(int num_points = 200) {
        // Adjust target endpoint if obstacle lies between start & end
        double y1_adj = end_.y;
        if (has_obstacle_ && (obs_x_ > start_.x && obs_x_ < end_.x)) {
            if (start_.y <= obs_y_) {
                y1_adj = obs_y_ + obs_w_ / 2.0 + safety_margin_;
            } else {
                y1_adj = obs_y_ - (obs_w_ / 2.0 + safety_margin_);
            }
        }

        computeCoefficients(end_.x, end_.y, y1_adj);

        std::vector<TrajectoryPoint> traj;
        traj.reserve(num_points + 1);

        double L = end_.x - start_.x;
        for (int i = 0; i <= num_points; i++) {
            double s = static_cast<double>(i) / num_points;
            double y = evalPoly(s);
            double x = start_.x + L * s;
            traj.push_back({x, y});
        }
        return traj;
    }

private:
    TrajectoryConstraints start_, end_;
    double a0_, a1_, a2_, a3_, a4_, a5_;
    double obs_x_ = 0.0, obs_y_ = 0.0, obs_w_ = 0.0;
    bool has_obstacle_ = false;
    const double safety_margin_ = 0.5;

    void computeCoefficients(double x1, double y1_orig, double y1_adj) {
        double L = x1 - start_.x;

        // Slopes from headings
        double yp0 = std::tan(start_.heading);
        double yp1 = std::tan(end_.heading);

        // Normalize to s = (x-x0)/L
        double y0s = start_.y;
        double y1s = y1_adj;
        double yp0s = yp0 * L;
        double yp1s = yp1 * L;
        double ypp0s = 0.0; // assume zero curvature
        double ypp1s = 0.0;

        // Coeffs at s=0
        a0_ = y0s;
        a1_ = yp0s;
        a2_ = 0.5 * ypp0s;

        // Solve for a3,a4,a5 with s=1 boundary
        double S1 = y1s - (a0_ + a1_ + a2_);
        double S2 = yp1s - (a1_ + 2 * a2_);
        double S3 = ypp1s - (2 * a2_);

        a3_ = 10 * S1 - 4 * S2 + 0.5 * S3;
        a4_ = -15 * S1 + 7 * S2 - S3;
        a5_ = 6 * S1 - 3 * S2 + 0.5 * S3;
    }

    double evalPoly(double s) {
        return a0_ + a1_ * s + a2_ * s * s + a3_ * s * s * s
               + a4_ * s * s * s * s + a5_ * s * s * s * s * s;
    }
};

int main() {
    TrajectoryConstraints start{0.0, 0.0, 0.0};     // x, y, heading (rad)
    TrajectoryConstraints end{20.0, 0.0, 0.2};      // x, y, heading (rad)

    QuinticTrajectory gen(start, end);

    // Insert obstacle
    gen.addObstacle(10.0, 0.0, 4.2);

    auto traj = gen.generate(200);

    std::ofstream file("trajectory.csv");
    file << "x,y\n";
    for (const auto& p : traj) {
        file << p.x << "," << p.y << "\n";
    }
    file.close();

    std::cout << "Trajectory written to trajectory.csv\n";
    return 0;
}
