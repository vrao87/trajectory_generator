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

static inline double quinticBlend(double s) {
    double s2 = s*s;
    double s3 = s2*s;
    double s4 = s3*s;
    double s5 = s4*s;
    return 10.0*s3 - 15.0*s4 + 6.0*s5;
}

class TrajectoryGenerator {
public:
    TrajectoryGenerator(double x0, double y0, double heading0,
                        double x1, double y1, double heading1)
        : x0_(x0), y0_(y0), heading0_(heading0),
          x1_(x1), y1_(y1), heading1_(heading1) {}

    void addObstacle(double x, double y, double width) {
        obs_x_ = x;
        obs_y_ = y;
        obs_w_ = width;
        has_obstacle_ = true;
    }

    std::vector<TrajectoryPoint> generate(int num_points = 200) {
        // Adjust y1 if obstacle present
        double y1_adj = y1_;
        if (has_obstacle_) {
            // if the obstacle is between x0 and x1, adjust end y
            if (obs_x_ > x0_ && obs_x_ < x1_) {
                // shift up or down depending on start y position
                if (y0_ <= obs_y_) {
                    y1_adj = obs_y_ + obs_w_/2.0 + safety_margin_;
                } else {
                    y1_adj = obs_y_ - (obs_w_/2.0 + safety_margin_);
                }
            }
        }

        std::vector<TrajectoryPoint> traj;
        traj.reserve(num_points+1);

        double L = x1_ - x0_;
        for (int i = 0; i <= num_points; i++) {
            double x = x0_ + (L * i / num_points);
            double s = (x - x0_) / L; // [0,1]
            double y = y0_ + (y1_adj - y0_) * quinticBlend(s);
            traj.push_back({x, y});
        }
        return traj;
    }

private:
    double x0_, y0_, heading0_;
    double x1_, y1_, heading1_;
    double obs_x_ = 0.0, obs_y_ = 0.0, obs_w_ = 0.0;
    bool has_obstacle_ = false;
    const double safety_margin_ = 0.5; // extra clearance
};

int main() {
    // Start and end points
    double x0 = 0.0, y0 = 0.0, heading0 = 0.0;
    double x1 = 20.0, y1 = 0.0, heading1 = 0.0;

    TrajectoryGenerator gen(x0, y0, heading0, x1, y1, heading1);

    // Insert obstacle
    gen.addObstacle(10.0, 0.0, 4.2);

    auto traj = gen.generate(200);

    // Write to CSV
    std::ofstream file("trajectory.csv");
    file << "x,y\n";
    for (const auto& p : traj) {
        file << p.x << "," << p.y << "\n";
    }
    file.close();

    std::cout << "Trajectory written to trajectory.csv\n";
    return 0;
}
