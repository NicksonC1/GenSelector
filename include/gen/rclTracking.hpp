#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include "gen/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"

namespace gen {

enum class RclTimeUnit { SECOND, MILLISECOND };

class RclTimer {
    public:
        explicit RclTimer(double timeoutMs_ = 0)
            : timeoutMs(timeoutMs_),
              startTime(std::chrono::high_resolution_clock::now()) {}

        void reset() { startTime = std::chrono::high_resolution_clock::now(); }

        void hardReset(double newTimeoutMs) {
            timeoutMs = newTimeoutMs;
            reset();
        }

        bool timeIsUp() const { return elapsedMs() > timeoutMs; }

        double timeLeft() const {
            const double elapsed = elapsedMs();
            return elapsed < timeoutMs ? (timeoutMs - elapsed) : 0;
        }

        double elapsed(RclTimeUnit unit = RclTimeUnit::MILLISECOND) const {
            const double ms = elapsedMs();
            return unit == RclTimeUnit::SECOND ? (ms / 1000.0) : ms;
        }

    private:
        double timeoutMs;
        std::chrono::high_resolution_clock::time_point startTime;

        double elapsedMs() const {
            const auto dt = std::chrono::high_resolution_clock::now() - startTime;
            return std::chrono::duration<double, std::milli>(dt).count();
        }
};

inline constexpr double mmToInch = 0.039370078740157;
inline constexpr double MAX_OBSTACLE_DURATION = 1e12;  // ms
inline constexpr double FIELD_HALF_LENGTH = 70.5;      // inches
inline constexpr double FIELD_NEG_HALF_LENGTH = -70.5; // inches

inline double botToTrig(double ang) {
    double result = 90.0 - ang;
    while (result >= 360.0) result -= 360.0;
    while (result < 0.0) result += 360.0;
    return result;
}

inline double vexToStd(double vexDegrees) {
    double rads = (90.0 - vexDegrees) * (M_PI / 180.0);
    while (rads > M_PI) rads -= 2 * M_PI;
    while (rads < -M_PI) rads += 2 * M_PI;
    return rads;
}

inline double stdToVex(double stdRads) {
    double deg = 90.0 - (stdRads * 180.0 / M_PI);
    while (deg < 0.0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

inline double degToRad(double deg) { return deg * M_PI / 180.0; }

enum class CoordType { X, Y, INVALID };

struct SensorPose {
    double x = 0;
    double y = 0;
    double heading = 0;
    double slope = 0;
    double yIntercept = 0;
};

class LineObstacle {
    public:
        LineObstacle(double x1, double y1, double x2, double y2, double lifeTimeMs = -1);
        ~LineObstacle();

        LineObstacle(const LineObstacle&) = delete;
        LineObstacle& operator=(const LineObstacle&) = delete;

        bool expired() const;
        bool isIntersecting(const SensorPose& sp) const;

        static void addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs = -1);
        static void pruneExpired();
        static const std::vector<LineObstacle*>& obstacleCollection();

    private:
        struct LineData {
            double pt1[2] = {0, 0};
            double pt2[2] = {0, 0};
        } line;

        RclTimer lifeTimer;

        static std::vector<LineObstacle*> s_obstacles;
        static std::vector<std::unique_ptr<LineObstacle>> s_ownedObstacles;
};

class CircleObstacle {
    public:
        CircleObstacle(double x_, double y_, double r_, double lifeTimeMs = -1);
        ~CircleObstacle();

        CircleObstacle(const CircleObstacle&) = delete;
        CircleObstacle& operator=(const CircleObstacle&) = delete;

        bool expired() const;
        bool isIntersecting(const SensorPose& sp) const;

        static void pruneExpired();
        static const std::vector<CircleObstacle*>& obstacleCollection();

    private:
        double x;
        double y;
        double radius;
        RclTimer lifeTimer;

        static std::vector<CircleObstacle*> s_obstacles;
};

class RclSensor {
    public:
        RclSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng,
                  double angleTol = 10.0);
        ~RclSensor();

        RclSensor(const RclSensor&) = delete;
        RclSensor& operator=(const RclSensor&) = delete;

        void updatePose(const Pose& botPose);
        bool isValid(double distVal) const;
        std::pair<CoordType, double> getBotCoord(const Pose& botPose, double accum = NAN);
        int rawReading() const;
        SensorPose getPose() const;
        void logPos(std::ofstream* targetFile) const;

        static const std::vector<RclSensor*>& sensorCollection();

    private:
        pros::Distance* sensor;
        double offsetDist;
        double offsetAngle;
        double mainAngle;
        SensorPose sp;
        double angleTolerance;

        static std::vector<RclSensor*> s_sensors;
};

class RclTracking {
    public:
        RclTracking(Chassis* chassis_, int frequencyHz_ = 25, bool autoSync_ = true, double minDelta_ = 0.5,
                    double maxDelta_ = 4.0, double maxDeltaFromLemlib_ = 10.0, double maxSyncPerSec_ = 3.0,
                    int minPause_ = 20);
        ~RclTracking();

        void startTracking();
        void stopTracking();

        Pose getRclPose() const;
        void setRclPose(const Pose& p);
        void updateBotPose();
        void updateBotPose(RclSensor* sens);

        void setMaxSyncPerSec(double maxSyncPerSec_);

        void startAccumulating(bool autoUpdateAfterAccum = true);
        void stopAccumulating();
        void accumulateFor(int ms, bool autoUpdateAfterAccum = true);

        void discardData();

        void mainUpdate();
        void syncUpdate();
        void lifeTimeUpdate();

    private:
        Chassis* chassis;
        int goalMSPT;
        int minPause;
        double maxSyncPT;
        double minDelta;
        double maxDelta;
        double maxDeltaFromLemlib;
        bool autoSync;
        bool accumulating;
        bool updateAfterAccum;

        mutable pros::Mutex dataMutex;
        pros::Task* mainLoopTask = nullptr;
        pros::Task* miscLoopTask = nullptr;
        Pose latestPrecise;
        Pose poseAtLatest;

        void mainLoop();
        void miscLoop();
};

} // namespace gen

