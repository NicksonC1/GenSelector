#include "gen/rclTracking.hpp"

#include <algorithm>
#include <cmath>

namespace gen {

std::vector<LineObstacle*> LineObstacle::s_obstacles{};
std::vector<std::unique_ptr<LineObstacle>> LineObstacle::s_ownedObstacles{};
std::vector<CircleObstacle*> CircleObstacle::s_obstacles{};
std::vector<RclSensor*> RclSensor::s_sensors{};

static void eraseObstaclePtr(std::vector<LineObstacle*>& collection, LineObstacle* ptr) {
    collection.erase(std::remove(collection.begin(), collection.end(), ptr), collection.end());
}

static void eraseObstaclePtr(std::vector<CircleObstacle*>& collection, CircleObstacle* ptr) {
    collection.erase(std::remove(collection.begin(), collection.end(), ptr), collection.end());
}

LineObstacle::LineObstacle(double x1, double y1, double x2, double y2, double lifeTimeMs)
    : lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
    line.pt1[0] = x1;
    line.pt1[1] = y1;
    line.pt2[0] = x2;
    line.pt2[1] = y2;
    s_obstacles.push_back(this);
}

LineObstacle::~LineObstacle() { eraseObstaclePtr(s_obstacles, this); }

bool LineObstacle::expired() const { return lifeTimer.timeIsUp(); }

bool LineObstacle::isIntersecting(const SensorPose& sp) const {
    const double angRad = degToRad(botToTrig(sp.heading));
    const double rayX = std::cos(angRad);
    const double rayY = std::sin(angRad);

    const double segX = line.pt2[0] - line.pt1[0];
    const double segY = line.pt2[1] - line.pt1[1];

    const double det = (rayX * segY) - (rayY * segX);
    if (std::abs(det) < 1e-6) return false;

    const double dx = line.pt1[0] - sp.x;
    const double dy = line.pt1[1] - sp.y;

    const double t = (dx * segY - dy * segX) / det;
    const double u = (dx * rayY - dy * rayX) / det;
    return t > 0 && u >= 0 && u <= 1;
}

void LineObstacle::addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs) {
    if (points.size() < 3) return;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const std::size_t next = (i + 1) % points.size();
        s_ownedObstacles.emplace_back(std::make_unique<LineObstacle>(points[i].first, points[i].second,
                                                                      points[next].first, points[next].second,
                                                                      lifeTimeMs));
    }
}

void LineObstacle::pruneExpired() {
    s_obstacles.erase(std::remove_if(s_obstacles.begin(), s_obstacles.end(),
                                     [](LineObstacle* obstacle) { return obstacle == nullptr || obstacle->expired(); }),
                      s_obstacles.end());
    s_ownedObstacles.erase(std::remove_if(s_ownedObstacles.begin(), s_ownedObstacles.end(),
                                          [](const std::unique_ptr<LineObstacle>& obstacle) {
                                              return obstacle == nullptr || obstacle->expired();
                                          }),
                           s_ownedObstacles.end());
}

const std::vector<LineObstacle*>& LineObstacle::obstacleCollection() { return s_obstacles; }

CircleObstacle::CircleObstacle(double x_, double y_, double r_, double lifeTimeMs)
    : x(x_),
      y(y_),
      radius(r_),
      lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
    s_obstacles.push_back(this);
}

CircleObstacle::~CircleObstacle() { eraseObstaclePtr(s_obstacles, this); }

bool CircleObstacle::expired() const { return lifeTimer.timeIsUp(); }

bool CircleObstacle::isIntersecting(const SensorPose& sp) const {
    const double angRad = degToRad(botToTrig(sp.heading));
    const double rayX = std::cos(angRad);
    const double rayY = std::sin(angRad);

    const double dx = x - sp.x;
    const double dy = y - sp.y;

    const double t = dx * rayX + dy * rayY;
    if (t < 0) return false;

    const double closestX = sp.x + t * rayX;
    const double closestY = sp.y + t * rayY;
    const double distSq = std::pow(closestX - x, 2) + std::pow(closestY - y, 2);
    return distSq <= (radius * radius);
}

void CircleObstacle::pruneExpired() {
    s_obstacles.erase(
        std::remove_if(s_obstacles.begin(), s_obstacles.end(),
                       [](CircleObstacle* obstacle) { return obstacle == nullptr || obstacle->expired(); }),
        s_obstacles.end());
}

const std::vector<CircleObstacle*>& CircleObstacle::obstacleCollection() { return s_obstacles; }

RclSensor::RclSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol)
    : sensor(distSensor),
      offsetDist(std::hypot(horizOffset, vertOffset)),
      offsetAngle(std::fmod((std::atan2(vertOffset, horizOffset) * 180.0 / M_PI) + 360.0, 360.0)),
      mainAngle(mainAng),
      angleTolerance(std::abs(angleTol)) {
    s_sensors.push_back(this);
}

RclSensor::~RclSensor() { s_sensors.erase(std::remove(s_sensors.begin(), s_sensors.end(), this), s_sensors.end()); }

void RclSensor::updatePose(const Pose& botPose) {
    const double theta = degToRad(offsetAngle - botPose.theta);
    sp.x = botPose.x + std::cos(theta) * offsetDist;
    sp.y = botPose.y + std::sin(theta) * offsetDist;

    sp.heading = std::fmod(botPose.theta + mainAngle, 360.0);
    if (sp.heading < 0) sp.heading += 360.0;

    sp.slope = std::tan(degToRad(botToTrig(sp.heading)));
    sp.yIntercept = sp.y - sp.slope * sp.x;
}

bool RclSensor::isValid(double distVal) const {
    if (sensor == nullptr) return false;
    if (distVal <= 0 || distVal > 2000) return false;
    if (distVal > 200 && sensor->get_confidence() < 60) return false;

    double headingMod = std::fmod(sp.heading, 90.0);
    if (headingMod < 0) headingMod += 90.0;
    if (headingMod > angleTolerance && headingMod < (90.0 - angleTolerance)) return false;

    for (auto* obstacle : CircleObstacle::obstacleCollection()) {
        if (obstacle != nullptr && obstacle->isIntersecting(sp)) return false;
    }
    for (auto* obstacle : LineObstacle::obstacleCollection()) {
        if (obstacle != nullptr && obstacle->isIntersecting(sp)) return false;
    }
    return true;
}

std::pair<CoordType, double> RclSensor::getBotCoord(const Pose& botPose, double accum) {
    updatePose(botPose);

    const double rawVal = std::isnan(accum) ? static_cast<double>(rawReading()) : accum;
    if (!isValid(rawVal)) return {CoordType::INVALID, 0.0};

    const double val = rawVal * mmToInch;
    const double angRad = degToRad(botToTrig(sp.heading));
    const double cosA = std::cos(angRad);
    const double sinA = std::sin(angRad);

    double minDist = 1e9;
    int wall = -1;

    if (std::abs(cosA) > 1e-6) {
        const double dEast = (FIELD_HALF_LENGTH - sp.x) / cosA;
        if (dEast > 0 && dEast < minDist) {
            minDist = dEast;
            wall = 2;
        }
        const double dWest = (FIELD_NEG_HALF_LENGTH - sp.x) / cosA;
        if (dWest > 0 && dWest < minDist) {
            minDist = dWest;
            wall = 4;
        }
    }

    if (std::abs(sinA) > 1e-6) {
        const double dNorth = (FIELD_HALF_LENGTH - sp.y) / sinA;
        if (dNorth > 0 && dNorth < minDist) {
            minDist = dNorth;
            wall = 1;
        }
        const double dSouth = (FIELD_NEG_HALF_LENGTH - sp.y) / sinA;
        if (dSouth > 0 && dSouth < minDist) {
            minDist = dSouth;
            wall = 3;
        }
    }

    double result = 0;
    CoordType type = CoordType::INVALID;
    if (wall == 1) {
        type = CoordType::Y;
        result = FIELD_HALF_LENGTH - sinA * val;
    } else if (wall == 2) {
        type = CoordType::X;
        result = FIELD_HALF_LENGTH - cosA * val;
    } else if (wall == 3) {
        type = CoordType::Y;
        result = FIELD_NEG_HALF_LENGTH - sinA * val;
    } else if (wall == 4) {
        type = CoordType::X;
        result = FIELD_NEG_HALF_LENGTH - cosA * val;
    } else {
        return {CoordType::INVALID, 0.0};
    }

    const double offRad = degToRad(offsetAngle - botPose.theta);
    if (type == CoordType::X) result -= std::cos(offRad) * offsetDist;
    if (type == CoordType::Y) result -= std::sin(offRad) * offsetDist;
    return {type, result};
}

int RclSensor::rawReading() const { return sensor == nullptr ? 0 : sensor->get(); }

SensorPose RclSensor::getPose() const { return sp; }

void RclSensor::logPos(std::ofstream* targetFile) const {
    if (targetFile == nullptr || !targetFile->good()) return;
    *targetFile << "x=" << sp.x << ", y=" << sp.y << ", heading=" << sp.heading << ", raw=" << rawReading()
                << ", conf=" << (sensor == nullptr ? 0 : sensor->get_confidence()) << '\n';
}

const std::vector<RclSensor*>& RclSensor::sensorCollection() { return s_sensors; }

RclTracking::RclTracking(Chassis* chassis_, int frequencyHz_, bool autoSync_, double minDelta_, double maxDelta_,
                         double maxDeltaFromLemlib_, double maxSyncPerSec_, int minPause_)
    : chassis(chassis_),
      goalMSPT(std::max(1, static_cast<int>(std::round(1000.0 / std::max(1, frequencyHz_))))),
      minPause(std::max(1, minPause_)),
      maxSyncPT(maxSyncPerSec_ / std::max(1, frequencyHz_)),
      minDelta(minDelta_),
      maxDelta(maxDelta_),
      maxDeltaFromLemlib(maxDeltaFromLemlib_),
      autoSync(autoSync_),
      accumulating(false),
      updateAfterAccum(false),
      latestPrecise(0, 0, 0),
      poseAtLatest(0, 0, 0) {
    if (chassis != nullptr) {
        const Pose current = chassis->getPose();
        latestPrecise = current;
        poseAtLatest = current;
    }
}

RclTracking::~RclTracking() { stopTracking(); }

void RclTracking::startTracking() {
    if (mainLoopTask == nullptr) mainLoopTask = new pros::Task([this]() { mainLoop(); });
    if (miscLoopTask == nullptr) miscLoopTask = new pros::Task([this]() { miscLoop(); });
}

void RclTracking::stopTracking() {
    if (mainLoopTask != nullptr) {
        mainLoopTask->remove();
        delete mainLoopTask;
        mainLoopTask = nullptr;
    }
    if (miscLoopTask != nullptr) {
        miscLoopTask->remove();
        delete miscLoopTask;
        miscLoopTask = nullptr;
    }
}

Pose RclTracking::getRclPose() const {
    Pose latest(0, 0, 0);
    Pose anchor(0, 0, 0);
    dataMutex.take(TIMEOUT_MAX);
    latest = latestPrecise;
    anchor = poseAtLatest;
    dataMutex.give();

    if (chassis == nullptr) return latest;
    const Pose chassisPose = chassis->getPose();
    return Pose(latest.x + (chassisPose.x - anchor.x), latest.y + (chassisPose.y - anchor.y), chassisPose.theta);
}

void RclTracking::setRclPose(const Pose& p) {
    dataMutex.take(TIMEOUT_MAX);
    latestPrecise = p;
    poseAtLatest = chassis == nullptr ? p : chassis->getPose();
    dataMutex.give();
}

void RclTracking::updateBotPose() {
    if (chassis == nullptr) return;
    const Pose p = getRclPose();
    chassis->setPose(p);
    setRclPose(p);
}

void RclTracking::updateBotPose(RclSensor* sens) {
    if (sens == nullptr || chassis == nullptr) return;

    const auto data = sens->getBotCoord(chassis->getPose());
    Pose pose = chassis->getPose();
    if (data.first == CoordType::X) pose.x = data.second;
    else if (data.first == CoordType::Y) pose.y = data.second;
    else return;

    chassis->setPose(Pose(pose.x, pose.y, chassis->getPose().theta));
    setRclPose(Pose(pose.x, pose.y, chassis->getPose().theta));
}

void RclTracking::setMaxSyncPerSec(double maxSyncPerSec_) {
    const double updatesPerSecond = 1000.0 / goalMSPT;
    maxSyncPT = updatesPerSecond > 0 ? (maxSyncPerSec_ / updatesPerSecond) : 0;
}

void RclTracking::startAccumulating(bool autoUpdateAfterAccum) {
    accumulating = true;
    updateAfterAccum = autoUpdateAfterAccum;
}

void RclTracking::stopAccumulating() { accumulating = false; }

void RclTracking::accumulateFor(int ms, bool autoUpdateAfterAccum) {
    startAccumulating(autoUpdateAfterAccum);
    RclTimer timer(ms);
    while (!timer.timeIsUp()) pros::delay(minPause);
    stopAccumulating();
}

void RclTracking::discardData() {
    if (chassis == nullptr) return;
    const Pose current = chassis->getPose();
    dataMutex.take(TIMEOUT_MAX);
    latestPrecise = current;
    poseAtLatest = current;
    dataMutex.give();
}

void RclTracking::mainUpdate() {
    if (chassis == nullptr || RclSensor::sensorCollection().empty()) return;

    const auto& sensors = RclSensor::sensorCollection();
    std::vector<int> accTotal(sensors.size(), 0);
    std::vector<int> accCount(sensors.size(), 0);

    while (accumulating) {
        for (std::size_t i = 0; i < sensors.size(); i++) {
            accTotal[i] += sensors[i]->rawReading();
            accCount[i]++;
        }
        pros::delay(goalMSPT);
    }

    Pose botPose = getRclPose();
    const Pose chassisPose = chassis->getPose();
    const double diffFromChassis = std::hypot(botPose.x - chassisPose.x, botPose.y - chassisPose.y);
    if (diffFromChassis > maxDeltaFromLemlib && diffFromChassis > 1e-6) {
        botPose.x += (chassisPose.x - botPose.x) / diffFromChassis;
        botPose.y += (chassisPose.y - botPose.y) / diffFromChassis;
    }

    std::vector<double> xs;
    std::vector<double> ys;
    for (std::size_t i = 0; i < sensors.size(); i++) {
        const double avg = accCount[i] > 0 ? (1.0 * accTotal[i] / accCount[i]) : NAN;
        const auto [type, coord] = sensors[i]->getBotCoord(botPose, avg);

        if (type == CoordType::X && std::abs(coord - botPose.x) <= maxDelta) xs.push_back(coord);
        if (type == CoordType::Y && std::abs(coord - botPose.y) <= maxDelta) ys.push_back(coord);
    }

    const bool hadAccumSamples = std::any_of(accCount.begin(), accCount.end(), [](int count) { return count > 0; });
    const double minDeltaThreshold = hadAccumSamples ? 0.0 : minDelta;

    if (!xs.empty()) {
        const double meanX = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
        if (meanX > FIELD_NEG_HALF_LENGTH && meanX < FIELD_HALF_LENGTH && std::abs(meanX - botPose.x) >= minDeltaThreshold) {
            dataMutex.take(TIMEOUT_MAX);
            latestPrecise.x = meanX;
            poseAtLatest.x = chassis->getPose().x;
            dataMutex.give();
        }
    }

    if (!ys.empty()) {
        const double meanY = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
        if (meanY > FIELD_NEG_HALF_LENGTH && meanY < FIELD_HALF_LENGTH && std::abs(meanY - botPose.y) >= minDeltaThreshold) {
            dataMutex.take(TIMEOUT_MAX);
            latestPrecise.y = meanY;
            poseAtLatest.y = chassis->getPose().y;
            dataMutex.give();
        }
    }

    if (updateAfterAccum && hadAccumSamples) {
        updateBotPose();
        updateAfterAccum = false;
    }
}

void RclTracking::syncUpdate() {
    if (chassis == nullptr) return;

    const Pose currRclPose = getRclPose();
    const Pose currChassisPose = chassis->getPose();

    const double xDiff = currRclPose.x - currChassisPose.x;
    const double yDiff = currRclPose.y - currChassisPose.y;
    const double realDiff = std::hypot(xDiff, yDiff);
    if (realDiff < 1e-6) return;

    double xUpdate = xDiff;
    double yUpdate = yDiff;
    if (realDiff > maxSyncPT && maxSyncPT > 0) {
        xUpdate = (xDiff / realDiff) * maxSyncPT;
        yUpdate = (yDiff / realDiff) * maxSyncPT;
    }

    chassis->setPose(currChassisPose.x + xUpdate, currChassisPose.y + yUpdate, currChassisPose.theta);

    dataMutex.take(TIMEOUT_MAX);
    poseAtLatest.x += xUpdate;
    poseAtLatest.y += yUpdate;
    poseAtLatest.theta = chassis->getPose().theta;
    dataMutex.give();
}

void RclTracking::lifeTimeUpdate() {
    CircleObstacle::pruneExpired();
    LineObstacle::pruneExpired();
}

void RclTracking::mainLoop() {
    RclTimer frequencyTimer(goalMSPT);
    while (true) {
        frequencyTimer.reset();
        mainUpdate();

        const int delayMs = static_cast<int>(std::round(std::max(static_cast<double>(minPause), frequencyTimer.timeLeft())));
        pros::delay(delayMs);
    }
}

void RclTracking::miscLoop() {
    RclTimer frequencyTimer(goalMSPT);
    while (true) {
        frequencyTimer.reset();
        if (autoSync) syncUpdate();
        lifeTimeUpdate();

        const int delayMs = static_cast<int>(std::round(std::max(static_cast<double>(minPause), frequencyTimer.timeLeft())));
        pros::delay(delayMs);
    }
}

} // namespace gen

