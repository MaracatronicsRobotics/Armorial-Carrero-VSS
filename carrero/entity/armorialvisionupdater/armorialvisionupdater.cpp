/***
 * Maracatronics Robotics
 * Federal University of Pernambuco (UFPE) at Recife
 * http://www.maracatronics.com/
 *
 * This file is part of Armorial project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***/

#include "armorialvisionupdater.h"

QString ArmorialVisionUpdater::name() {
    return "ArmorialVisionUpdater";
}

ArmorialVisionUpdater::ArmorialVisionUpdater(ArmorialVisionClient *ArmorialVisionClient, FieldAreas::FieldArea fieldLimit, bool enableLossFilter, bool enableKalmanFilter, bool enableNoiseFilter,  bool debugDetection, bool debugGeometry) {
    _ArmorialVisionClient = ArmorialVisionClient;
    _fieldLimit = fieldLimit;
    _debugDetection = debugDetection;
    _debugGeometry = debugGeometry;

    // Create Sensor (GEARSystem)
    _sensor = new Sensor();

    // Initialize robot objects
    QHash<int,Robot*> yellowRobots;
    QHash<int,Robot*> blueRobots;
    for(int id=0; id<MAX_ROBOTS; id++) {
        Robot *yellowRobot = new Robot(Colors::YELLOW, _yellowTeamIndex, id, enableLossFilter, enableKalmanFilter, enableNoiseFilter, debugDetection);
        yellowRobot->setSensor(_sensor);
        yellowRobots.insert(id, yellowRobot);

        Robot* blueRobot = new Robot(Colors::BLUE, _blueTeamIndex, id, enableLossFilter, enableKalmanFilter, enableNoiseFilter, debugDetection);
        blueRobot->setSensor(_sensor);
        blueRobots.insert(id, blueRobot);
    }
    _objRobots.insert(Colors::YELLOW, yellowRobots);
    _objRobots.insert(Colors::BLUE, blueRobots);

    // Initialize ball object
    _objBall = new Ball(enableLossFilter, enableKalmanFilter, enableNoiseFilter, debugDetection);
    _objBall->setSensor(_sensor);

    // Initialize robot multi object filters
    QHash<int,MultiObjectFilter*> yellowRobotsFilter;
    QHash<int,MultiObjectFilter*> blueRobotsFilter;
    for(int id=0; id<MAX_ROBOTS; id++) {
        MultiObjectFilter* yellowRobotFilter = new MultiObjectFilter();
        yellowRobotsFilter.insert(id, yellowRobotFilter);
        MultiObjectFilter* blueRobotFilter = new MultiObjectFilter();
        blueRobotsFilter.insert(id, blueRobotFilter);
    }
    _multiFilterRobots.insert(Colors::YELLOW, yellowRobotsFilter);
    _multiFilterRobots.insert(Colors::BLUE, blueRobotsFilter);
    // Initialize ball multi object filtering
    _multiFilterBall = new MultiObjectFilter();

    // Frequency
    this->setLoopFrequency(60); // Hz
}

ArmorialVisionUpdater::~ArmorialVisionUpdater() {
    // Sensor
    delete _sensor;

    // Robot objects
    QList<QHash<int,Robot*> > robots = _objRobots.values();
    for(int i=0; i<robots.size(); i++) {
        QList<Robot*> colorRobots = robots.at(i).values();

        for(int id=0; id<colorRobots.size(); id++)
            delete colorRobots.at(id);
    }

    // Ball object
    delete _objBall;

    // Robots multi object filtering
    QList<QHash<int,MultiObjectFilter*> > robotsFilter = _multiFilterRobots.values();
    for(int i=0; i<robotsFilter.size(); i++) {
        QList<MultiObjectFilter*> colorRobots = robotsFilter.at(i).values();
        for(int id=0; id<colorRobots.size(); id++)
            delete colorRobots.at(id);
    }
    // Ball multi object filtering
    delete _multiFilterBall;
}

QList<Robot*> ArmorialVisionUpdater::getRobotObjects() const {
    // Generate robots list
    QList<Robot*> yellowRobots = _objRobots.value(Colors::YELLOW).values();
    QList<Robot*> blueRobots = _objRobots.value(Colors::BLUE).values();

    return QList<Robot*>(yellowRobots+blueRobots);
}

Ball* ArmorialVisionUpdater::getBallObject() const {
    return _objBall;
}

fira_message::Field ArmorialVisionUpdater::getFieldGeometry() const {
    return _ArmorialVisionClient->getGeometryData();
}

bool ArmorialVisionUpdater::hasDetectionUpdates(int timeout) const {
    return _newDetectionUpdates.tryAcquire(1, timeout);
}

bool ArmorialVisionUpdater::hasGeometryUpdates() const {
    return _newGeometryUpdates.tryAcquire();
}

//************************Começo dos métodos abstratos de Entity*********************

void ArmorialVisionUpdater::initialization() {

    // WRBackbone connect (as Sensor)
    _sensor->connect("127.0.0.1", 0);
    if(_sensor->isConnected())
        cout << ">> ArmorialVision: Connected to WRBackbone.\n";
    else {
        cout << ">> ArmorialVision: [ERROR] Cannot connect to WRBackbone.\n";
        this->stopRunning();
        return;
    }

    // Sensor configuration
    _sensor->addBall(0);
    _sensor->addTeam(0, "Maracatronics");
    _sensor->addTeam(1, "Opponent");
    std::cout << ">> ArmorialVision: Teams #0 (yellow) and #1 (blue) created.\n";
    for(int i=0; i<MAX_ROBOTS; i++) {
        _sensor->addPlayer(0, i);
        _sensor->addPlayer(1, i);
        std::cout << ">> ArmorialVision: Player #" << i << " created.\n";
    }

    // Debug config
    cout.setf(ios::fixed,ios::floatfield);
    cout.precision(2);
}

void ArmorialVisionUpdater::loop() {

    // Detection data
    if(_ArmorialVisionClient->hasDetectionUpdate()) {

        // Request data to EyeClient
        QList<fira_message::Frame> detectionFrames = _ArmorialVisionClient->getDetectionData();

        // Process
        processDetectionData(detectionFrames);

        // Release update
        if(_newDetectionUpdates.available()==0)
            _newDetectionUpdates.release();
    }

    // Geometry data
    if(_ArmorialVisionClient->hasGeometryUpdate()) {

        // Request data to EyeClient
        fira_message::Field geometryData = _ArmorialVisionClient->getGeometryData();

        // Process
        processGeometryData(geometryData);

        // Release update
        if(_newGeometryUpdates.available()==0)
            _newGeometryUpdates.release();
    }
}

void ArmorialVisionUpdater::finalization() {
    _sensor->disconnect();
}

void ArmorialVisionUpdater::processDetectionData(const QList<fira_message::Frame> &detectionFrames) {

    // Parse camera data
    QList<std::pair<int,fira_message::Ball> > balls = parseCamerasBalls(detectionFrames);
    QHash<int,std::pair<int,fira_message::Robot> > robots = parseCamerasRobots(detectionFrames);

    // Process data
    processBalls(balls);
    processRobots(robots);
 }

QList<std::pair<int,fira_message::Ball> > ArmorialVisionUpdater::parseCamerasBalls(const QList<fira_message::Frame> &detectionFrames) const {
    QList<std::pair<int,fira_message::Ball> > retn;

    // Run on detection frames
    QList<fira_message::Frame>::const_iterator it;
    for(it=detectionFrames.constBegin(); it!=detectionFrames.constEnd(); it++) {
        fira_message::Frame frame = *it;

        // Add to retn list
        retn.append(std::make_pair(0, frame.ball()));
    }

    return retn;
}

QHash<int,std::pair<int,fira_message::Robot> > ArmorialVisionUpdater::parseCamerasRobots(const QList<fira_message::Frame> &detectionFrames) const {
    QHash<int,std::pair<int,fira_message::Robot> > retn;

    // Run on detection frames
    QList<fira_message::Frame>::const_iterator it;
    for(it=detectionFrames.constBegin(); it!=detectionFrames.constEnd(); it++) {
        fira_message::Frame frame = *it;

        // Add yellow to retn list
        for(int i=0; i<frame.robots_yellow_size(); i++)
            retn.insertMulti(Colors::YELLOW, std::make_pair(0, frame.robots_yellow(i))); // 0 = camera_id, change later if needed

        // Add blue to retn list
        for(int i=0; i<frame.robots_blue_size(); i++)
            retn.insertMulti(Colors::BLUE, std::make_pair(0, frame.robots_blue(i))); // 0 = camera_id, change later if needed
    }

    return retn;
}

void ArmorialVisionUpdater::processBalls(const QList<std::pair<int,fira_message::Ball> > &balls) {
    QList<std::pair<int,fira_message::Ball> >::const_iterator it;
    for(it=balls.constBegin(); it!=balls.constEnd(); it++) {
        const int camId = it->first;
        const fira_message::Ball ball = it->second;

        float realX = ball.x();
        float realY = ball.y();

        _objBall->update(100.0, Position(true, realX, realY, 0.0), Angle(false, 0));
    }
}

void ArmorialVisionUpdater::processRobots(const QHash<int,std::pair<int,fira_message::Robot> > &robots) {
    // Start tracker for unkwnown id robots
    _robotTracker.startAssociations();
    // Check both yellow and blue
    for(int color=Colors::YELLOW; color<=Colors::BLUE; color++) {
        QList<std::pair<int,fira_message::Robot> > colorRobots = robots.values(color);
        // Check all robot ids
        for(unsigned id=0; id<MAX_ROBOTS; id++) {
            QList<std::pair<int,fira_message::Robot> >::iterator it;
            for(it=colorRobots.begin(); it!=colorRobots.end(); it++) {
                const int camId = it->first;
                const fira_message::Robot robot = it->second;

                // Discard robots with wrong id
                if(robot.robot_id()!=id)
                    continue;

                float realX = robot.x();
                float realY = robot.y();

                // Add to multi robot filtering
                Angle ori(true, robot.orientation());
                _objRobots.value(color).value(id)->update(100.0, Position(true, realX, realY, 0.0), ori);
            }
        }
    }
}

void ArmorialVisionUpdater::processGeometryData(const fira_message::Field &geometryData) {
    // Get field data
    const fira_message::Field &field = geometryData;

    // Update to system
    // Change this later!
    double areaRadius = 150.0;
    double centerRadius = 200.0;
    double areaLength = 700.0;
    double areaWidth = 150.0;
    double goalDepth = 100.0;
    _sensor->setFieldCenter(Position(true, 0.0, 0.0, 0.0));
    _sensor->setFieldTopRightCorner(Position(true, (field.length()/2.0)*MM2METER, (field.width()/2.0)*MM2METER, 0.0));
    _sensor->setFieldTopLeftCorner(Position(true, (-field.length()/2.0)*MM2METER, (field.width()/2.0)*MM2METER, 0.0));
    _sensor->setFieldBottomLeftCorner(Position(true, (-field.length()/2.0)*MM2METER, (-field.width()/2.0)*MM2METER, 0.0));
    _sensor->setFieldBottomRightCorner(Position(true, (field.length()/2.0)*MM2METER, (-field.width()/2.0)*MM2METER, 0.0));
    _sensor->setRightPenaltyMark(Position(true, (field.length()/2.0 - areaRadius)*MM2METER, 0.0, 0.0));
    _sensor->setLeftPenaltyMark(Position(true, (-field.length()/2.0 + areaRadius)*MM2METER, 0.0, 0.0));
    _sensor->setLeftGoalPosts(Position(true, (-field.length()/2.0)*MM2METER, (-field.goal_width()/2.0)*MM2METER, 0.0), Position(true, (-field.length()/2.0)*MM2METER, (field.goal_width()/2.0)*MM2METER, 0.0));
    _sensor->setRightGoalPosts(Position(true, (field.length()/2.0)*MM2METER, (field.goal_width()/2.0)*MM2METER, 0.0), Position(true, (field.length()/2.0)*MM2METER, (-field.goal_width()/2.0)*MM2METER, 0.0));
    _sensor->setFieldCenterRadius(centerRadius*MM2METER);
    _sensor->setGoalArea(areaLength*MM2METER, areaWidth*MM2METER, areaRadius*MM2METER);
    _sensor->setGoalDepth(goalDepth*MM2METER);


    if(_debugGeometry) {
        printf("[Geometry Data]\n");
        printf("Field Dimensions:\n");
        printf("  -field_length=%lf (mm)\n",field.length());
        printf("  -field_width=%lf (mm)\n",field.width());
        printf("  -goal_width=%lf (mm)\n",field.goal_width());
        printf("  -goal_depth=%lf (mm)\n",field.goal_depth());
    }
}
