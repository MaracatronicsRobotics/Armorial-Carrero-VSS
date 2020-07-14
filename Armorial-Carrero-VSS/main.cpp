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

#include <QApplication>
#include <iostream>

#include <entity/armorialvisionclient/armorialvisionclient.h>
#include <entity/armorialvisionupdater/armorialvisionupdater.h>
#include <utils/fieldareas/fieldareas.h>
#include <utils/fieldtypes/fieldtypes.h>
#include <exithandler.h>

int main(int argc, char** argv) {

    QApplication app(argc, argv);
    int visionSystemPort = 10002;
    FieldTypes::FieldType fieldType = FieldTypes::VISION;
    FieldAreas::FieldArea fieldLimit = FieldAreas::ALL;
    bool debugDetection = false;
    bool debugGeometry = false;
    /*******************************************/
    /*********** EXIT HANDLER (CTRL+C) *********/
    /*******************************************/
    ExitHandler::setApplication(&app);
    ExitHandler::setup();
    /******************************************/
    /************* EYE THREADS ****************/
    /******************************************/
    // Filters
    const bool enableLossFilter = true;
    const bool enableKalmanFilter = true;
    const bool enableNoiseFilter = true;
    // Create modules
    ArmorialVisionClient eyeClient(visionSystemPort);
    ArmorialVisionUpdater eyeUpdater(&eyeClient, fieldLimit, enableLossFilter, enableKalmanFilter, enableNoiseFilter, debugDetection, debugGeometry);

    // Start modules
    eyeClient.start();
    eyeUpdater.start();

    // Run app
    bool exec = app.exec();
    // Stop modules
    eyeClient.terminate(); // blocking thread, need to terminate
    eyeUpdater.stopRunning();

    // Wait for modules threads
    eyeClient.wait();
    eyeUpdater.wait();

    return exec;
}
