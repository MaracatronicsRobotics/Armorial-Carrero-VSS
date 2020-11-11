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

#include <QCoreApplication>
#include <iostream>

#include <entity/armorialvisionclient/armorialvisionclient.h>
#include <entity/armorialvisionupdater/armorialvisionupdater.h>
#include <utils/fieldareas/fieldareas.h>
#include <utils/fieldtypes/fieldtypes.h>
#include <exithandler.h>

int main(int argc, char** argv) {

    QCoreApplication app(argc, argv);
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
    /*
     * Parsing info
    */

    // Command line parser, get arguments
    QCommandLineParser parser;
    parser.setApplicationDescription("Carrrero VSS application help.");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("visionAddress", "Sets the address that the application will listen to. (default is 224.5.23.2)");
    parser.addPositionalArgument("visionPort", "Sets the port that the application will listen to. (default is 10002)");
    parser.process(app);
    QStringList args = parser.positionalArguments();

    int visionSystemPort = 10002;
    std::string visionSystemAddress = "224.5.23.2";

    if(args.size() >= 1){
        std::string visionAddress = args.at(0).toStdString();
        visionSystemAddress = visionAddress;
    }

    if(args.size() >= 2){
        int port = args.at(1).toInt();

        if(port < 1 || port > 65535){
            std::cout << "[ERROR] Invalid port: " << port << std::endl;
            return EXIT_FAILURE;
        }

        visionSystemPort = port;
    }


    // Create modules
    ArmorialVisionClient eyeClient(visionSystemAddress, visionSystemPort);
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
