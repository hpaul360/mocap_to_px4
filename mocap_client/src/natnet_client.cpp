#include <cstdio>
#include <vector>
#include <string>

#include <NatNetClientInterface.h>

using namespace std;

// Constructor
MotionCaptureClient::MotionCaptureClient(NatnetDriverNode* _natNetHandler)
{
    initializeConnectionParams(_natNetHandler);
    setFrameCallback();
}

// Destructor
MotionCaptureClient::~MotionCaptureClient()
{
    if (this->dataDefinitions)
    {
        NatNet_FreeDescriptions(this->dataDefinitions);
        this->dataDefinitions = nullptr; 
    }
    this->natNetHandler = nullptr;
}

// Connect to the server
int MotionCaptureClient::connectToServer()
{
    int retCode = 0;
    ErrorCode ret = ErrorCode_OK;
    
    // Disconnect previous connection
    this->Disconnect();
    
    // Get and print NatNet version info
    getVersionInfo() ;
    
    // Connect the client to the server
    retCode = this->Connect(connectionParams);
    memset(&serverDescription, 0, sizeof(serverDescription));
    
    ret = this->GetServerDescription(&serverDescription);
    if ( ret != ErrorCode_OK || ! serverDescription.HostPresent)
    {
        RCLCPP_ERROR(this->natNetHandler->get_logger(), "ERROR: Unable to connect. Exiting!!!\n");
        return 1;
    }

    // Retrieve server info
    getServerInfo();

    // Retrieve frame rate
    getFrameRate();

    // Retrieve analog samples
    getAnalogSamples();

    printLogSeparator(this->natNetHandler->get_logger(), 40, '-');

    // Check and start recording if requested
    std::string recordName = natNetHandler->getRecordName();
    if (!recordName.empty()) 
    {
        this->setRecordName(recordName);
    }

    if (natNetHandler->isRecordingRequested()) 
    {
        this->isRecordingStarted = this->startRecording();
    }

    // Get all info from the server
    getDataDescription();

    return retCode;
}

// Disconnect from server
void MotionCaptureClient::disconnectFromServer()
{
    // Stop recording if it's ongoing
    if (this->isRecordingStarted) 
    {
        bool recordingStopped = this->stopRecording();
        if (recordingStopped) 
        {
            this->isRecordingStarted = false;
        }
    }
    this->Disconnect();
}

// Get publisher
NatnetDriverNode* MotionCaptureClient::getPublisher()
{
    return this->natNetHandler;
}

// Send data to handler for publishing
void MotionCaptureClient::sendRigidBodyMessage(double currentSecsSinceEpoch, sRigidBodyData* bodies, int nRigidBodies)
{
    this->natNetHandler->storeRigidBodyMessage(currentSecsSinceEpoch, bodies, nRigidBodies);
}

// Callbacks for data frames from the server
void NATNET_CALLCONV dataFrameHandler(sFrameOfMocapData* data, void* pUserData)
{
    MotionCaptureClient* pClient = static_cast<MotionCaptureClient*>(pUserData);

    auto logger = pClient->getPublisher()->get_logger();

    RCLCPP_DEBUG(logger, "FrameID : %d", data->iFrame);
    RCLCPP_DEBUG(logger, "Timestamp : %3.2lf", data->fTimestamp);

    if (data->params & 0x01)
        RCLCPP_DEBUG(logger, "RECORDING!!!");
    if (data->params & 0x02)
        RCLCPP_DEBUG(logger, "Models Changed.");

    int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

    char szTimecode[128] = "";
    NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
    RCLCPP_DEBUG(logger, "Timecode : %s", szTimecode);

    const double currentSecsSinceEpoch = pClient->getPublisher()->get_clock()->now().seconds();

    pClient->sendRigidBodyMessage(currentSecsSinceEpoch, data->RigidBodies, data->nRigidBodies);

    RCLCPP_DEBUG(logger, "Markers [Count=%d]", data->nLabeledMarkers);
    for (int i = 0; i < data->nLabeledMarkers; i++) {
        sMarker marker = data->LabeledMarkers[i];

        int modelID, markerID;
        NatNet_DecodeID(marker.ID, &modelID, &markerID);

        std::string markerType = (marker.params & 0x20) ? "Active" : ((marker.params & 0x10) ? "Unlabeled" : "Labeled");

        RCLCPP_DEBUG(logger, "%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]",
            markerType.c_str(), modelID, markerID, marker.size, marker.x, marker.y, marker.z);
    }
	
}

// Set record name
bool MotionCaptureClient::setRecordName(const std::string& recordName)
{
    void* pResult = nullptr;
    int nBytes = 0;
    std::string command = "SetRecordTakeName," + recordName;
    ErrorCode ret = this->SendMessageAndWait(command.c_str(), &pResult, &nBytes);

    if (ret != ErrorCode_OK) 
    {
        RCLCPP_ERROR(this->natNetHandler->get_logger(), "ERROR: Recording name could not be set. Return code: %d", ret);
    }
    return ret == ErrorCode_OK;
}

// Start record request
bool MotionCaptureClient::startRecording()
{
    void* pResult = nullptr;
    int nBytes = 0;
    ErrorCode ret = this->SendMessageAndWait("StartRecording", &pResult, &nBytes);

    if (ret != ErrorCode_OK) 
    {
        RCLCPP_ERROR(this->natNetHandler->get_logger(), "ERROR: Recording could not be started. Return code: %d", ret);
    }
    return ret == ErrorCode_OK;
}

// Stop record request
bool MotionCaptureClient::stopRecording()
{
    void* pResult = nullptr;
    int nBytes = 0;
    ErrorCode ret = this->SendMessageAndWait("StopRecording", &pResult, &nBytes);

    if (ret != ErrorCode_OK) 
    {
        RCLCPP_ERROR(this->natNetHandler->get_logger(), "ERROR: Recording could not be stopped. Return code: %d", ret);
    }
    return ret == ErrorCode_OK;
}

// Log seperator
void MotionCaptureClient::printLogSeparator(const rclcpp::Logger& logger, int count, char separatorChar) 
{
    RCLCPP_INFO(logger, "%s", std::string(count, separatorChar).c_str());
}

// Initialize connection parameters
void MotionCaptureClient::initializeConnectionParams(NatnetDriverNode* _natNetHandler) 
{
    this->natNetHandler = _natNetHandler;

    connectionParams.connectionType = (natNetHandler->getConnectionType() == 0) ? ConnectionType_Multicast : ConnectionType_Unicast;
    
    connectionParams.serverDataPort = natNetHandler->getServerDataPort();
    connectionParams.serverCommandPort = natNetHandler->getServerCommandPort();

    std::string serverAddress_s = natNetHandler->getServerAddress();
    char* serverAddress = (char*) malloc(serverAddress_s.length() * sizeof(char));
    strcpy(serverAddress, serverAddress_s.c_str());
    connectionParams.serverAddress = (const char*) serverAddress;

    std::string multiCastAddress_s = natNetHandler->getMulticastAddress();
    char* multiCastAddress = (char*) malloc(multiCastAddress_s.length() * sizeof(char));
    strcpy(multiCastAddress, multiCastAddress_s.c_str());
    connectionParams.multicastAddress = (const char*) multiCastAddress;

    dataDefinitions = nullptr;
}

// Frame callback
void MotionCaptureClient::setFrameCallback() 
{
    this->SetFrameReceivedCallback(dataFrameHandler, this);
}

// Get the data description from the server
void MotionCaptureClient::getDataDescription()
{
    int iResult = 0;

    RCLCPP_INFO(this->natNetHandler->get_logger(), "Requesting descriptions from server...");
    iResult = this->GetDataDescriptionList(&this->dataDefinitions);

	if (iResult != ErrorCode_OK || this->dataDefinitions == nullptr)
	{
		RCLCPP_ERROR(this->natNetHandler->get_logger(), "ERROR: Unable to retreive data descriptions!!!");
	}
	else
	{
	// Log successful retrieval 
    processDataDefinitions(this->dataDefinitions);
    }
}

// NatNet version
void MotionCaptureClient::getVersionInfo() 
{
    unsigned char ver[4];
    NatNet_GetVersion(ver);
    // Print version info
    RCLCPP_INFO(this->natNetHandler->get_logger(), "NatNet Client (NatNet ver. %d.%d.%d.%d)", ver[0], ver[1], ver[2], ver[3]);
    printLogSeparator(this->natNetHandler->get_logger(), 40, '-');
}

// Server info
void MotionCaptureClient::getServerInfo()
{
    char versionStr[20];
    char natNetVersionStr[20];

    sprintf(versionStr, "%d.%d.%d.%d", serverDescription.HostAppVersion[0],
                                       serverDescription.HostAppVersion[1],
                                       serverDescription.HostAppVersion[2],
                                       serverDescription.HostAppVersion[3]);

    sprintf(natNetVersionStr, "%d.%d.%d.%d", serverDescription.NatNetVersion[0],
                                             serverDescription.NatNetVersion[1],
                                             serverDescription.NatNetVersion[2],
                                             serverDescription.NatNetVersion[3]);

    // Print server info
    RCLCPP_INFO(this->natNetHandler->get_logger(), "####### Server application info ########");
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Application: %s (ver.%s)", serverDescription.szHostApp, versionStr);
    RCLCPP_INFO(this->natNetHandler->get_logger(), "NatNet version: %s", natNetVersionStr);
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Server IP: %s", connectionParams.serverAddress);
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Server name: %s", serverDescription.szHostComputerName);
}

// Frame rate
void MotionCaptureClient::getFrameRate()
{
    void* pResult = nullptr;
    int nBytes = 0;
    ErrorCode ret = this->SendMessageAndWait("FrameRate", &pResult, &nBytes);

    if (ret == ErrorCode_OK)
    {
        float frameRate;
        memcpy(&frameRate, pResult, sizeof(float));
        RCLCPP_INFO(this->natNetHandler->get_logger(), "MoCap framerate: %3.2f", frameRate);
    }
    else
    {
        RCLCPP_ERROR(this->natNetHandler->get_logger(), "ERROR: Cannot get the frame rate!!!");
    }
}

// Analog samples per MoCap frame
void MotionCaptureClient::getAnalogSamples()
{
    void* pResult = nullptr;
    int nBytes = 0;
    ErrorCode ret = this->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
    
    if (ret == ErrorCode_OK)
    {
        int analogSamples;
        memcpy(&analogSamples, pResult, sizeof(int));
        RCLCPP_INFO(this->natNetHandler->get_logger(), "Analog samples per MoCap frame: %d", analogSamples);
    }
    else
    {
        RCLCPP_ERROR(this->natNetHandler->get_logger(), "Error: Cannot get analog samples!!!");
    }
}

// Get server description
sServerDescription MotionCaptureClient::getServerDescription()
{
    return this->serverDescription;
}

// Process the data description retreived from the server
void MotionCaptureClient::processDataDefinitions(sDataDescriptions* dataDefinitions)
{
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Retreived data descriptions: %d", dataDefinitions->nDataDescriptions);

    for(int i=0; i < dataDefinitions->nDataDescriptions; i++)
    {
        switch (dataDefinitions->arrDataDescriptions[i].type)
         {
            case Descriptor_MarkerSet:
                RCLCPP_INFO(this->natNetHandler->get_logger(), "Marker set description (Index: %d)", i);
                processMarkerSet(dataDefinitions->arrDataDescriptions[i].Data.MarkerSetDescription);
                break;
            case Descriptor_RigidBody:
                RCLCPP_INFO(this->natNetHandler->get_logger(), "Rigid body description (Index: %d)", i);
                processRigidBody(dataDefinitions->arrDataDescriptions[i].Data.RigidBodyDescription);
                break;
            case Descriptor_Skeleton:
                // TODO: Implement Skeleton processing
                RCLCPP_INFO(this->natNetHandler->get_logger(), "Skeleton description not implemented (Index: %d)", i);
                break;
            case Descriptor_ForcePlate:
                // TODO: Implement Force Plate processing
                RCLCPP_INFO(this->natNetHandler->get_logger(), "Force plate description not implemented (Index: %d)", i);
                break;
            case Descriptor_Device:
                // TODO: Implement Device processing
                RCLCPP_INFO(this->natNetHandler->get_logger(), "Device description not implemented (Index: %d)", i);
                break;
            case Descriptor_Camera:
                RCLCPP_INFO(this->natNetHandler->get_logger(), "Camera description (Index: %d)", i);
                processCamera(dataDefinitions->arrDataDescriptions[i].Data.CameraDescription);
                break;
            default:
                RCLCPP_WARN(this->natNetHandler->get_logger(), "Unknown data type encountered at index %d!", i);
                break;
        }
        printLogSeparator(this->natNetHandler->get_logger(), 40, '-');
    }
}

// MarkerSet descriptor
void MotionCaptureClient::processMarkerSet(sMarkerSetDescription* markerSetDesc)
{
    RCLCPP_INFO(this->natNetHandler->get_logger(), "MarkerSet Name : %s", markerSetDesc->szName);
    for(int i=0; i < markerSetDesc->nMarkers; i++)
        RCLCPP_INFO(this->natNetHandler->get_logger(), "Marker %d: %s", i + 1, markerSetDesc->szMarkerNames[i]);
}

// RigidBody descriptor
void MotionCaptureClient::processRigidBody(sRigidBodyDescription* rigidBodyDesc)
{
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Rigid body name : %s", rigidBodyDesc->szName);
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Rigid body ID : %d", rigidBodyDesc->ID);
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Rigid body parent ID : %d", rigidBodyDesc->parentID);
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Parent offset : %3.2f,%3.2f,%3.2f", rigidBodyDesc->offsetx, rigidBodyDesc->offsety, rigidBodyDesc->offsetz);

    if (rigidBodyDesc->MarkerPositions != NULL && rigidBodyDesc->MarkerRequiredLabels != NULL)
    {
        for (int markerIdx = 0; markerIdx < rigidBodyDesc->nMarkers; ++markerIdx)
        {
            const MarkerData& markerPosition = rigidBodyDesc->MarkerPositions[markerIdx];
            const int markerRequiredLabel = rigidBodyDesc->MarkerRequiredLabels[markerIdx];
            
            RCLCPP_INFO(this->natNetHandler->get_logger(), "Marker: %d", markerIdx);
            RCLCPP_INFO(this->natNetHandler->get_logger(), "Position: %.2f, %.2f, %.2f", markerPosition[0], markerPosition[1], markerPosition[2]);
            
            if (markerRequiredLabel != 0)
            {
                RCLCPP_INFO(this->natNetHandler->get_logger(), "Required active label: %d", markerRequiredLabel);
            }
        }
    }
}

// Camera device descriptor
void MotionCaptureClient::processCamera(sCameraDescription* pCamera)
{
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Camera name : %s", pCamera->strName);
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Camera position (%3.2f, %3.2f, %3.2f)", pCamera->x, pCamera->y, pCamera->z);
    RCLCPP_INFO(this->natNetHandler->get_logger(), "Camera orientation (%3.2f, %3.2f, %3.2f, %3.2f)", pCamera->qx, pCamera->qy, pCamera->qz, pCamera->qw);
}
