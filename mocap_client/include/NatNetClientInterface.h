#ifndef MOCAP_NATNET_CLIENT_H
#define MOCAP_NATNET_CLIENT_H

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include <NatNetHandlerInterface.h>

void NATNET_CALLCONV dataFrameHandler(sFrameOfMocapData* data, void* pUserData);

class MotionCaptureClient : private NatNetClient
{
public:
    // Constructor and Destructor
    MotionCaptureClient(NatnetDriverNode* publisher);
    ~MotionCaptureClient();

    // Connection methods
    int connectToServer();
    void disconnectFromServer();

    // Getter methods
    NatnetDriverNode* getPublisher();

    // Data handling methods
    void sendRigidBodyMessage(double currentSecsSinceEpoch, sRigidBodyData* bodies, int nRigidBodies); 

    // Recording related methods
    bool setRecordName(const std::string& recordName);
    bool startRecording();
    bool stopRecording();

    // Other
    void printLogSeparator(const rclcpp::Logger& logger, int count, char separatorChar);  

private:
    // Private member variables
    sNatNetClientConnectParams connectionParams;
    sServerDescription serverDescription;
    sDataDescriptions* dataDefinitions;
    int analogSamplesPerMocapFrame = 0;
    NatnetDriverNode* natNetHandler;
    bool isRecordingStarted = false;
    
    // Private member functions
    void initializeConnectionParams(NatnetDriverNode* publisher);

    // Setter methods
    void setFrameCallback();

    // Getter methods
    void getDataDescription();
    void getVersionInfo();
    void getServerInfo();
    void getFrameRate();
    void getAnalogSamples();
    sServerDescription getServerDescription();
    void processDataDefinitions(sDataDescriptions* dataDefinitions);
    void processMarkerSet(sMarkerSetDescription* markerSetDesc);
    void processRigidBody(sRigidBodyDescription* rigidBodyDesc);
    void processCamera(sCameraDescription* cameraDesc);

    //TODO
    //void processSkeleton(sSkeletonDescription* skeletonDesc);
    //void processForcePlate(sForcePlateDescription* forceplateDesc);
    //void processPeripheralDevice(sDeviceDescription* deviceDesc);
};
 
#endif
