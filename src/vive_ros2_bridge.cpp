#include <string>
#include <stdarg.h>
#include <openvr.h>

//TODO: proper linux compatibility
#ifdef __linux__
#include <linuxcompathack.h>
#endif

// ros2 headers
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


static bool g_bPrintf = true;
class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher():Node("vive_ros2_bridge")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  void Publish(const std::string deviceName,
                        const vr::HmdVector3_t position, 
                        const vr::HmdQuaternion_t quaternion){
  handle_vive_pose(deviceName, position, quaternion);
  }

private:
  void handle_vive_pose(const std::string deviceName,
                        const vr::HmdVector3_t position, 
                        const vr::HmdQuaternion_t quaternion)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = deviceName;

    t.transform.translation.x = position.v[0];
    t.transform.translation.y = position.v[1];
    t.transform.translation.z = position.v[2];

    // tf2::Quaternion q;
    // q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = quaternion.x;
    t.transform.rotation.y = quaternion.y;
    t.transform.rotation.z = quaternion.z;
    t.transform.rotation.w = quaternion.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

//-----------------------------------------------------------------------------
// Purpose:
//------------------------------------------------------------------------------
class CMainApplication
{
public:
	CMainApplication( int argc, char *argv[] );
	virtual ~CMainApplication();

	bool BInit();
  bool InitROS(int argc, char *argv[]);

	void Shutdown();

	void RunMainLoop();

	vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);
  vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix);
  void printPositionalData();
  void printDevicePositionalData(const char* deviceName, vr::HmdMatrix34_t poseMatrix, vr::HmdVector3_t position, vr::HmdQuaternion_t quaternion);
	
	bool HandleInput();
	void ProcessVREvent( const vr::VREvent_t & event );

private: 
	vr::IVRSystem *m_pHMD;
	std::string m_strDriver;
	std::string m_strDisplay;
	// vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
	// Matrix4 m_rmat4DevicePose[ vr::k_unMaxTrackedDeviceCount ];
	bool m_rbShowTrackedDevice[ vr::k_unMaxTrackedDeviceCount ];

	float m_fNearClip;
	float m_fFarClip;

	int m_iTrackedControllerCount;
	int m_iTrackedControllerCount_Last;
	int m_iValidPoseCount;
	int m_iValidPoseCount_Last;
	std::string m_strPoseClasses;    
	char m_rDevClassChar[ vr::k_unMaxTrackedDeviceCount ];  
  FramePublisher* m_publisher;
};

//-----------------------------------------------------------------------------
// Purpose: terminal output function
//-----------------------------------------------------------------------------
void dprintf( const char *fmt, ... )
{
	va_list args;
	char buffer[ 2048 ];

	va_start( args, fmt );
	vsprintf_s( buffer, fmt, args );
	va_end( args );

	if ( g_bPrintf )
		printf( "%s", buffer );

	OutputDebugStringA( buffer );
}

//-----------------------------------------------------------------------------
// Purpose: Constructor
//-----------------------------------------------------------------------------
CMainApplication::CMainApplication( int argc, char *argv[] )
	: m_pHMD( NULL )
{
  InitROS(argc, argv);
  m_publisher = new FramePublisher();

	// for( int i = 1; i < argc; i++ )
	// {
  //  if( !stricmp( argv[i], "-noprintf" ) )
	// 	{
	// 		g_bPrintf = false;
	// 	}
	// }
	// other initialization tasks are done in BInit
};

//-----------------------------------------------------------------------------
// Purpose: Destructor
//-----------------------------------------------------------------------------
CMainApplication::~CMainApplication()
{
	// work is done in Shutdown
	dprintf( "Shutdown" );
}

//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//			into a std::string
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
	if( unRequiredBufferLen == 0 )
		return "";

	char *pchBuffer = new char[ unRequiredBufferLen ];
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
	std::string sResult = pchBuffer;
	delete [] pchBuffer;
	return sResult;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication::BInit()
{
	// Loading the SteamVR Runtime
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);

	if ( eError != vr::VRInitError_None )
	{
		m_pHMD = NULL;
		char buf[1024];
		sprintf_s( buf, sizeof( buf ), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError ) );
		return false;
	}

	m_strDriver = "No Driver";
	m_strDisplay = "No Display";

	m_strDriver = GetTrackedDeviceString( m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String );
	m_strDisplay = GetTrackedDeviceString( m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String );

	return true;
}


bool CMainApplication::InitROS(int argc, char *argv[])
{
  
  auto logger = rclcpp::get_logger("logger");
  rclcpp::init(argc, argv);//, argv);
  // rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
  return 0;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::Shutdown()
{
	if( m_pHMD )
	{
		vr::VR_Shutdown();
		m_pHMD = NULL;
    rclcpp::shutdown();
	}
}

//-----------------------------------------------------------------------------
// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

vr::HmdQuaternion_t CMainApplication::GetRotation(vr::HmdMatrix34_t matrix) {
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

vr::HmdVector3_t CMainApplication::GetPosition(vr::HmdMatrix34_t matrix) {
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}


//-----------------------------------------------------------------------------
// Purpose: Prints the timestamped data in proper format(x,y,z).
//-----------------------------------------------------------------------------
 
void CMainApplication::printDevicePositionalData(const char * deviceName, vr::HmdMatrix34_t posMatrix, vr::HmdVector3_t position, vr::HmdQuaternion_t quaternion)
{
    dprintf("\n%lld, %s, x = %.5f, y = %.5f, z = %.5f, qw = %.5f, qx = %.5f, qy = %.5f, qz = %.5f",
        0, deviceName,
        position.v[0], position.v[1], position.v[2],
        quaternion.w, quaternion.x, quaternion.y, quaternion.z); 
}


//-----------------------------------------------------------------------------
// Purpose: Prints out position (x,y,z) and rotation (qw,qx,qy,qz) into the console.
// Contain warnings of unhandled enumerations. 
//-----------------------------------------------------------------------------
void CMainApplication::printPositionalData()
{
 
    // Process SteamVR device states
    for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
    {
        if (!m_pHMD->IsTrackedDeviceConnected(unDevice))
            continue;
 
        vr::VRControllerState_t state;
        if (m_pHMD->GetControllerState(unDevice, &state))//, sizeof(state)))
        {
            vr::TrackedDevicePose_t trackedDevicePose;
            vr::TrackedDevicePose_t trackedControllerPose; 
            vr::VRControllerState_t controllerState;            
            vr::HmdMatrix34_t poseMatrix;
            vr::HmdVector3_t position;
            vr::HmdQuaternion_t quaternion;
            vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);
 
            switch (trackedDeviceClass) {
            case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
                vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);                
                // print positiona data for the HMD.
                poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
                position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
                quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
 
                // printDevicePositionalData("HMD", poseMatrix, position, quaternion);
 
                break;
 
            // case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:                
            //     vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
            //     // print positiona data for a general vive tracker.
            //     break;
 
            case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
                // vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState, sizeof(controllerState), &trackedControllerPose);
                vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState, &trackedControllerPose);
                poseMatrix = trackedControllerPose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
                position = GetPosition(trackedControllerPose.mDeviceToAbsoluteTracking);
                quaternion = GetRotation(trackedControllerPose.mDeviceToAbsoluteTracking);
 
                auto trackedControllerRole = vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice);
                std::string whichHand = "";
                if (trackedControllerRole == vr::TrackedControllerRole_LeftHand)
                {
                    whichHand = "LeftHand";
                }
                else if (trackedControllerRole == vr::TrackedControllerRole_RightHand)
                {
                    whichHand = "RightHand";
                }
 
                switch (trackedControllerRole)
                {
                case vr::TrackedControllerRole_Invalid:
                    // invalid
                    break;
 
                case vr::TrackedControllerRole_LeftHand:
                case vr::TrackedControllerRole_RightHand:
                    // printDevicePositionalData(whichHand.c_str(), poseMatrix, position, quaternion);
                    m_publisher->Publish(whichHand, position, quaternion);
 
                    break;
                }
                break;
            }
 
        }
    }
 
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication::HandleInput()
{
	bool bRet = false;


	// Process SteamVR events
	vr::VREvent_t event;
	while( m_pHMD->PollNextEvent( &event, sizeof( event ) ) )
	{
		ProcessVREvent( event );
		//printf("what is here?");
	}
	
	printPositionalData();

	// Process SteamVR controller state
	for( vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++ )
	{
		vr::VRControllerState_t state;
		if( m_pHMD->GetControllerState( unDevice, &state ) )
		{
			m_rbShowTrackedDevice[ unDevice ] = state.ulButtonPressed == 0;
		}
	}

	return bRet;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::RunMainLoop()
{
	bool bQuit = false;
	while ( !bQuit )
	{
		bQuit = HandleInput();
	}
}

//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------
void CMainApplication::ProcessVREvent( const vr::VREvent_t & event )
{
	switch( event.eventType )
	{
	case vr::VREvent_TrackedDeviceActivated:
		{
			//SetupRenderModelForTrackedDevice( event.trackedDeviceIndex );
			dprintf( "Device %u attached. Setting up render model.\n", event.trackedDeviceIndex );
		}
		break;
	case vr::VREvent_TrackedDeviceDeactivated:
		{
			dprintf( "Device %u detached.\n", event.trackedDeviceIndex );
		}
		break;
	case vr::VREvent_TrackedDeviceUpdated:
		{
			dprintf( "Device %u updated.\n", event.trackedDeviceIndex );
		}
		break;
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int main(int argc, char *argv[]){
	CMainApplication *pMainApplication = new CMainApplication( argc, argv );

	if (!pMainApplication->BInit())
	{
		pMainApplication->Shutdown();
		return 1;
	}
	pMainApplication->RunMainLoop();
	pMainApplication->Shutdown();
	return 0;
}
