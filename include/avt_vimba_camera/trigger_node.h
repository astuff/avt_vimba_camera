//#include <avt_vimba_camera/avt_vimba_camera.h>
#include <VimbaC/Include/VimbaC.h>


// struct representing an Action Command
typedef struct tActionCommand
{
    VmbUint32_t     mDeviceKey;
    VmbUint32_t     mGroupKey;
    VmbUint32_t     mGroupMask;

} tActionCommand;

typedef enum tFeatureOwner
{
    eFeatureOwnerUnknown     = 0,
    eFeatureOwnerSystem      = 1,
    eFeatureOwnerInterface   = 2,
    eFeatureOwnerCamera      = 3

} tFeatureOwner;

class cActionCommands
{
    private:
        //VimbaSystem&    mSystem;
        //InterfacePtr    mInterface;
        //CameraPtr       mCamera;

        //cFrameObserver* mFrameObserver;

        bool            mSystemFlag;
        bool            mInterfaceFlag;
        bool            mCameraFlag;
        bool            mStreamingFlag;

    public:
        //
        // Constructor
        //
        cActionCommands();

        //
        // Destructor
        //
        ~cActionCommands();

        //
        // Helper method to read feature values of any type
        //
        // Parameters:
	//
};

VmbErrorType GetFeatureValue( const char* aName, tFeatureOwner aOwner, VmbFeatureDataType* aType, void* aValue, size_t* aSize );

VmbErrorType SetFeatureValue( const char* aName, tFeatureOwner aOwner, VmbFeatureDataType aType, void* aValue, size_t aSize );


VmbError_t PrepareActionCommand( VmbHandle_t aHandle, tActionCommand* aCommand );

VmbErrorType RunCommand( const char* aName, tFeatureOwner aOwner );

void FailureShutdown();


