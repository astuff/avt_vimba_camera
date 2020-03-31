#include "ros/ros.h"
#include "std_msgs/String.h"
#include <avt_vimba_camera/trigger_node.h>
#include <VimbaC/Include/VimbaC.h>

#include <sstream>
#include <stdio.h>
#include <stdlib.h>



VmbError_t lError = VmbErrorSuccess;
tActionCommand aCommand;
int triggerFreq = 1;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "trigger_node");

	ros::NodeHandle nh;
	
	VmbStartup();

	aCommand.mDeviceKey = 1;
	aCommand.mGroupKey = 1;
	aCommand.mGroupMask = 1;

	lError = PrepareActionCommand( gVimbaHandle, &aCommand );

	if(argc > 0){
		//sscanf(argv[1], "%d", &triggerFreq);
		triggerFreq = atoi(argv[1]);
	}


	ros::Rate loop_rate(triggerFreq);

	while (ros::ok())
	{
		if( VmbErrorSuccess == lError )
            	{
                	// send Action Command by calling command feature
                	lError = PrepareActionCommand( gVimbaHandle, &aCommand );

			//lError = RunCommand( "ActionCommand", eFeatureOwnerSystem );
                	//runCommand("ActionCommand")
			lError = VmbFeatureCommandRun( gVimbaHandle, "ActionCommand" );

			if( VmbErrorSuccess != lError )
			{
				return 0;
			}
		}
		loop_rate.sleep();
		//printf("time: %f \n", loop_rate.cycleTime().toSec());
	}

	return 0;
}

VmbError_t PrepareActionCommand( VmbHandle_t aHandle, tActionCommand* aCommand )
{
    VmbError_t lError = VmbErrorSuccess;

    // check parameter
    if( (NULL == aHandle) || (NULL == aCommand) )
    {
        printf( "[F]...Invalid parameter given.\n" );
        return VmbErrorBadParameter;
    }

    // set device key
    lError = VmbFeatureIntSet( aHandle, "ActionDeviceKey", aCommand->mDeviceKey );
    if( VmbErrorSuccess != lError )
    {
        printf( "[F]...Could not set ActionDeviceKey. Reason: %i\n", lError );
        //FailureShutdown( aHandle );
        //return lError;
    }
    // set group key
    lError = VmbFeatureIntSet( aHandle, "ActionGroupKey", aCommand->mGroupKey );
    if( VmbErrorSuccess != lError )
    {
        printf( "[F]...Could not set ActionGroupKey. Reason: %i\n", lError );
        //FailureShutdown( aHandle );
        //return lError;
    }

    // set group mask
    lError = VmbFeatureIntSet( aHandle, "ActionGroupMask", aCommand->mGroupMask );
    if( VmbErrorSuccess != lError )
    {
        printf( "[F]...Could not set ActionGroupMask. Reason: %i\n", lError );
        //FailureShutdown( aHandle );
        //return lError;
    }

    printf( "......Action Command has been set (%i,%i,%i)\n", aCommand->mDeviceKey, aCommand->mGroupKey, aCommand->mGroupMask );

    return lError;
}


