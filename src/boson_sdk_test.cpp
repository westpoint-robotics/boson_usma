#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
extern "C" {
#include "EnumTypes.h"
#include "UART_Connector.h"
#include "Client_API.h"
}

int main() {
    std::cout << "Hello World\n";
    
	// Connect to the camera. 16 is ttyACM0.
	int32_t dev = 16;
	int32_t baud = 921600;
	
	FLR_RESULT result;
	result = Initialize(dev, baud); //COM6, 921600 baud (port_number=5 for COM6) 
	printf("Initialize: 0x%08X\n", result);
	if (result)
	{
		printf("Failed to initialize, exiting.\n");
		Close();
		return -1;
	}
	printf("\n");
	
	// Retrieve the Camera SN and print it
	printf("CameraSN: ");
	uint32_t camera_sn;
	result = bosonGetCameraSN(&camera_sn);
	if (result)
	{
		printf("Failed with status 0x%08X, exiting.\n",result);
		Close();
		return -1;
	}
	printf(" %d \n", camera_sn);
	printf("\n");
	
	
	FLR_DVO_TYPE_E dvo_src;
	result = dvoGetType(&dvo_src);
	printf("DVO Source:  0x%08X -- 0x%08X \n", result, dvo_src);
	
	
	printf("\n");
	uint32_t major, minor, patch;
	printf("SoftwareRev:  ");
	result = bosonGetSoftwareRev(&major, &minor, &patch);
	if (result)
	{
		printf("Failed with status 0x%08X, exiting.\n",result);
		Close();
		return -1;
	}
	printf(" %u.%u.%u \n", major,minor,patch);
	
	printf("\n");
	FLR_BOSON_SENSOR_PARTNUMBER_T part_num;
	printf("PartNum: ");
	result = bosonGetSensorPN(&part_num);
	if (result)
	{
		printf("Failed with status 0x%08X, exiting.\n",result);
		Close();
		return -1;
	}
	printf(" \"%s\"", part_num.value);
	int idx =0;
	for (idx=0; idx<sizeof(part_num); idx++)
	{
		uint8_t tempchar = part_num.value[idx];
		if ( !(idx%16) )
			printf("\n\t");
		if (tempchar>=32 && tempchar<=125)
		{
			printf(" \"%c\"",tempchar);
		}
		else
		{
			printf("  %02X",tempchar);
		}
	}
	printf("\n");
	   
    
    
    
    
    return 0;
}