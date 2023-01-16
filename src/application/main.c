#define  _GNU_SOURCE

#include "reconos.h"
#include "reconos_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#define DEFAULT_IMAGE_HEIGHT 1024
#define DEFAULT_IMAGE_WIDTH  1280


void init_msg(void)
{
  	rgaussianblur_output_image->height = 480;
  	rgaussianblur_output_image->width = 640;
  	rgaussianblur_output_image->encoding.data = "bgr8";
	rgaussianblur_output_image->encoding.size = 4;  
	rgaussianblur_output_image->encoding.capacity = 5;  
  	rgaussianblur_output_image->is_bigendian = 0;
  	rgaussianblur_output_image->step = 640*3;
  	rgaussianblur_output_image->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
	rgaussianblur_output_image->data.size = 480*640*3;
	rgaussianblur_output_image->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;
  
	return;
}



static void exit_signal(int sig) 
{
	//ReconROS_Executor_Terminate(&reconros_executor);
	reconos_cleanup();
	printf("[ReconROS] aborted\n");
	exit(0);
}

int main(int argc, char **argv) 
{
	reconos_init();
	reconos_app_init();

	init_msg();

	signal(SIGINT, exit_signal);
	signal(SIGTERM, exit_signal);
	signal(SIGABRT, exit_signal);
	

	reconos_thread_create_hwt_gaussian(0);

	while(1)
	{
		sleep(1);

		}

	reconos_app_cleanup();
	reconos_cleanup();
	return 0;
}
