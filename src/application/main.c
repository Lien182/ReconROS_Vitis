#define  _GNU_SOURCE

#include "reconos.h"
#include "reconos_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#define OUTPUT_IMAGE_HEIGHT 600
#define OUTPUT_IMAGE_WIDTH  1000


void init_msg(void)
{
  	rcameraimageprojection_output_image->height = OUTPUT_IMAGE_HEIGHT;
  	rcameraimageprojection_output_image->width = OUTPUT_IMAGE_WIDTH;
  	rcameraimageprojection_output_image->encoding.data = "bgr8";
	rcameraimageprojection_output_image->encoding.size = 4;  
	rcameraimageprojection_output_image->encoding.capacity = 5;  
  	rcameraimageprojection_output_image->is_bigendian = 0;
  	rcameraimageprojection_output_image->step = OUTPUT_IMAGE_WIDTH*3;
  	rcameraimageprojection_output_image->data.data = malloc(OUTPUT_IMAGE_HEIGHT*OUTPUT_IMAGE_WIDTH*3);
	rcameraimageprojection_output_image->data.size = OUTPUT_IMAGE_HEIGHT*OUTPUT_IMAGE_WIDTH*3;
	rcameraimageprojection_output_image->data.capacity = OUTPUT_IMAGE_HEIGHT*OUTPUT_IMAGE_WIDTH*3;
  
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
	if(argc != 2)
	{
		printf("Usage: ./reconrosvitisexample <sw/hw> \n");
		return -1;
	}
	reconos_init();
	reconos_app_init();

	init_msg();

	signal(SIGINT, exit_signal);
	signal(SIGTERM, exit_signal);
	signal(SIGABRT, exit_signal);
	
	if(!strcmp(argv[1], "sw"))
	{
		printf("Start execution in sw \n");
		reconos_thread_create_swt_projection(0,0);		
	}

	else
	{
		printf("Start execution in hw \n");	
		reconos_thread_create_hwt_projection(0);
	
	}


	while(1)
	{
		sleep(1);
	}

	reconos_app_cleanup();
	reconos_cleanup();
	return 0;
}
