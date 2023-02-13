#include "reconos_thread.h"
#include "reconos_calls.h"

#include "hls_stream.h"
#include <ap_int.h>

#include <common/xf_common.hpp>
#include <common/xf_utility.hpp>
#include <imgproc/xf_warp_transform.hpp>
#include <imgproc/xf_resize.hpp>

#include <sensor_msgs/msg/image.h>

using namespace std;

#define HEIGHT 480
#define WIDTH 640
#define INTYPE uint64_t

#define OUTPUT_WIDTH 640
#define OUTPUT_HEIGHT 480

#define RO 0 // 8 Pixel Processing
#define NO 1 // 1 Pixel Processing

// Number of rows of input image to be stored
#define NUM_STORE_ROWS 320

// Number of rows of input image after which output image processing must start
#define START_PROC 250

#define RGBA 1
#define GRAY 0

// transform type 0-NN 1-BILINEAR
#define INTERPOLATION 1


#define XF_INTERPOLATION_TYPE XF_INTERPOLATION_BILINEAR
#define MAXDOWNSCALE 2
// transform type 0-AFFINE 1-PERSPECTIVE
#define TRANSFORM_TYPE 1
#define XF_USE_URAM false

#if NO
#define NPC1 XF_NPPC1
#endif
#if RO
#define NPC1 XF_NPPC8
#endif


#define WARPTRANSFORM_LATENCY_OFFSET (START_PROC)
#define RESIZE_LATENCY_OFFSET 0
#define OPERATION_LATENCY (WARPTRANSFORM_LATENCY_OFFSET + RESIZE_LATENCY_OFFSET)



void proc(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt,uint64_t pMessage, uint64_t pMessageOut)
{
    ap_uint<64> ram_in [WIDTH*3/8];
    ap_uint<64> ram_out[OUTPUT_WIDTH*3/8];

    //#pragma HLS stream variable=ram_in depth=32

   
    xf::cv::Mat<XF_8UC4, HEIGHT, WIDTH, XF_NPPC1> in_mat(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC4, HEIGHT, WIDTH, XF_NPPC1> tmp_mat(HEIGHT,WIDTH);  
    xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1> resizing_mat(HEIGHT, WIDTH);      
    xf::cv::Mat<XF_8UC3, OUTPUT_HEIGHT,  OUTPUT_WIDTH, XF_NPPC1> out_mat(OUTPUT_HEIGHT,  OUTPUT_WIDTH);       


    //float transform_matrix[9]= {    -2.405063291139244f, -3.797468354430384f,       1269.620253164558f,
    //                                0.0f,                -8.354430379746846f,       2422.784810126584f,
    //                                0.0f,                -0.007594936708860767f,    1.0f};


    float transform_matrix[9]= {   -0.64967f,     0.30239f,   -58.21053f,
                                    0.00000f,     0.12442f, -241.15789f,
                                    0.00000f,     0.00094f,    -0.83158f};


    #pragma HLS DATAFLOW

    for(int lines = 0; lines < (OUTPUT_HEIGHT + OPERATION_LATENCY + 1); lines++)
    {
        int ramptr = 0;
        
        if(lines < HEIGHT)
        {
            MEM_READ(pMessage + lines*WIDTH*3, ram_in, WIDTH*3);
            
            loop_read: for(int i = 0; i < WIDTH / 8; i++)
            {
                ap_uint<24> pix;
                uint64_t tmp_ram = ram_in[ramptr++];

                pix.range(23,16) =  (tmp_ram >> 0) & 0xff;
                pix.range(15,8) = (tmp_ram >> 8) & 0xff;
                pix.range(7,0 )= (tmp_ram >> 16) & 0xff;
                in_mat.write(lines*WIDTH + i*8 + 0, pix);

                pix.range(23,16) =  (tmp_ram >> 24 )& 0xff;
                pix.range(15,8) = (tmp_ram >> 32) & 0xff;
                pix.range(7,0 )= (tmp_ram >> 40) & 0xff;
                in_mat.write(lines*WIDTH + i*8 + 1, pix);

                pix.range(23,16) =  (tmp_ram >> 48 )& 0xff;
                pix.range(15,8) = (tmp_ram >> 56) & 0xff;
                tmp_ram = ram_in[ramptr++];
                pix.range(7,0 )= (tmp_ram >> 0) & 0xff;    
                in_mat.write(lines*WIDTH + i*8 + 2, pix);

                pix.range(23,16) =  (tmp_ram >> 8) & 0xff;
                pix.range(15,8) = (tmp_ram >> 16) & 0xff;
                pix.range(7,0 )= (tmp_ram >> 24) & 0xff;
                in_mat.write(lines*WIDTH + i*8 + 3, pix);

                pix.range(23,16) =  (tmp_ram >> 32) & 0xff;
                pix.range(15,8) = (tmp_ram >> 40) & 0xff;
                pix.range(7,0 )= (tmp_ram >> 48) & 0xff;
                in_mat.write(lines*WIDTH + i*8 + 4, pix);

                pix.range(23,16) =  (tmp_ram >> 56) & 0xff;
                tmp_ram = ram_in[ramptr++];
                pix.range(15,8) = (tmp_ram >> 0) & 0xff;
                pix.range(7,0 )= (tmp_ram >> 8) & 0xff;
                in_mat.write(lines*WIDTH + i*8 + 5, pix);

                pix.range(23,16) =  (tmp_ram >> 16) & 0xff;
                pix.range(15,8) = (tmp_ram >> 24) & 0xff;
                pix.range(7,0 )= (tmp_ram >> 32) & 0xff;
                in_mat.write(lines*WIDTH + i*8 + 6, pix);

                pix.range(23,16) =  (tmp_ram >> 40) & 0xff;
                pix.range(15,8) = (tmp_ram >> 48) & 0xff;
                pix.range(7,0 )= (tmp_ram >> 56) & 0xff;
                in_mat.write(lines*WIDTH + i*8 + 7, pix);

            }
        }
        
    
        
        if(lines > (OPERATION_LATENCY))
        {
            ramptr = 0;
            loop_write: for(int i = 0; i < OUTPUT_WIDTH / 8; i++)
            {
                ap_uint<24> pix;
                ap_uint<64> tmp_ram;
                
                pix = tmp_mat.read (lines*OUTPUT_WIDTH + i*8 + 0);
                tmp_ram.range( 7, 0) = pix.range(23,16);
                tmp_ram.range(15, 8) = pix.range(15,8);
                tmp_ram.range(23,16) = pix.range(7,0);

                pix = tmp_mat.read (lines*OUTPUT_WIDTH + i*8 + 1);
                tmp_ram.range(31,24) = pix.range(23,16);
                tmp_ram.range(39,32) = pix.range(15,8);
                tmp_ram.range(47,40) = pix.range(7,0);

                pix = tmp_mat.read (lines*OUTPUT_WIDTH + i*8 + 2);
                tmp_ram.range(55,48) = pix.range(23,16);
                tmp_ram.range(63,56) = pix.range(15,8);
                ram_out[ramptr++] = tmp_ram;
                tmp_ram.range( 7, 0) = pix.range(7,0 );

                pix = tmp_mat.read (lines*OUTPUT_WIDTH + i*8 + 3);
                tmp_ram.range(15, 8) = pix.range(23,16);
                tmp_ram.range(23,16) = pix.range(15,8);
                tmp_ram.range(31,24) = pix.range(7,0);

                pix = tmp_mat.read (lines*OUTPUT_WIDTH + i*8 + 4);
                tmp_ram.range(39,32) = pix.range(23,16);
                tmp_ram.range(47,40) = pix.range(15,8);
                tmp_ram.range(55,48) = pix.range(7,0);

                pix = tmp_mat.read (lines*OUTPUT_WIDTH + i*8 + 5);
                tmp_ram.range(63,56) = pix.range(23,16);
                ram_out[ramptr++] = tmp_ram;
                tmp_ram.range( 7, 0) = pix.range(15,8);
                tmp_ram.range(15, 8) = pix.range(7,0);

                pix = tmp_mat.read (lines*OUTPUT_WIDTH + i*8 + 6);
                tmp_ram.range(23,16) = pix.range(23,16);
                tmp_ram.range(31,24) = pix.range(15,8);
                tmp_ram.range(39,32) = pix.range(7,0);

                pix = tmp_mat.read (lines*OUTPUT_WIDTH + i*8 + 7);
                tmp_ram.range(47,40) = pix.range(23,16);
                tmp_ram.range(55,48) = pix.range(15,8);
                tmp_ram.range(63,56) = pix.range(7,0);
                ram_out[ramptr++] = tmp_ram;
            }

            MEM_WRITE(ram_out, pMessageOut + (lines-OPERATION_LATENCY-1) * OUTPUT_WIDTH*3, OUTPUT_WIDTH*3);
        }
   
    }

    
    xf::cv::warpTransform<NUM_STORE_ROWS, START_PROC, TRANSFORM_TYPE, INTERPOLATION, XF_8UC4, HEIGHT, WIDTH, NPC1, XF_USE_URAM>(in_mat, tmp_mat, transform_matrix);
    
    /*
    for(int i = 0; i < HEIGHT * WIDTH; i++)
    {
        ap_uint<24> output_pix; 
        ap_uint<32> input_pix = tmp_mat.read (i);
        output_pix = input_pix.range(23, 0);
        resizing_mat.write(i, output_pix);
    }

    resize<XF_INTERPOLATION_TYPE, XF_8UC3, HEIGHT, WIDTH, OUTPUT_HEIGHT, OUTPUT_WIDTH, XF_NPPC1, MAXDOWNSCALE>( resizing_mat , out_mat);

    */
   

}

THREAD_ENTRY() {
    
    uint64_t output_payload_addr[1];

    THREAD_INIT();
	uint64_t initdata = GET_INIT_DATA();

    uint64_t pMessage_OutputTopic = MEMORY_GETOBJECTADDR(rcameraimageprojection_output_image);
    MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + pMessage_OutputTopic, output_payload_addr, 8);
                
    while(1)
    {
        uint64_t payload_addr[1];
        uint64_t pMessage =  ROS_SUBSCRIBE_TAKE(rcameraimageprojection_subdata, rcameraimageprojection_camera_image);

        MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + pMessage, payload_addr, 8);            
        proc(memif_hwt2mem, memif_mem2hwt, payload_addr[0], output_payload_addr[0]);

        ROS_PUBLISH(rcameraimageprojection_pubout, rcameraimageprojection_output_image);
    }

    return;
}
