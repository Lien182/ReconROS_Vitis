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

#define OUTPUT_WIDTH 600
#define OUTPUT_HEIGHT 1000

#define RO 0 // 8 Pixel Processing
#define NO 1 // 1 Pixel Processing

// Number of rows of input image to be stored
#define NUM_STORE_ROWS 100

// Number of rows of input image after which output image processing must start
#define START_PROC 50

#define RGBA 1
#define GRAY 0

// transform type 0-NN 1-BILINEAR
#define INTERPOLATION 1


#define XF_INTERPOLATION_TYPE XF_INTERPOLATION_BILINEAR
#define MAXDOWNSCALE 2
// transform type 0-AFFINE 1-PERSPECTIVE
#define TRANSFORM_TYPE 0
#define XF_USE_URAM false


#define FILTER_WIDTH 7
#define FILTER 7

#if NO
#define NPC1 XF_NPPC1
#endif
#if RO
#define NPC1 XF_NPPC8
#endif


void proc(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt,uint64_t pMessage, ap_uint<64> ram_out[640*480*3/8])
{
    ap_uint<64> ram_in[640*480*3/8];
    

    #pragma HLS stream variable=ram_in depth=32

   
    xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1> in_mat(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC3,    600,  1000, XF_NPPC1> resized_mat(600,  1000);
    xf::cv::Mat<XF_8UC4,    600,  1000, XF_NPPC1> tmp_mat(600,  1000);    
    xf::cv::Mat<XF_8UC4,    600,  1000, XF_NPPC1> out_mat(600,  1000);       


    float transform_matrix[9]= {    -2.405063291139244f, -3.797468354430384f,       1269.620253164558f,
                                    0.0f,                -8.354430379746846f,       2422.784810126584f,
                                    0.0f,                -0.007594936708860767f,    1.0f};


    #pragma HLS DATAFLOW

    MEM_READ(pMessage, ram_in, WIDTH*HEIGHT*3);
    int ramptr = 0;
    loop_read: for(int i = 0; i < WIDTH * HEIGHT / 8; i+=1)
    {
        ap_uint<24> pix;
        uint64_t tmp_ram = ram_in[ramptr++];

        pix.range(23,16) =  (tmp_ram >> 0) & 0xff;
        pix.range(15,8) = (tmp_ram >> 8) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 16) & 0xff;
        in_mat.write(i*8 + 0, pix);

        pix.range(23,16) =  (tmp_ram >> 24 )& 0xff;
        pix.range(15,8) = (tmp_ram >> 32) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 40) & 0xff;
        in_mat.write(i*8 + 1, pix);

        pix.range(23,16) =  (tmp_ram >> 48 )& 0xff;
        pix.range(15,8) = (tmp_ram >> 56) & 0xff;
        tmp_ram = ram_in[ramptr++];
        pix.range(7,0 )= (tmp_ram >> 0) & 0xff;    
        in_mat.write(i*8 + 2, pix);

        pix.range(23,16) =  (tmp_ram >> 8) & 0xff;
        pix.range(15,8) = (tmp_ram >> 16) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 24) & 0xff;
        in_mat.write(i*8 + 3, pix);

        pix.range(23,16) =  (tmp_ram >> 32) & 0xff;
        pix.range(15,8) = (tmp_ram >> 40) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 48) & 0xff;
        in_mat.write(i*8 + 4, pix);

        pix.range(23,16) =  (tmp_ram >> 56) & 0xff;
        tmp_ram = ram_in[ramptr++];
        pix.range(15,8) = (tmp_ram >> 0) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 8) & 0xff;
        in_mat.write(i*8 + 5, pix);

        pix.range(23,16) =  (tmp_ram >> 16) & 0xff;
        pix.range(15,8) = (tmp_ram >> 24) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 32) & 0xff;
        in_mat.write(i*8 + 6, pix);

        pix.range(23,16) =  (tmp_ram >> 40) & 0xff;
        pix.range(15,8) = (tmp_ram >> 48) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 56) & 0xff;
        in_mat.write(i*8 + 7, pix);

    }
    
    resize<XF_INTERPOLATION_TYPE, XF_8UC3, HEIGHT, WIDTH, 600, 1000, XF_NPPC1, MAXDOWNSCALE>(in_mat, resized_mat);


    for(int i = 0; i < OUTPUT_WIDTH * OUTPUT_HEIGHT; i++)
    {
        ap_uint<32> output_pix; 
        ap_uint<24> input_pix = resized_mat.read (i*8 + 0);
        output_pix.range(23, 0) = input_pix;
        tmp_mat.write(i*8 + 6, output_pix);
    }


    xf::cv::warpTransform<NUM_STORE_ROWS, START_PROC, TRANSFORM_TYPE, INTERPOLATION, XF_8UC4, 600, 1000, NPC1, XF_USE_URAM>(tmp_mat, out_mat, transform_matrix);
    

    ramptr = 0;
    loop_write: for(int i = 0; i < WIDTH * HEIGHT / 8; i+=1)
    {
        ap_uint<24> pix;
        ap_uint<64> tmp_ram;
        
        pix = out_mat.read (i*8 + 0);
        tmp_ram.range( 7, 0) = pix.range(23,16);
        tmp_ram.range(15, 8) = pix.range(15,8);
        tmp_ram.range(23,16) = pix.range(7,0);

        pix = out_mat.read (i*8 + 1);
        tmp_ram.range(31,24) = pix.range(23,16);
        tmp_ram.range(39,32) = pix.range(15,8);
        tmp_ram.range(47,40) = pix.range(7,0);

        pix = out_mat.read (i*8 + 2);
        tmp_ram.range(55,48) = pix.range(23,16);
        tmp_ram.range(63,56) = pix.range(15,8);
        ram_out[ramptr++] = tmp_ram;
        tmp_ram.range( 7, 0) = pix.range(7,0 );

        pix = out_mat.read (i*8 + 3);
        tmp_ram.range(15, 8) = pix.range(23,16);
        tmp_ram.range(23,16) = pix.range(15,8);
        tmp_ram.range(31,24) = pix.range(7,0);

        pix = out_mat.read (i*8 + 4);
        tmp_ram.range(39,32) = pix.range(23,16);
        tmp_ram.range(47,40) = pix.range(15,8);
        tmp_ram.range(55,48) = pix.range(7,0);

        pix = out_mat.read (i*8 + 5);
        tmp_ram.range(63,56) = pix.range(23,16);
        ram_out[ramptr++] = tmp_ram;
        tmp_ram.range( 7, 0) = pix.range(15,8);
        tmp_ram.range(15, 8) = pix.range(7,0);

        pix = out_mat.read (i*8 + 6);
        tmp_ram.range(23,16) = pix.range(23,16);
        tmp_ram.range(31,24) = pix.range(15,8);
        tmp_ram.range(39,32) = pix.range(7,0);

        pix = out_mat.read (i*8 + 7);
        tmp_ram.range(47,40) = pix.range(23,16);
        tmp_ram.range(55,48) = pix.range(15,8);
        tmp_ram.range(63,56) = pix.range(7,0);
        ram_out[ramptr++] = tmp_ram;
    }

}

THREAD_ENTRY() {
    ap_uint<64> ram_out[640*480*3/8];
    uint64_t output_payload_addr[1];


    //not tested
    //#pragma HLS RESOURCE variable=ram_out core=XPM_MEMORY uram

    THREAD_INIT();
	uint64_t initdata = GET_INIT_DATA();

    uint64_t pMessage_OutputTopic = MEMORY_GETOBJECTADDR(rcameraimageprojection_output_image);
    MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + pMessage_OutputTopic, output_payload_addr, 8);
                
    while(1)
    {
        uint64_t payload_addr[1];
        uint64_t pMessage =  ROS_SUBSCRIBE_TAKE(rcameraimageprojection_subdata, rcameraimageprojection_camera_image);

        MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + pMessage, payload_addr, 8);            
        proc(memif_hwt2mem, memif_mem2hwt, payload_addr[0], ram_out);

        MEM_WRITE(ram_out, output_payload_addr[0], WIDTH*HEIGHT*3);
        ROS_PUBLISH(rcameraimageprojection_pubout, rcameraimageprojection_output_image);
    }

    return;
}
