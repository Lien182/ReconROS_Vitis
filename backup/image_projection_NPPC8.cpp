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

#define OUTPUT_WIDTH 1000
#define OUTPUT_HEIGHT 600

#define RO 1 // 8 Pixel Processing
#define NO 0 // 1 Pixel Processing

// Number of rows of input image to be stored
#define NUM_STORE_ROWS 300

// Number of rows of input image after which output image processing must start
#define START_PROC 100

#define RGBA 1
#define GRAY 0

// transform type 0-NN 1-BILINEAR
#define INTERPOLATION 1


#define XF_INTERPOLATION_TYPE XF_INTERPOLATION_BILINEAR
#define MAXDOWNSCALE 2
// transform type 0-AFFINE 1-PERSPECTIVE
#define TRANSFORM_TYPE 1
#define XF_USE_URAM false


#define FILTER_WIDTH 7
#define FILTER 7

#if NO
#define NPC1 XF_NPPC1
#endif
#if RO
#define NPC1 XF_NPPC8
#endif


void proc(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt, ap_uint<64> ram_in[640*480*3/8], uint64_t pMessage, uint64_t pMessageOut)
{
    xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC8> in_mat(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC3,    600,  1000, XF_NPPC8> resized_mat(600,  1000);
    xf::cv::Mat<XF_8UC4,    600,  1000, XF_NPPC8> tmp_mat(600,  1000);    
    xf::cv::Mat<XF_8UC4,    600,  1000, XF_NPPC8> out_mat(600,  1000);       


    //float transform_matrix[9]= {    -2.405063291139244f, -3.797468354430384f,       1269.620253164558f,
    //                                0.0f,                -8.354430379746846f,       2422.784810126584f,
    //                                0.0f,                -0.007594936708860767f,    1.0f};


    float transform_matrix[9]= {    -1.539240506329116f,    -3.037974683544307f,    1269.620253164558f,
                                    0.0f,                   -6.683544303797476f,    2422.784810126584f,
                                    0.0f,                   -0.006075949367088614f, 1.0f};



    float inv_transform_matrix[9] =   {     -0.64967f,     0.37799f,   -90.95395f,
                                            0.00000f,     0.12442f,  -301.44737f,
                                            0.00000f,     0.00076f,    -0.83158f };

    #pragma HLS DATAFLOW

    
    int ramptr = 0;
    loop_read: for(int i = 0; i < WIDTH * HEIGHT / 8; i++)
    {
        ap_uint<192> pix;
        uint64_t tmp_ram = ram_in[ramptr++];
        
        pix.range( 0 + 23,  0 + 16) = (tmp_ram >>  0) & 0xff;
        pix.range( 0 + 15,  0 +  8) = (tmp_ram >>  8) & 0xff;
        pix.range( 0 +  7,  0 +  0) = (tmp_ram >> 16) & 0xff;

        pix.range( 8 + 23,  8 + 16) = (tmp_ram >> 24) & 0xff;
        pix.range( 8 + 15,  8 +  8) = (tmp_ram >> 32) & 0xff;
        pix.range( 8 +  7,  8 +  0) = (tmp_ram >> 40) & 0xff;

        pix.range(16 + 23, 16 + 16) = (tmp_ram >> 48) & 0xff;
        pix.range(16 + 15, 16 +  8) = (tmp_ram >> 56) & 0xff;
        tmp_ram = ram_in[ramptr++];
        pix.range(16 +  7, 16 + 0 ) = (tmp_ram >>  0) & 0xff;    

        pix.range(24 + 23, 24 + 16) = (tmp_ram >>  8) & 0xff;
        pix.range(24 + 15, 24 +  8) = (tmp_ram >> 16) & 0xff;
        pix.range(24 + 7,  24 +  0) = (tmp_ram >> 24) & 0xff;

        pix.range(32 + 23, 32 + 16) = (tmp_ram >> 32) & 0xff;
        pix.range(32 + 15, 32 +  8) = (tmp_ram >> 40) & 0xff;
        pix.range(32 + 7,  32 +  0) = (tmp_ram >> 48) & 0xff;

        pix.range(40 + 23, 40 + 16) = (tmp_ram >> 56) & 0xff;
        tmp_ram = ram_in[ramptr++];
        pix.range(40 + 15, 40 +  8) = (tmp_ram >>  0)  & 0xff;
        pix.range(40 + 7,  40 +  0) = (tmp_ram >>  8)  & 0xff;

        pix.range(48 + 23, 48 + 16) = (tmp_ram >> 16) & 0xff;
        pix.range(48 + 15, 48 +  8) = (tmp_ram >> 24) & 0xff;
        pix.range(48 +  7, 48 +  0)=  (tmp_ram >> 32) & 0xff;

        pix.range(56 + 23, 56 + 16) = (tmp_ram >> 40) & 0xff;
        pix.range(56 + 15, 56 +  8) = (tmp_ram >> 48) & 0xff;
        pix.range(56 +  7, 56 +  0) = (tmp_ram >> 56) & 0xff;
        in_mat.write(i, pix);

    }
    
    resize<XF_INTERPOLATION_TYPE, XF_8UC3, HEIGHT, WIDTH, OUTPUT_HEIGHT, OUTPUT_WIDTH, XF_NPPC8, MAXDOWNSCALE>(in_mat, resized_mat);

    for(int i = 0; i < OUTPUT_WIDTH * OUTPUT_HEIGHT / 8; i++)
    {
        ap_uint<256> output_pix;
        ap_uint<192> input_pix = resized_mat.read(i);
        output_pix.range( 23,   0)  = input_pix( 23,   0);
        output_pix.range( 55,  32)  = input_pix( 47,  24);
        output_pix.range( 87,  64)  = input_pix( 71,  48);
        output_pix.range(119,  96)  = input_pix( 95,  72);
        output_pix.range(151, 128)  = input_pix(119,  96);
        output_pix.range(182, 160)  = input_pix(143, 120);
        output_pix.range(215, 192)  = input_pix(167, 144);
        output_pix.range(247, 224)  = input_pix(191, 168);

        tmp_mat.write(i, output_pix);
    }

    xf::cv::warpTransform<NUM_STORE_ROWS, START_PROC, TRANSFORM_TYPE, INTERPOLATION, XF_8UC4, OUTPUT_HEIGHT, OUTPUT_WIDTH, NPC1, XF_USE_URAM>(tmp_mat, out_mat, inv_transform_matrix);
        
    loop_write: for(int i = 0; i < OUTPUT_WIDTH * OUTPUT_HEIGHT / 8; i+=1)
    {
        ap_uint<64> ram_out[3];
        ramptr = 0;
        ap_uint<256> pix;
        ap_uint<64> tmp_ram;
        int j = 0;
        
        pix = out_mat.read (i);
        tmp_ram.range( 7, 0) = pix.range(23,16);
        tmp_ram.range(15, 8) = pix.range(15,8);
        tmp_ram.range(23,16) = pix.range(7,0);
        j+=8;
        tmp_ram.range(31,24) = pix.range(j+23,j+16);
        tmp_ram.range(39,32) = pix.range(j+15,j+8);
        tmp_ram.range(47,40) = pix.range(j+7,j+0);
        j+=8;
        tmp_ram.range(55,48) = pix.range(j+23,j+16);
        tmp_ram.range(63,56) = pix.range(j+15,j+8);
        ram_out[ramptr++]    = tmp_ram;
        tmp_ram.range( 7, 0) = pix.range(j+7,j+0 );
        j+=8;
        tmp_ram.range(15, 8) = pix.range(j+23,j+16);
        tmp_ram.range(23,16) = pix.range(j+15,j+8);
        tmp_ram.range(31,24) = pix.range(j+7,j+0);
        j+=8;
        tmp_ram.range(39,32) = pix.range(j+23,j+16);
        tmp_ram.range(47,40) = pix.range(j+15,j+8);
        tmp_ram.range(55,48) = pix.range(j+7,j+0);
        j+=8;
        tmp_ram.range(63,56) = pix.range(j+23,j+16);
        ram_out[ramptr++]    = tmp_ram;
        tmp_ram.range( 7, 0) = pix.range(j+15,j+8);
        tmp_ram.range(15, 8) = pix.range(j+7,j+0);
        j+=8;

        tmp_ram.range(23,16) = pix.range(j+23,j+16);
        tmp_ram.range(31,24) = pix.range(j+15,j+8);
        tmp_ram.range(39,32) = pix.range(j+7,j+0);
        j+=8;
        tmp_ram.range(47,40) = pix.range(j+23,j+16);
        tmp_ram.range(55,48) = pix.range(j+15,j+8);
        tmp_ram.range(63,56) = pix.range(j+7,j+0);
        ram_out[ramptr++] = tmp_ram;
        j+=8;
        MEM_WRITE(ram_out, pMessageOut + i*3*8, 8*3);
    }

}

THREAD_ENTRY() {
    
    uint64_t output_payload_addr[1];

    ap_uint<64> ram_in[WIDTH*HEIGHT*3/8];
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
        MEM_READ(pMessage, ram_in, WIDTH*HEIGHT*3);           
        proc(memif_hwt2mem, memif_mem2hwt, ram_in, payload_addr[0], output_payload_addr[0]);

        //MEM_WRITE(ram_out, output_payload_addr[0], OUTPUT_WIDTH * OUTPUT_HEIGHT*3);
        ROS_PUBLISH(rcameraimageprojection_pubout, rcameraimageprojection_output_image);
    }

    return;
}
