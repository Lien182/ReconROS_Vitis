#include "reconos_thread.h"
#include "reconos_calls.h"

#include "hls_stream.h"
#include <ap_int.h>

#include <common/xf_common.hpp>
#include <common/xf_utility.hpp>

#include <imgproc/xf_gaussian_filter.hpp>

#include <sensor_msgs/msg/image.h>

using namespace std;

#define HEIGHT 480
#define WIDTH 640
#define INTYPE uint64_t



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
    //#pragma HLS stream variable=in_mat.data depth=640
    //xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1> _in_mat(HEIGHT,WIDTH);
    //#pragma HLS stream variable=bgr2rgb.data depth=640
    xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1> out_mat(HEIGHT,WIDTH);
    //#pragma HLS stream variable=hsv_mat.data depth=640

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
    
    xf::cv::GaussianBlur<FILTER_WIDTH, XF_BORDER_CONSTANT, XF_8UC3, HEIGHT, WIDTH, XF_NPPC1>(in_mat, out_mat, 1.16666f);
    

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


    //not tested yet, but puts ram_out into the ultraRAM section
    //#pragma HLS RESOURCE variable=ram_out core=XPM_MEMORY uram

    THREAD_INIT();
	uint64_t initdata = GET_INIT_DATA();

    uint64_t pMessage_OutputTopic = MEMORY_GETOBJECTADDR(rgaussianblur_output_image);
    MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + pMessage_OutputTopic, output_payload_addr, 8);
                
    while(1)
    {
        uint64_t payload_addr[1];
        uint64_t pMessage =  ROS_SUBSCRIBE_TAKE(rgaussianblur_subdata, rgaussianblur_camera_image);

        MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + pMessage, payload_addr, 8);            
        proc(memif_hwt2mem, memif_mem2hwt, payload_addr[0], ram_out);

        MEM_WRITE(ram_out, output_payload_addr[0], WIDTH*HEIGHT*3);
        ROS_PUBLISH(rgaussianblur_pubout, rgaussianblur_output_image);
    }

    return;
}
