/*
version:    4.0
author:     Shawn Pan 
Time:       2021.10.10 16:41
图像增加时间戳
增加节点rate可调
增加曝光增益调节
增加断线检测，断线之后发布topic:/error_signal(std_msgs::Int8 1)
*/

#include "daheng/GxIAPI.h"
#include "daheng/DxImageProc.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>  

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block
#define FILE_NAME_LEN           50              ///< Save image file name length

#define PIXFMT_CVT_FAIL             -1             ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS          0              ///< PixelFormatConvert success

//Show error message
#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        return emStatus;                   \
    }

//Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        GXCloseDevice(g_hDevice);          \
        g_hDevice = NULL;                  \
        GXCloseLib();                      \
        printf("<App Exit!>\n");           \
        return emStatus;                   \
    }

GX_DEV_HANDLE g_hDevice = NULL;                     ///< Device handle
bool g_bColorFilter = false;                        ///< Color filter support flag
int64_t g_i64ColorFilter = GX_COLOR_FILTER_NONE;    ///< Color filter of device


unsigned char* g_pRGBImageBuf = NULL;               ///< Memory for RAW8toRGB24
unsigned char* g_pRaw8Image = NULL;                 ///< Memory for RAW16toRAW8

int64_t g_nPayloadSize = 0;                         ///< Payload size

//Allocate the memory for pixel format transform 
void PreForAcquisition();

//Release the memory allocated
void UnPreForAcquisition();

//Convert frame date to suitable pixel format
int PixelFormatConvert(PGX_FRAME_BUFFER);


//Get description of error
void GetErrorString(GX_STATUS);


int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Starting loading ros launch parameters....");
    ros::init(argc, argv, "daheng_driver");
    ros::NodeHandle nh("~");  //私有命名空间，这样利用这个句柄的参数都会继承 ros::init 的节点名，但是这个节点名是可以被launch文件覆盖的。
//============================================================//
// 初始化 publisher & subscriber
    std::string cameraTopicName;
    int cameraQueueSize;
    int rate;
    nh.param("publishers/camera_pub/topic", cameraTopicName, std::string("/camera/image_raw"));
    nh.param("publishers/camera_pub/queue_size", cameraQueueSize, 1);
    nh.param("node_rate",rate,40);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(cameraTopicName,cameraQueueSize); //发布图像时消息队列长度只能为1
    ros::Publisher error_signal_pub;
    error_signal_pub = nh.advertise<std_msgs::Int8>("/error_signal", 1);
    
    ros::Rate loop_rate(rate);
// 初始化 publisher & subscriber结束
//============================================================//


//============================================================//
//初始化和检查相机版本是否适配
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    uint32_t ui32DeviceNum = 0;

    //Initialize libary 初始化设备库,进行一些资源申请操作。使用 GxIAPI 与相机进行交互前必须调用此接口进行初始化操作
    emStatus = GXInitLib(); 
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    //Get device enumerated number 枚举当前可用的所有设备,并且获取设备个数
    emStatus = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    if(emStatus != GX_STATUS_SUCCESS)
    { 
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }

    //If no device found, app exit
    if(ui32DeviceNum <= 0)
    {
        printf("<No device found>\n");
        GXCloseLib();
        return emStatus;
    }

    //Open first device enumerated
    emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;           
    }

    //Get Device Info
    printf("***********************************************\n");
    //Get libary version
    printf("<Libary Version : %s>\n", GXGetLibVersion());
    size_t nSize = 0;
    //Get string length of Vendor name
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Vendor name
    char *pszVendorName = new char[nSize];
    //Get Vendor name
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszVendorName;
        pszVendorName = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Vendor Name : %s>\n", pszVendorName);
    //Release memory for Vendor name
    delete[] pszVendorName;
    pszVendorName = NULL;

    //Get string length of Model name
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Model name
    char *pszModelName = new char[nSize];
    //Get Model name
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszModelName;
        pszModelName = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Model Name : %s>\n", pszModelName);
    //Release memory for Model name
    delete[] pszModelName;
    pszModelName = NULL;

    //Get string length of Serial number
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Serial number
    char *pszSerialNumber = new char[nSize];
    //Get Serial Number
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszSerialNumber;
        pszSerialNumber = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Serial Number : %s>\n", pszSerialNumber);
    //Release memory for Serial number
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    //Get string length of Device version
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_VERSION, &nSize);
    GX_VERIFY_EXIT(emStatus);
    char *pszDeviceVersion = new char[nSize];
    //Get Device Version
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Device Version : %s>\n", pszDeviceVersion);
    //Release memory for Device version
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
    printf("***********************************************\n");

    //Get the type of Bayer conversion. whether is a color camera. 检查是否是彩色相机
    emStatus = GXIsImplemented(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
    GX_VERIFY_EXIT(emStatus);

    //This app only support color cameras
    if (!g_bColorFilter)
    {
        printf("<This app only support color cameras! App Exit!>\n");
        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();
        return 0;
    }
    else
    {
        emStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
        GX_VERIFY_EXIT(emStatus);
    }
    
    emStatus = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &g_nPayloadSize);
    GX_VERIFY(emStatus);

    printf("\n");
    printf("Ready to start acquisition\n");
    printf("\n");

//初始化和检查相机版本结束
//===========================================================//

//============================================================//
//设置相机采集模式
    std::string   w_mode;
    double        w_red,w_green,w_blue;
    bool          reverse_x,reverse_y;
    std::string   exposure_mode, gain_mode;
    double        exposure_time_min, exposure_time_max, gain;

    nh.param("camera_param/w_mode", w_mode, std::string("continuous"));
    nh.param("camera_param/w_red", w_red,  1.4023);
    nh.param("camera_param/w_green", w_green,   1.0);
    nh.param("camera_param/reverse_x", reverse_x, true);
    nh.param("camera_param/reverse_x", reverse_y, true);
    nh.param("camera_param/exposure_mode", exposure_mode,std::string("continuous")); 
    nh.param("camera_param/exposure_time_min", exposure_time_min,20.00);
    nh.param("camera_param/exposure_time_max", exposure_time_max,20000.00);
    nh.param("camera_param/gain_mode",gain_mode,std::string("continuous"));
    nh.param("camera_param/gain_value",gain,15.00);
    

    //Set acquisition mode 连续采集模式
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_VERIFY_EXIT(emStatus);

    //Set trigger mode 非触发
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_VERIFY_EXIT(emStatus);

    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
    GX_VERIFY_EXIT(emStatus);

    //查询当前相机是否支持某功能 bStreamTransferSize = false 不支持/Ture 支持
    bool bStreamTransferSize = false;
    emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_VERIFY_EXIT(emStatus);
    if(bStreamTransferSize)
    {
        //Set size of data transfer block
        emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        GX_VERIFY_EXIT(emStatus);
    }

    bool bStreamTransferNumberUrb = false;
    emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_VERIFY_EXIT(emStatus);
    if(bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block
        emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        GX_VERIFY_EXIT(emStatus);
    }


    //白平衡模式 Set Balance White Mode : Continuous
    if(w_mode == "continuous")
    {
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        GX_VERIFY_EXIT(emStatus);
    }
    else
    {
        //set white balance red channel
        emStatus = GXSetEnum(g_hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_RED); 
        emStatus += GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, w_red); 
        //set white balance green channel
        emStatus += GXSetEnum(g_hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_GREEN); 
        emStatus += GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, w_green); 
        //set white balance blue channel
        emStatus += GXSetEnum(g_hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_BLUE); 
        emStatus += GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, w_blue); 

        if (emStatus != GX_STATUS_SUCCESS )
        {
             GXCloseDevice(g_hDevice);          
             g_hDevice = NULL;                  
             GXCloseLib();                      
             printf("<Error in setting Balance White Mode!>\n");
        }
    }

    //水平翻转
    emStatus = GXSetBool(g_hDevice, GX_BOOL_REVERSE_X, reverse_x);
    GX_VERIFY_EXIT(emStatus);
    //垂直翻转
    emStatus = GXSetBool(g_hDevice, GX_BOOL_REVERSE_Y, reverse_y);
    GX_VERIFY_EXIT(emStatus);

    //自动曝光模式
    if (exposure_mode == "continuous"){
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    }else if(exposure_mode == "once"){
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_ONCE);
    }else{
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    }
    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, exposure_time_max);
    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, exposure_time_min);

    if (emStatus != GX_STATUS_SUCCESS )
    {
        GXCloseDevice(g_hDevice);          
        g_hDevice = NULL;                  
        GXCloseLib();                      
        printf("<Error in setting EXPOSURE_AUTO!>\n");
    }

    //设置曝光增益
    if (gain_mode =="continuous"){
        GXSetEnum(g_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    }else{
        GXSetEnum(g_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
        emStatus = GXSetFloat(g_hDevice, GX_FLOAT_GAIN, gain);
        GX_VERIFY_EXIT(emStatus);
    }

//设置相机采集模式结束
//============================================================//


    //Allocate the memory for pixel format transform 
    PreForAcquisition();

    //Device start acquisition
    //开采,包括流开采和设备开采
    emStatus = GXStreamOn(g_hDevice); 
        if(emStatus != GX_STATUS_SUCCESS)
        {
            //Release the memory allocated
            UnPreForAcquisition();
            GX_VERIFY_EXIT(emStatus);
        }

    //原ProcGetImage函数：
    emStatus = GX_STATUS_SUCCESS;
    VxInt32 emDXStatus = DX_OK;

    //Thread running flag setup
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;
    uint32_t ui32AcqFrameRate = 0;
    double exposure_time = 0;
    double w_red_value = 0;
    double w_green_value = 0;
    double w_blue_value = 0;
    double gain_value = 0;
    std_msgs::Int8 error_msgs;

    while(nh.ok())
    {
        // emStatus = GXUpdateDeviceList(&ui32DeviceNum, 1000);
        // if(emStatus != GX_STATUS_SUCCESS || ui32DeviceNum ==0)
        // { 
        //     ROS_INFO("no device found!\n");
        //     break;
        // }

        if(!ui32FrameCount)
        {
            time(&lInit);
        }

        // Get a frame from Queue 在开始采集之后,通过此接口可以获取一副图像(零拷贝)
        emStatus = GXDQBuf(g_hDevice, &pFrameBuffer, 1000); 
            if(emStatus != GX_STATUS_SUCCESS)
            {
                if (emStatus == GX_STATUS_TIMEOUT)
                {
                    ROS_INFO("time out! check camera! \n"); 
                }
                else
                {
                    GetErrorString(emStatus);
                }
            break;
            }

        if(pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            ROS_INFO("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        }
        else
        {
            ui32FrameCount++;   //应该是在算每秒帧数,linit是初始时间，lend在算结束时间，两个一减 >= 1,计算出1秒的时间
            time (&lEnd);
            // Print acquisition info each second.
            if (lEnd - lInit >= 1)
            {
                std::cout << lEnd << std::endl;
                GXGetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, &exposure_time);
                GXSetEnum(g_hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_RED); 
                GXGetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, &w_red_value); 
                GXSetEnum(g_hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_GREEN); 
                GXGetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, &w_green_value); 
                GXSetEnum(g_hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_BLUE); 
                GXGetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, &w_blue_value); 
                GXGetFloat(g_hDevice, GX_FLOAT_GAIN, &gain_value);
                
                printf("<--- RZS-DAHENG-DRIVER --->\n");
                printf("<---     Shawn Pan     --->\n");
                printf("Width: %d \nHeight: %d \nFrameRate: %u \nFrameID: %llu \nexposure_time: %f ms \nw_red_value: %f \nw_green_value: %f \nw_blue_value: %f \n",
                    pFrameBuffer->nWidth, pFrameBuffer->nHeight, ui32FrameCount, pFrameBuffer->nFrameID, exposure_time/1000, w_red_value, w_green_value, w_blue_value);
                printf("exposure_gain: %f dB\n",gain_value);
                // printf("\033c");
                ui32FrameCount = 0;
            }
            //======试着创建图像=======//
            emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                ROS_INFO("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                break;
            }

            //传递图像
            cv::Mat frame(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3, g_pRGBImageBuf);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
            msg->header.stamp = ros::Time::now();
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            //======创建图像结束=======//
        }
        emStatus = GXQBuf(g_hDevice, pFrameBuffer);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            break;
        }  
    }

    error_msgs.data = 1;
    error_signal_pub.publish(error_msgs);
    ros::spinOnce();
    loop_rate.sleep();


    //Device stop acquisition
    emStatus = GXStreamOff(g_hDevice); //停采,包括流停采和设备停采
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }

    //Release the resources and stop acquisition thread
    UnPreForAcquisition();

    //Close device
    emStatus = GXCloseDevice(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        g_hDevice = NULL;
        GXCloseLib();
        return emStatus;
    }

    //Release libary
    emStatus = GXCloseLib();
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    printf("<App exit!>\n");
    return 0;
}



//-------------------------------------------------
/**
\brief Convert frame date to suitable pixel format
\param pParam[in]           pFrameBuffer       FrameData from camera
\return void
*/
//-------------------------------------------------
int PixelFormatConvert(PGX_FRAME_BUFFER pFrameBuffer)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW8 or RAW16 image to RGB24 image
    switch (pFrameBuffer->nPixelFormat)
    {
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
        {
            // Convert to the RGB image
            emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
        {
            // Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, g_pRaw8Image, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            // Convert to the RGB24 image
            emDXStatus = DxRaw8toRGB24((unsigned char*)g_pRaw8Image, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        default:
        {
            printf("Error : PixelFormat of this camera is not supported\n");
            return PIXFMT_CVT_FAIL;
        }
    }
    return PIXFMT_CVT_SUCCESS;
}

//-------------------------------------------------
/**
\brief Allocate the memory for pixel format transform 
\return void
*/
//-------------------------------------------------
void PreForAcquisition()
{
    g_pRGBImageBuf = new unsigned char[g_nPayloadSize * 3]; 
    g_pRaw8Image = new unsigned char[g_nPayloadSize];

    return;
}

//-------------------------------------------------
/**
\brief Release the memory allocated
\return void
*/
//-------------------------------------------------
void UnPreForAcquisition()
{
    //Release resources
    if (g_pRaw8Image != NULL)
    {
        delete[] g_pRaw8Image;
        g_pRaw8Image = NULL;
    }
    if (g_pRGBImageBuf != NULL)
    {
        delete[] g_pRGBImageBuf;
        g_pRGBImageBuf = NULL;
    }

    return;
}

//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS emErrorStatus)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    
    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }
    
    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }
    
    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}

