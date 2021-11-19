/*
------------------------------------------------------------------------
-  USMA Modification of FLIR Systems - Linux Boson Capture & Recording -
------------------------------------------------------------------------
-  This code is based on the code found at                             -
-  https://github.com/FLIR/BosonUSB.git                                -
-                                                                      -
------------------------------------------------------------------------
*/
#include <stdio.h>
#include <fcntl.h> // open, O_RDWR
#include <opencv2/opencv.hpp>
#include <unistd.h>    // close
#include <sys/ioctl.h> // ioctl
#include <asm/types.h> // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include <string>
extern "C"
{ // For using the Boson C SDK
#include "boson/EnumTypes.h"
#include "boson/UART_Connector.h"
#include "boson/Client_API.h"
}

#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "std_msgs/String.h" // Callback for directory needs this
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include <chrono>

#define YUV 0
#define RAW16 1

using namespace cv;

#define v_major 1
#define v_minor 0

// Define COLOR CODES
#define RED "\x1B[31m"
#define GRN "\x1B[32m"
#define YEL "\x1B[33m"
#define BLU "\x1B[34m"
#define MAG "\x1B[35m"
#define CYN "\x1B[36m"
#define WHT "\x1B[37m"
#define RESET "\x1B[0m"

// Need to clean video for linux structs to avoid some random initializations problems (not always present)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

using namespace std;
using namespace std::chrono;

class BosonUSMA
{
private:
    int width;
    int height;
    int ret;
    int fd;
    int i;
    long frame;                   // First frame number enumeration
    std::string  video_id;               // To store Video Port Device
    //TODO convert below to std""string
    char label[50];               // To display the information
    char thermal_sensor_name[20]; // To store the sensor name
    char filename[60];            // PATH/File_count
    char folder_name[30];         // To store the folder name
    char video_frames_str[30];
    // Default Program options
    int video_mode;
    int device_id; 
    int record_enable;
    struct v4l2_format format;
    struct v4l2_buffer bufferinfo;
    std::string data_dir;
    std::string trigtime;


    std::string serial_num;
    std::ofstream csvOutfile;
    std::string image_folder8;
    std::string image_folder16;
    std::string image_filename8;
    std::string image_filename16;
    std::string img_time;
    std::string crnt_time;
    ros::Time ros_timenow;
    int saved_count;

    // State updated by callbacks
    sensor_msgs::MagneticField mag_data;
    sensor_msgs::Imu imu_data;
    mavros_msgs::Altitude rel_alt;
    sensor_msgs::NavSatFix gps_fix;
    geometry_msgs::TwistStamped vel_gps;
    sensor_msgs::Temperature temp_imu;

    FLR_BOSON_EXT_SYNC_MODE_E sync_mode;

    // Declarations for RAW16 representation
    // Will be used in case we are reading RAW16 format
    // Boson320 , Boson 640
    Mat thermal16;        // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
    Mat thermal16_linear; // OpenCV output buffer : Data used to display the video

    // To record images
    std::vector<int> compression_params;

    image_transport::Publisher image_pub_8;
    image_transport::Publisher image_pub_16;
    //TODO Add the subscribers to get logging data

    ros::Subscriber record_sub;
    ros::Subscriber dir_sub;
    ros::Subscriber alt_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber mag_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber temp_sub;
    ros::Subscriber trigtime_sub;


public:
    BosonUSMA(ros::NodeHandle *nh)
    {
        frame = 0; // First frame number enumeration
        // Default Program options
        width = 640;        //TODO query the camera to get this
        height = 512;       //TODO query the camera to get this
        video_mode = RAW16; //TODO query the camera to get this
        record_enable = 0;
        saved_count = 0;
        compression_params.push_back(IMWRITE_PXM_BINARY);
        data_dir = "/tmp/BHG_DATA/";
        image_filename8 = "default_imgname.ppm";
        image_filename16 = "default_imgname.ppm";
        trigtime = "TrigTimeNotSet";
        csvOutfile;
        image_folder8 = data_dir + "boson_imgs8/";
        image_folder16 = data_dir + "boson_imgs16/";
        serial_num = "9999999"; // TODO get from camera using sysinfoGetCameraSN sysinfoGetProductName
        sync_mode = FLR_BOSON_EXT_SYNC_DISABLE_MODE;
        // Video device by default
        sprintf(folder_name, "TestFolder");
        sprintf(thermal_sensor_name, "Boson_640");

        // TODO reconsile the file saving between Boson method and boson method
        if (strlen(folder_name) <= 1)
        { // File name has to be more than two chars
            strcpy(folder_name, thermal_sensor_name);
        }
        mkdir(folder_name, 0700);
        chdir(folder_name);
        ROS_INFO(WHT ">>> Folder " YEL "%s" WHT " selected to record files", folder_name);

        image_transport::ImageTransport it_(*nh);
        image_pub_8 = it_.advertise("boson_image8", 1);
        image_pub_16 = it_.advertise("boson_image16", 1);
        
        // get ros param for device ID
        if (nh->hasParam("/video_id")){  
            nh->getParam("/video_id", video_id);
        }
        else{ // default to setting for the laptop. use video0 for NUC
            video_id = "/dev/video0"; 
        }

        dir_sub = nh->subscribe("/directory", 1000, &BosonUSMA::dirCallback, this);
        record_sub = nh->subscribe("/record", 10, &BosonUSMA::recordCallback, this);
        dir_sub = nh->subscribe("/directory", 1000, &BosonUSMA::dirCallback, this);
        alt_sub = nh->subscribe("/mavros/altitude", 1000, &BosonUSMA::alt_cb, this);
        gps_sub = nh->subscribe("/mavros/global_position/raw/fix", 1000, &BosonUSMA::gps_cb, this);
        vel_sub = nh->subscribe("/mavros/global_position/raw/gps_vel", 1000, &BosonUSMA::vel_cb, this);
        mag_sub = nh->subscribe("/mavros/imu/mag", 1000, &BosonUSMA::mag_cb, this);
        imu_sub = nh->subscribe("/mavros/imu/data", 1000, &BosonUSMA::imu_cb, this);
        temp_sub = nh->subscribe("/mavros/imu/temperature_imu", 1000, &BosonUSMA::temp_cb, this);
        trigtime_sub = nh->subscribe("/trig_timer", 1000, &BosonUSMA::trigtime_cb, this);
        //print_caminfo(); TODO this does not work, it appears to start a second connection
    }

    ~BosonUSMA()
    {
        perror(RED "VIDIOC_QBUF" WHT);
        exit(1);
    }

    void recordCallback(const std_msgs::Bool msg)
    {
        this->record_enable = msg.data;
        ROS_INFO("***** BOSON:  RECORD CALLBACK FIRED WITH A VALUE OF: [%d]", this->record_enable);
    }

    void set_record(bool is_recording){
        this->record_enable = is_recording;
    }
    
    int get_savedcount(){
        return this->saved_count;
    }

    int closeSensor()
    {
        // Deactivate streaming
        int type = bufferinfo.type;
        if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0)
        {
            perror(RED "VIDIOC_STREAMOFF" WHT);
            exit(1);
        };

        close(fd);
        ROS_INFO(WHT ">>> " YEL "%s exited cleanly", thermal_sensor_name);

        return EXIT_SUCCESS;
    }

    int openSensor()
    {
        struct v4l2_capability cap;

        // ROS_INFO Sensor defined
        ROS_INFO(WHT ">>> " YEL "%s" WHT " selected", thermal_sensor_name);

        // We open the Video Device
        ROS_INFO(WHT ">>> " YEL "%s" WHT " selected", video_id.c_str());
        if ((fd = open(video_id.c_str(), O_RDWR)) < 0)
        {
            perror(RED "Error : OPEN. Invalid Video Device" WHT);
            exit(1);
        }

        // Check VideoCapture mode is available
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
        {
            perror(RED "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" WHT);
            exit(1);
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
            fprintf(stderr, RED "The device does not handle single-planar video capture." WHT);
            exit(1);
        }

        CLEAR(format);
        //ROS_INFO(WHT ">>> " YEL "16 bits " WHT "capture selected");
          
        // I am requiring thermal 16 bits mode
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

        // Common varibles
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;

        // request desired FORMAT
        if (ioctl(fd, VIDIOC_S_FMT, &format) < 0)
        {
            perror(RED "VIDIOC_S_FMT" WHT);
            exit(1);
        }

        // we need to inform the device about buffers to use.
        // and we need to allocate them.
        // we’ll use a single buffer, and map our memory using mmap.
        // All this information is sent using the VIDIOC_REQBUFS call and a
        // v4l2_requestbuffers structure:
        struct v4l2_requestbuffers bufrequest;
        bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufrequest.memory = V4L2_MEMORY_MMAP;
        bufrequest.count = 1; // we are asking for one buffer

        if (ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0)
        {
            perror(RED "VIDIOC_REQBUFS" WHT);
            exit(1);
        }

        // Now that the device knows how to provide its data,
        // we need to ask it about the amount of memory it needs,
        // and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
        // and its v4l2_buffer structure.

        memset(&bufferinfo, 0, sizeof(bufferinfo));

        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = 0;

        if (ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0)
        {
            perror(RED "VIDIOC_QUERYBUF" WHT);
            exit(1);
        }

        // map fd+offset into a process location (kernel will decide due to our NULL). lenght and
        // properties are also passed
        ROS_INFO(WHT ">>> Image width  =" YEL "%i" WHT, width);
        ROS_INFO(WHT ">>> Image height =" YEL "%i" WHT, height);
        ROS_INFO(WHT ">>> Buffer lenght=" YEL "%i" WHT, bufferinfo.length);

        void *buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, bufferinfo.m.offset);

        if (buffer_start == MAP_FAILED)
        {
            perror(RED "mmap" WHT);
            exit(1);
        }

        // Fill this buffer with ceros. Initialization. Optional but nice to do
        memset(buffer_start, 0, bufferinfo.length);

        // Activate streaming
        int type = bufferinfo.type;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
        {
            perror(RED "VIDIOC_STREAMON" WHT);
            exit(1);
        }

        // Declarations for RAW16 representation
        // Will be used in case we are reading RAW16 format
        // Boson320 , Boson 640
        thermal16 = cv::Mat(height, width, CV_16U, buffer_start); // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
        thermal16_linear = cv::Mat(height, width, CV_8U, 1);      // OpenCV output buffer : Data used to display the video
    }

    int getFrame(int cnt)
    {
        //std::string start_time = make_datetime_stamp();
        auto crnt_time_code = high_resolution_clock::now();
        // ROS_INFO(GRN ">>> HERE NOW1 Type: %d, Length: %d", bufferinfo.type, bufferinfo.length);
        // Put the buffer in the incoming queue.
        if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0)
        {
            perror(RED "VIDIOC_QBUF" WHT);
            exit(1);
        }

        // The buffer's waiting in the outgoing queue.
        if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0)
        {
            perror(RED "VIDIOC_QBUF" WHT);
            exit(1);
        } // DML: 17 mSec for the two buffer calls, this 1 mSec faster when synch is disabled.
        auto delta_time1 = duration<double>(high_resolution_clock::now() - crnt_time_code).count();
        //ROS_INFO(CYN ",***** AftrBuffer ,%f" WHT, delta_time);

        AGC_Basic_Linear(thermal16, thermal16_linear, height, width); // DML: 9 mSec for the AGC

        auto delta_time2 = duration<double>(high_resolution_clock::now() - crnt_time_code).count();
        //ROS_INFO(CYN ",***** AftrAGC ,%f" WHT, delta_time);

        // Display thermal after 16-bits AGC... will display an image
        sprintf(label, "%s : RAW16  Linear", thermal_sensor_name);
        //imshow(label, thermal16_linear);
            this->crnt_time = this->trigtime;

        //if ((record_enable == 1) && (cnt % 3 == 0))
        if (record_enable == 1) 
        {
            //this->crnt_time = make_datetime_stamp();
            this->image_filename8 = image_folder8 + "BOSON" + this->serial_num + "_8_" + this->crnt_time + ".ppm";
            this->image_filename16 = image_folder16 + "BOSON" + this->serial_num + "_16_" + this->crnt_time + ".ppm";
            //errorCode = XC_SaveData(this->handle, "output.png", XSD_SaveThermalInfo | XSD_Force16);
            cv::imwrite(this->image_filename8, thermal16_linear, compression_params);
            cv::imwrite(this->image_filename16, thermal16, compression_params);
            this->csvOutfile << make_logentry() << std::endl;
            this->saved_count++;
            //ROS_INFO(CYN ",***** rostime ,%s,%s" WHT, start_time.c_str(),this->crnt_time.c_str());

            //ROS_INFO_THROTTLE(1, "File Location is: %s", this->image_filename16.c_str());
        } // DML: 12mSec from after AGC to here when saving images it drops to 9ms if we remove saving 8 bit image

        auto delta_time3 = duration<double>(high_resolution_clock::now() - crnt_time_code).count();

        cv::putText(thermal16_linear, this->crnt_time, cvPoint(30, 40),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(1, 1, 1), 2);
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->encoding = "mono8";
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = "boson";
        cv_ptr->image = thermal16_linear;
        this->image_pub_8.publish(cv_ptr->toImageMsg());
        cv::putText(thermal16, this->crnt_time, cvPoint(30, 40),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(1, 1, 1), 2);
        auto delta_time4 = duration<double>(high_resolution_clock::now() - crnt_time_code).count();

        cv_ptr->encoding = "mono16";
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = "boson";
        cv_ptr->image = thermal16;
        this->image_pub_16.publish(cv_ptr->toImageMsg());
        // DML: 0.45 mSec to create and publish both images, 2nd image adds 0.15 mSec
        auto delta_time5 = duration<double>(high_resolution_clock::now() - crnt_time_code).count();
        // DML: 36.8 mSec for this function to run this is about 27hz.
        //ROS_INFO(CYN ",***** AftrSave ,%f,%f,%f,%f,%f" WHT, delta_time1,delta_time2,delta_time3,delta_time4,delta_time5);
        return 0;
    }

    // AGC Sample ONE: Linear from min to max.
    // Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
    // Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
    void AGC_Basic_Linear(Mat input_16, Mat output_8, int height, int width)
    {
        int i, j; // aux variables

        // auxiliary variables for AGC calcultion
        unsigned int max1 = 0;      // 16 bits
        unsigned int min1 = 0xFFFF; // 16 bits
        unsigned int value1, value2, value3, value4;

        // RUN a super basic AGC
        for (i = 0; i < height; i++)
        {
            for (j = 0; j < width; j++)
            {
                value1 = input_16.at<uchar>(i, j * 2 + 1) & 0XFF; // High Byte
                value2 = input_16.at<uchar>(i, j * 2) & 0xFF;     // Low Byte
                value3 = (value1 << 8) + value2;
                if (value3 <= min1)
                {
                    min1 = value3;
                }
                if (value3 >= max1)
                {
                    max1 = value3;
                }
                //ROS_INFO("%X.%X.%X  ", value1, value2, value3);
            }
        }
        //ROS_INFO("max1=%04X, min1=%04X", max1, min1);

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                value1 = input_16.at<uchar>(i, j * 2 + 1) & 0XFF; // High Byte
                value2 = input_16.at<uchar>(i, j * 2) & 0xFF;     // Low Byte
                value3 = (value1 << 8) + value2;
                value4 = ((255 * (value3 - min1))) / (max1 - min1);
                // ROS_INFO("%04X ", value4);

                output_8.at<uchar>(i, j) = (uchar)(value4 & 0xFF);
            }
        }
    }

    /* ---------------------------- Other Aux functions ---------------------------------------*/

    // Must be run before connecting to the camera with V4l.
    int print_caminfo()
    {
        // Connect to the camera. 16 is ttyACM0.
        int32_t dev = 16;
        int32_t baud = 921600;

        FLR_RESULT result;
        result = Initialize(dev, baud); //COM6, 921600 baud (port_number=5 for COM6)
        if (result)
        {
            ROS_ERROR("Failed to initialize, exiting. 0x%08X\n", result);
            Close();
            return -1;
        }
        else
        {

            ROS_INFO(CYN "BOSON initialized successfully result is: 0x%08X" WHT, result);
        }

        // Retrieve the Camera SN and print it
        uint32_t camera_sn;
        result = bosonGetCameraSN(&camera_sn);
        if (result)
        {
            ROS_ERROR("Failed with status 0x%08X, exiting.\n", result);
            Close();
            return -1;
        }
        else
        {
            serial_num = to_string(camera_sn);
            ROS_INFO(CYN "CameraSN: %d" WHT, camera_sn);
        }

        uint32_t major, minor, patch;
        result = bosonGetSoftwareRev(&major, &minor, &patch);
        if (result)
        {
            ROS_ERROR("Failed with status 0x%08X, exiting.\n", result);
            Close();
            return -1;
        }
        else
        {
            ROS_INFO(CYN "SoftwareRev:  %u.%u.%u " WHT, major, minor, patch);
        }

        FLR_BOSON_SENSOR_PARTNUMBER_T part_num;
        result = bosonGetSensorPN(&part_num);
        if (result)
        {
            ROS_ERROR("Failed with status 0x%08X, exiting.\n", result);
            Close();
            return -1;
        }
        else
        {
            ROS_INFO(CYN "PartNum:  \"%s\"" WHT, part_num.value);
        }

        //Set Trigger mode
//        FLR_BOSON_EXT_SYNC_MODE_E sync_mode = FLR_BOSON_EXT_SYNC_SLAVE_MODE;
        // enum e_FLR_BOSON_EXT_SYNC_MODE_E {
        // FLR_BOSON_EXT_SYNC_DISABLE_MODE = (int32_t) 0,
        // FLR_BOSON_EXT_SYNC_MASTER_MODE = (int32_t) 1,
        // FLR_BOSON_EXT_SYNC_SLAVE_MODE = (int32_t) 2,
        // FLR_BOSON_EXT_SYNC_END = (int32_t) 3,
//        result = bosonSetExtSyncMode(sync_mode);
//        if (result)
//        {
//            ROS_ERROR("Failed with status 0x%08X, exiting.", result);
//            Close();
//            return -1;
//        }
//        else
//        {
//            ROS_INFO(CYN "Boson Synch mode set to master" WHT);
//        }

        // Retrieve the Synch Mode
        result = bosonGetExtSyncMode(&sync_mode);
        if (result)
        {
            ROS_ERROR("Failed with status 0x%08X, exiting.\n", result);
            Close();
            return -1;
        }
        else
        {
            std::string sync_mode_str="Disabled";
            if(sync_mode ==1 ){sync_mode_str = "Master";}
            else if(sync_mode ==2 ){sync_mode_str = "Slave";}
            ROS_INFO(CYN "Camera Synch Mode: %s" WHT, sync_mode_str.c_str());
        }



//        string trig_mode = "disable";
//        if (nh->hasParam("boson/trig_mode"))
//        {
//            nh->getParam("boson/trig_mode", trig_mode);
//            if (trig_mode == "master")
//            {
//                sync_mode = FLR_BOSON_EXT_SYNC_MASTER_MODE;
//        ROS_INFO("***** BOSON:  RECORD CAMERA IN SYNC MASTER MODE");
//            }
//            else if (trig_mode == "slave")
//            {
//                sync_mode = FLR_BOSON_EXT_SYNC_SLAVE_MODE;
//        ROS_INFO("***** BOSON:  RECORD CAMERA IN SYNC SLAVE MODE");trigtime_cb
//            }
//            else
//            {
//                sync_mode = FLR_BOSON_EXT_SYNC_DISABLE_MODE;
//        ROS_INFO("***** BOSON:  RECORD CAMERA IN SYNC DISABLE MODE");
//            }
//        }
//        else
//        { // Default to no trigger mode
//            sync_mode = FLR_BOSON_EXT_SYNC_DISABLE_MODE;
//        ROS_INFO("***** BOSON:  RECORD CAMERA IN SYNC DISABLE MODE no param specified");
//            
//        }



        Close(); //TODO rename this function in the SDK
        return 0;
    }

    // HELP INFORMATION
    // TODO Update to reflect current usage
    void print_help()
    {
        ROS_INFO(CYN "West Point Robotics Boson Capture and Record Node for Project Convergence v%i.%i" WHT, v_major, v_minor);
        ROS_INFO(CYN "Press " YEL "Ctrl-C " CYN "to shutdown the node cleanly" WHT);
    }

    // Make a string with the date time stamp formatted for file names. Includes milliseconds.
    std::string make_datetime_stamp()
    {
        char buffer[80];
        this->ros_timenow = ros::Time::now();
        std::time_t raw_time = static_cast<time_t>(this->ros_timenow.toSec());
        struct tm *local_tm = localtime(&raw_time);
        int ros_millisec = int((this->ros_timenow.nsec) / 1000);
        strftime(buffer, 80, "%Y%m%d_%H%M%S", local_tm);
        sprintf(buffer, "%s_%06d", buffer, ros_millisec);
        std::string datetime_stamp(buffer);
        return datetime_stamp;
    }

    // Create a directory to files into. Use default or a published common directory.
    bool create_directories()
    {
        this->data_dir.pop_back();
        std::string dir_time(data_dir.substr(data_dir.rfind("/")));

        this->image_folder8 = data_dir + "/BOSON8_SN_" + this->serial_num + "/";
        this->image_folder16 = data_dir + "/BOSON16_SN_" + this->serial_num + "/";
        string csv_filename = data_dir + dir_time + "_boson.csv";
        //ROS_INFO("***** BOSON:  Creating directory %s", this->image_folder.c_str());
        if (mkdir(this->image_folder8.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
        {
            if (errno != 17)
            {
                ROS_INFO("***** BOSON 8:  0 Directory does not exist and MKDIR failed the errno is %i", errno);
            }
        }
        else
        {
            //ROS_INFO("***** BOSON:  Created Data Directory and establishing csv file" );
            ROS_INFO("***** BOSON 8 Bit:  Data directory is: [%s]", this->image_folder8.c_str());

            if (csvOutfile.is_open())
            { // Close the old csv file
                ROS_INFO("***** BOSON:  BOSON CSV File is already open: [%s]", csv_filename.c_str());
                csvOutfile.close();
            }
            csvOutfile.open(csv_filename, std::ios_base::app); // open new csv file
            if (!csvOutfile)
            {
                ROS_ERROR("***** BOSON:  ERROR   FAILED TO OPEN BOSON CSV File: %s,  [%s]", strerror(errno), csv_filename.c_str());
                csvOutfile.open(csv_filename, std::ios_base::app); // DML is unrealable in creating the initial file
                ROS_ERROR("***** BOSON:  ERROR   BLINDLY TRYIED TO OPEN csv AGAIN: %s,  [%s]", strerror(errno), csv_filename.c_str());
            }
            ROS_INFO("***** BOSON:  CSV file is: [%s]", csv_filename.c_str());
            csvOutfile << make_header() << endl;
        }

        if (mkdir(this->image_folder16.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
        {
            if (errno != 17)
            {
                ROS_INFO("***** BOSON 16:  0 Directory does not exist and MKDIR failed the errno is %i", errno);
            }
        }
        else
        {
            //ROS_INFO("***** BOSON:  Created Data Directory and establishing csv file" );
            ROS_INFO("***** BOSON 16 Bit:  Data directory is: [%s]", this->image_folder8.c_str());
}

    }

    // Make the header for the csv file
    // TODO clean this up, a lot of unused columns that printing zeros. Formating of code needs cleanup also.
    string make_header()
    {
        string header = "";
        header = "filename,rostime,"; //rel_alt.monotonic,rel_alt.amsl,rel_alt.local,rel_alt.relative,";
        header += "gps_fix.status.status,gps_fix.status.service,gps_fix.latitude,gps_fix.longitude,gps_fix.altitude,";
        header += "mag_data.magnetic_field.x,mag_data.magnetic_field.y,mag_data.magnetic_field.z,";
        header += "imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w,";
        header += "imu_data.angular_velocity.x,imu_data.angular_velocity.y,imu_data.angular_velocity.z,";
        header += "imu_data.linear_acceleration.x,imu_data.linear_acceleration.y,imu_data.linear_acceleration.z,";
        header += "vel_gps.twist.linear.x,vel_gps.twist.linear.y,vel_gps.twist.linear.z,";
        //header += "vel_gps.twist.angular.x,vel_gps.twist.angular.y,vel_gps.twist.angular.z,";
        //header += "temp_imu.temperature";
        return header;
    }

    string make_logentry()
    {
        char time_str[50];
        sprintf(time_str,"%f", this->ros_timenow.toSec());// +
        string alt_str = image_filename16 + "," + time_str + ",";// + to_string(rel_alt.monotonic) + "," + to_string(rel_alt.amsl) + "," + to_string(rel_alt.local) + "," + to_string(rel_alt.relative);
        string gps_str = to_string(gps_fix.status.status) + "," + to_string(gps_fix.status.service) + "," + to_string(gps_fix.latitude) + "," + to_string(gps_fix.longitude) + "," + to_string(gps_fix.altitude);
        string mag_str = to_string(mag_data.magnetic_field.x) + "," + to_string(mag_data.magnetic_field.y) + "," + to_string(mag_data.magnetic_field.z);
        string imu_str = to_string(imu_data.orientation.x) + "," + to_string(imu_data.orientation.y) + "," + to_string(imu_data.orientation.z) + "," + to_string(imu_data.orientation.w) + ",";
        imu_str += to_string(imu_data.angular_velocity.x) + "," + to_string(imu_data.angular_velocity.y) + "," + to_string(imu_data.angular_velocity.z) + ",";
        imu_str += to_string(imu_data.linear_acceleration.x) + "," + to_string(imu_data.linear_acceleration.y) + "," + to_string(imu_data.linear_acceleration.z);
        string vel_str = to_string(vel_gps.twist.linear.x) + "," + to_string(vel_gps.twist.linear.y) + "," + to_string(vel_gps.twist.linear.z);
        //vel_str += to_string(vel_gps.twist.angular.x) + "," + to_string(vel_gps.twist.angular.y) + "," + to_string(vel_gps.twist.angular.z);
        //string temp_str = to_string(temp_imu.temperature);
        string output = alt_str + "," + gps_str + "," + mag_str + "," + imu_str;// + "," + vel_str + "," + temp_str;
        return output;
    }

    string char_array_to_string(char *char_array)
    {
        string my_string(char_array);

        return my_string;
    }

    void dirCallback(const std_msgs::String::ConstPtr &msg)
    {
        this->data_dir = msg->data.c_str();
        create_directories();
    }

    void mag_cb(const sensor_msgs::MagneticField::ConstPtr &msg)
    {
        mag_data = *msg;
    }

    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        imu_data = *msg;
    }

    void alt_cb(const mavros_msgs::Altitude::ConstPtr &msg)
    {
        rel_alt = *msg;
    }

    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        gps_fix = *msg;
    }

    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        vel_gps = *msg;
    }

    void temp_cb(const sensor_msgs::Temperature::ConstPtr &msg)
    {
        temp_imu = *msg;
    }

    void trigtime_cb(const std_msgs::String::ConstPtr &msg)
    {
        trigtime = msg->data.c_str();
    }


};

int main(int argc, char **argv)
{
    // start the ros node
    ros::init(argc, argv, "boson_node");
    ros::NodeHandle nh;

    BosonUSMA boson_cam(&nh);
    boson_cam.print_help();
    
    //boson_cam.print_caminfo();
    
    boson_cam.openSensor();

    // Big while loop, continuously publish the images
    uint64_t n = 0;
    // TODO Adjust the rate to what is needed. currently slow for testing
    ros::Rate loop_rate(70); //This should be faster than the camera capture rate.
    auto prev_start = high_resolution_clock::now();
    while (ros::ok())
    {
        //auto start = high_resolution_clock::now();

        //boson_cam.getFrame(n);
        if(boson_cam.getFrame(n) ==0){        
            n++;
            ROS_INFO_THROTTLE(5,YEL "***** BOSON:  Grabbed Image %lu, and saved %d" WHT, n, boson_cam.get_savedcount());
        }    
        else{
            ROS_INFO(RED "ERROR: Boson get frame failed" WHT);

        }        
        // // Press 'q' to exit
        // if (waitKey(1) == 'q')
        // { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
        //     ROS_INFO(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !");
        //     break;
        // }
        ros::spinOnce();
        loop_rate.sleep();
        //auto delta_time = duration<double>(prev_start - start).count();
        //ROS_INFO(CYN "***** BOSON:  Loop time is %f" WHT, delta_time);
        
        //prev_start = start;

    }
    ROS_INFO("***** Boson:  Received ROS shutdown command");
}
