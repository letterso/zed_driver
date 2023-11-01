#include <iostream>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <zed_driver/sl_tools.h>
using namespace sl;

#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

// camera
Camera zed;
std::queue<sensor_msgs::ImagePtr> left_image_queue, right_image_queue;
std::queue<sensor_msgs::ImuPtr> sensors_queue;

// image data
ros::Publisher pub_left_image, pub_right_image;
unsigned int image_max_size = 100;
std::mutex image_mutex;
void images_thread_func()
{
    sl::Mat left_image, right_image;
    uint64_t last_time = 0;
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        // A new image is available if grab() returns ERROR_CODE::SUCCESS
        if (zed.grab() == ERROR_CODE::SUCCESS)
        {

            // Get the raw image
            zed.retrieveImage(left_image, VIEW::LEFT_UNRECTIFIED_GRAY, sl::MEM::CPU);
            zed.retrieveImage(right_image, VIEW::RIGHT_UNRECTIFIED_GRAY, sl::MEM::CPU);

            // Display the image resolution and its acquisition timestamp
            // std::cout << "Left image resolution: " << left_image.getWidth() << "x" << left_image.getHeight() << " || Image timestamp: " << left_image.timestamp.data_ns << std::endl;
            // std::cout << "Right image resolution: " << right_image.getWidth() << "x" << right_image.getHeight() << " || Image timestamp: " << right_image.timestamp.data_ns << std::endl;

            uint64_t time_diff = left_image.timestamp.getMilliseconds() - last_time;
            if (time_diff > 40 || time_diff < 20)
                std::cout << "Time differ : " << left_image.timestamp.getMilliseconds() - last_time << std::endl;
            last_time = left_image.timestamp.getMilliseconds();

            // to sensor_msgs image
            sensor_msgs::ImagePtr left_img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
            sensor_msgs::ImagePtr right_img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
            sl_tools::imageToROSmsg(left_img_msg_ptr, left_image, "ZED2", sl_tools::slTime2Ros(left_image.timestamp));
            sl_tools::imageToROSmsg(right_img_msg_ptr, right_image, "ZED2", sl_tools::slTime2Ros(right_image.timestamp));

            image_mutex.lock();
            if (left_image_queue.size() > image_max_size)
            {
                left_image_queue.pop();
                right_image_queue.pop();
            }
            left_image_queue.push(left_img_msg_ptr);
            right_image_queue.push(right_img_msg_ptr);
            image_mutex.unlock();

            pub_left_image.publish(left_img_msg_ptr);
            pub_right_image.publish(right_img_msg_ptr);
        }

        loop_rate.sleep();
    }
}

// sensor data
// Basic structure to compare timestamps of a sensor. Determines if a specific sensor data has been updated or not.
struct TimestampHandler
{

    // Compare the new timestamp to the last valid one. If it is higher, save it as new reference.
    inline bool isNew(Timestamp &ts_curr, Timestamp &ts_ref)
    {
        bool new_ = ts_curr > ts_ref;
        if (new_)
            ts_ref = ts_curr;
        return new_;
    }
    // Specific function for IMUData.
    inline bool isNew(SensorsData::IMUData &imu_data)
    {
        return isNew(imu_data.timestamp, ts_imu);
    }
    // Specific function for MagnetometerData.
    inline bool isNew(SensorsData::MagnetometerData &mag_data)
    {
        return isNew(mag_data.timestamp, ts_mag);
    }
    // Specific function for BarometerData.
    inline bool isNew(SensorsData::BarometerData &baro_data)
    {
        return isNew(baro_data.timestamp, ts_baro);
    }

    Timestamp ts_imu = 0, ts_baro = 0, ts_mag = 0; // Initial values
};

ros::Publisher pub_sensor_data;
std::mutex sensor_mutex;
unsigned int sensor_max_size = 10000;
void sensor_thread_func()
{
    // Used to store sensors timestamps and check if new data is available.
    TimestampHandler ts;

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        sl::SensorsData sens_data;
        if (zed.getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT) == ERROR_CODE::SUCCESS)
        {

            if (ts.isNew(sens_data.imu))
            {
                sensor_msgs::ImuPtr imuMsg = boost::make_shared<sensor_msgs::Imu>();
                imuMsg->header.stamp = sl_tools::slTime2Ros(sens_data.imu.timestamp);

                imuMsg->header.frame_id = "ZED2";

                imuMsg->orientation.x = sens_data.imu.pose.getOrientation()[0];
                imuMsg->orientation.y = sens_data.imu.pose.getOrientation()[1];
                imuMsg->orientation.z = sens_data.imu.pose.getOrientation()[2];
                imuMsg->orientation.w = sens_data.imu.pose.getOrientation()[3];

                imuMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
                imuMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
                imuMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;

                imuMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
                imuMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
                imuMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

                for (int i = 0; i < 3; ++i)
                {
                    int r = 0;

                    if (i == 0)
                    {
                        r = 0;
                    }
                    else if (i == 1)
                    {
                        r = 1;
                    }
                    else
                    {
                        r = 2;
                    }

                    imuMsg->orientation_covariance[i * 3 + 0] = sens_data.imu.pose_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
                    imuMsg->orientation_covariance[i * 3 + 1] = sens_data.imu.pose_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
                    imuMsg->orientation_covariance[i * 3 + 2] = sens_data.imu.pose_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;

                    imuMsg->linear_acceleration_covariance[i * 3 + 0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
                    imuMsg->linear_acceleration_covariance[i * 3 + 1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
                    imuMsg->linear_acceleration_covariance[i * 3 + 2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

                    imuMsg->angular_velocity_covariance[i * 3 + 0] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
                    imuMsg->angular_velocity_covariance[i * 3 + 1] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
                    imuMsg->angular_velocity_covariance[i * 3 + 2] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
                }

                sensor_mutex.lock();
                if (sensors_queue.size() > sensor_max_size)
                    sensors_queue.pop();
                sensors_queue.push(imuMsg);
                sensor_mutex.unlock();

                pub_sensor_data.publish(imuMsg);
            }
        }

        loop_rate.sleep();
    }
}

// bag
rosbag::Bag bag;
void bag_thread_func()
{
    std::vector<sensor_msgs::ImuPtr> imu_msg_cache;
    imu_msg_cache.reserve(100);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        image_mutex.lock();
        if (!left_image_queue.empty() && !right_image_queue.empty())
        {
            sensor_msgs::ImagePtr left_img_msg_ptr = left_image_queue.front();
            sensor_msgs::ImagePtr right_img_msg_ptr = right_image_queue.front();
            left_image_queue.pop();
            right_image_queue.pop();
            image_mutex.unlock();
            bag.write("/zed2/zed_node/left_raw/image_raw_gray", left_img_msg_ptr->header.stamp, left_img_msg_ptr);
            bag.write("/zed2/zed_node/right_raw/image_raw_gray", right_img_msg_ptr->header.stamp, right_img_msg_ptr);
        }
        else
        {
            image_mutex.unlock();
        }

        sensor_mutex.lock();
        if (!sensors_queue.empty())
        {
            while (!sensors_queue.empty())
            {
                imu_msg_cache.emplace_back(sensors_queue.front());
                sensors_queue.pop();
            }
            sensor_mutex.unlock();
            for (int i = imu_msg_cache.size() - 1; i >= 0; i--)
            {
                bag.write("/zed2/zed_node/imu/data", imu_msg_cache[i]->header.stamp, imu_msg_cache[i]);
            }
            imu_msg_cache.clear();
        }
        else
        {
            sensor_mutex.unlock();
        }

        loop_rate.sleep();
    }

    // load queue in bag
    while (!left_image_queue.empty())
    {
        sensor_msgs::ImagePtr left_img_msg_ptr = left_image_queue.front();
        sensor_msgs::ImagePtr right_img_msg_ptr = right_image_queue.front();
        left_image_queue.pop();
        right_image_queue.pop();
        bag.write("/zed2/zed_node/left_raw/image_raw_gray", left_img_msg_ptr->header.stamp, left_img_msg_ptr);
        bag.write("/zed2/zed_node/right_raw/image_raw_gray", right_img_msg_ptr->header.stamp, right_img_msg_ptr);
    }
    while (!sensors_queue.empty())
    {
        sensor_msgs::ImuPtr imu_msg_ptr = sensors_queue.front();
        sensors_queue.pop();
        bag.write("/zed2/zed_node/imu/data", imu_msg_ptr->header.stamp, imu_msg_ptr);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_driver_node");
    ros::start();
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle nh;
    pub_left_image = nh.advertise<sensor_msgs::Image>("/zed2/zed_node/left_raw/image_raw_gray", 30);
    pub_right_image = nh.advertise<sensor_msgs::Image>("/zed2/zed_node/right_raw/image_raw_gray", 30);
    pub_sensor_data = nh.advertise<sensor_msgs::Imu>("/zed2/zed_node/imu/data", 400);

    // Set configuration parameters
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::VGA; // Use HD720 HD1080 video mode
    init_parameters.camera_fps = 30;                     // Set fps at 30

    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

    init_parameters.async_grab_camera_recovery = true;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.sdk_gpu_id = 0;
    init_parameters.sdk_verbose = false;
    init_parameters.camera_image_flip = false;
    init_parameters.camera_disable_self_calib = false;

    init_parameters.enable_image_enhancement = true; // Always active

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        std::cout << "Error " << returned_state << ", exit program." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Open ZED camera" << std::endl;

    int value;
    sl::VIDEO_SETTINGS setting;
    sl::ERROR_CODE err;

    setting = sl::VIDEO_SETTINGS::BRIGHTNESS;
    err = zed.getCameraSettings(setting, value);
    int CamBrightness = 4;
    if (err == sl::ERROR_CODE::SUCCESS && value != CamBrightness)
    {
        err = zed.setCameraSettings(setting, CamBrightness);
    }
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        ROS_WARN_STREAM("Error setting parameter " << sl::toString(setting) << ": " << sl::toString(err));
    }
    else
    {
        ROS_INFO_STREAM(sl::toString(setting) << " changed: " << CamBrightness << " <- " << value);
    }

    setting = sl::VIDEO_SETTINGS::CONTRAST;
    err = zed.getCameraSettings(setting, value);
    int CamContrast = 4;
    if (err == sl::ERROR_CODE::SUCCESS && value != CamContrast)
    {
        err = zed.setCameraSettings(setting, CamContrast);
    }
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        ROS_WARN_STREAM("Error setting parameter " << sl::toString(setting) << ": " << sl::toString(err));
    }
    else
    {
        ROS_INFO_STREAM(sl::toString(setting) << " changed: " << CamContrast << " <- " << value);
    }

    setting = sl::VIDEO_SETTINGS::HUE;
    err = zed.getCameraSettings(setting, value);
    int CamHue = 0;
    if (err == sl::ERROR_CODE::SUCCESS && value != CamHue)
    {
        err = zed.setCameraSettings(setting, CamHue);
    }
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        ROS_WARN_STREAM("Error setting parameter " << sl::toString(setting) << ": " << sl::toString(err));
    }
    else
    {
        ROS_INFO_STREAM(sl::toString(setting) << " changed: " << CamHue << " <- " << value);
    }

    setting = sl::VIDEO_SETTINGS::SATURATION;
    err = zed.getCameraSettings(setting, value);
    int CamSaturation = 4;
    if (err == sl::ERROR_CODE::SUCCESS && value != CamSaturation)
    {
        err = zed.setCameraSettings(setting, CamSaturation);
    }
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        ROS_WARN_STREAM("Error setting parameter " << sl::toString(setting) << ": " << sl::toString(err));
    }
    else
    {
        ROS_INFO_STREAM(sl::toString(setting) << " changed: " << CamSaturation << " <- " << value);
    }

    setting = sl::VIDEO_SETTINGS::SHARPNESS;
    err = zed.getCameraSettings(setting, value);
    int CamSharpness = 4;
    if (err == sl::ERROR_CODE::SUCCESS && value != CamSharpness)
    {
        err = zed.setCameraSettings(setting, CamSharpness);
    }
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        ROS_WARN_STREAM("Error setting parameter " << sl::toString(setting) << ": " << sl::toString(err));
    }
    else
    {
        ROS_INFO_STREAM(sl::toString(setting) << " changed: " << CamSharpness << " <- " << value);
    }

    setting = sl::VIDEO_SETTINGS::GAMMA;
    err = zed.getCameraSettings(setting, value);
    int CamGamma = 8;
    if (err == sl::ERROR_CODE::SUCCESS && value != CamGamma)
    {
        err = zed.setCameraSettings(setting, CamGamma);
    }
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        ROS_WARN_STREAM("Error setting parameter " << sl::toString(setting) << ": " << sl::toString(err));
    }
    else
    {
        ROS_INFO_STREAM(sl::toString(setting) << " changed: " << CamGamma << " <- " << value);
    }

    setting = sl::VIDEO_SETTINGS::AEC_AGC;
    err = zed.getCameraSettings(setting, value);
    int aec_agc = 1;
    if (err == sl::ERROR_CODE::SUCCESS && value != aec_agc)
    {
        err = zed.setCameraSettings(setting, aec_agc);
    }
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        ROS_WARN_STREAM("Error setting parameter " << sl::toString(setting) << ": " << sl::toString(err));
    }
    else
    {
        ROS_INFO_STREAM(sl::toString(setting) << " changed: " << aec_agc << " <- " << value);
    }

    setting = sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO;
    err = zed.getCameraSettings(setting, value);
    int wb_auto = 1;
    if (err == sl::ERROR_CODE::SUCCESS && value != wb_auto)
    {
        err = zed.setCameraSettings(setting, wb_auto);
    }
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        ROS_WARN_STREAM("Error setting parameter " << sl::toString(setting) << ": " << sl::toString(err));
    }
    else
    {
        ROS_INFO_STREAM(sl::toString(setting) << " changed: " << wb_auto << " <- " << value);
    }

    // open bag
    bag.open("zed_date.bag", rosbag::bagmode::Write);

    // run
    std::thread image_thread = std::thread(images_thread_func);
    std::thread sensor_thread = std::thread(sensor_thread_func);
    std::thread bag_thread = std::thread(bag_thread_func);
    ros::spin();

    // stop thread
    image_thread.join();
    sensor_thread.join();
    bag_thread.join();

    // close bag and camera
    bag.close();
    zed.close();

    ros::shutdown();
    return EXIT_SUCCESS;
}
