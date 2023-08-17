#ifndef REAL_DEEPRACER_CAMERA_SENSOR_H
#define REAL_DEEPRACER_CAMERA_SENSOR_H
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_camera_sensor.h>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer_device.h>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer.h>
#include <sensor_msgs/msg/image.hpp>
#include <pthread.h>

using namespace argos;

class CRealDeepracerCameraSensor :
        public CCI_DeepracerCameraSensor,
        public CRealDeepracerDevice {

public:

    CRealDeepracerCameraSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle);

    virtual ~CRealDeepracerCameraSensor();

    virtual void Init(TConfigurationNode& t_node);

    virtual void Destroy();

    virtual const unsigned char* GetPixels() const;

    virtual void Do(Real f_elapsed_time);

public:

    struct SBlobFilter {
        CColor Color;
        CRange<UInt8> Hue;
        CRange<UInt8> Saturation;
        CRange<UInt8> Value;
        UInt32 Tolerance;

        bool Match(const unsigned char* pch_hsv);
    };

    typedef std::vector<SBlobFilter> TBlobFilters;

private:
    void CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_ptCameraSubscription;
    /* Image buffer */
    unsigned char*  m_pTempBuffer;
    unsigned char* m_pchImgBuffer;
    /* Work buffer for blob detection */
    CCI_DeepracerCameraSensor::TBlobs m_tBlobWorkBuffer;
    /* Ready buffer for blob detection */
    CCI_DeepracerCameraSensor::TBlobs m_tBlobReadyBuffer;
    /* Vector of blob filters */
    TBlobFilters m_tBlobFilters;
    /* Thread handle */
    pthread_t m_tThread;
    /* Blob ready buffer mutex */
    pthread_mutex_t m_tBlobReadyMutex;
    pthread_mutex_t m_tImgBufferMutex;
    /* True when new blob readings are available */
    bool m_bNewBlobReadings;
};

#endif

