#include "real_deepracer_camera_sensor.h"
#include <argos3/core/utility/logging/argos_log.h>

using SBlob        = CCI_DeepracerCameraSensor::SBlob;
using TBlobs       = CCI_DeepracerCameraSensor::TBlobs;
using TBlobFilters = CRealDeepracerCameraSensor::TBlobFilters;

/****************************************/
/****************************************/

static void RGBtoHSV(unsigned char* pch_hsv, unsigned char* pch_rgb) {
    UInt8 unCMax = pch_rgb[0] > pch_rgb[1] ? pch_rgb[0] : pch_rgb[1];
    unCMax = unCMax > pch_rgb[2] ? unCMax : pch_rgb[2];
    UInt8 unCMin = pch_rgb[0] < pch_rgb[1] ? pch_rgb[0] : pch_rgb[1];
    unCMin = unCMin < pch_rgb[2] ? unCMin : pch_rgb[2];
    UInt8 unDelta = unCMax - unCMin;
    /* Value */
    pch_hsv[2] = unCMax;
    /* Hue */
    if(unDelta == 0) {
        pch_hsv[0] = 0;
    }
    else if(unCMax == pch_rgb[0]) {
        pch_hsv[0] = 42.5 * Mod((static_cast<Real>(pch_rgb[1]) - static_cast<Real>(pch_rgb[2])) / unDelta, 6);
    }
    else if(unCMax == pch_rgb[1]) {
        pch_hsv[0] = 42.5 * (2 + (static_cast<Real>(pch_rgb[2]) - static_cast<Real>(pch_rgb[0])) / unDelta);
    }
    else if(unCMax == pch_rgb[2]) {
        pch_hsv[0] = 42.5 * (4 + (static_cast<Real>(pch_rgb[0]) - static_cast<Real>(pch_rgb[1])) / unDelta);
    }
    /* Saturation */
    if(pch_hsv[2] == 0) {
        pch_hsv[1] = 0;
    }
    else {
        pch_hsv[1] = 255 * static_cast<Real>(unDelta) / pch_hsv[2];
    }
}

static ssize_t FilterMatch(TBlobFilters& t_blob_filters,
                           const unsigned char* pch_hsv) {
    for(size_t i = 0; i < t_blob_filters.size(); ++i) {
        if(t_blob_filters[i].Match(pch_hsv))
            return i;
    }
    return -1;
}

static SBlob* FindBlob(TBlobs& t_blobs,
                       UInt32 un_x,
                       UInt32 un_y,
                       UInt32 un_tolerance) {
    for(size_t i = 0; i < t_blobs.size(); ++i) {
        SBlob& sBlob = t_blobs[i];
        if(((un_x >= sBlob.Min.GetX() && un_x <= sBlob.Max.GetX()) ||
            (Abs(sBlob.Min.GetX() - un_x) < un_tolerance) ||
            (Abs(sBlob.Max.GetX() - un_x) < un_tolerance)) &&
           ((un_y >= sBlob.Min.GetY() && un_y <= sBlob.Max.GetY()) ||
            (Abs(sBlob.Min.GetY() - un_y) < un_tolerance) ||
            (Abs(sBlob.Max.GetY() - un_y) < un_tolerance))) {
            return &sBlob;
        }
    }
    return NULL;
}

static void AddToBlob(SBlob& s_blob,
                      UInt32 un_x,
                      UInt32 un_y) {
    s_blob.Min.Set(
            Min<Real>(s_blob.Min.GetX(), un_x),
            Min<Real>(s_blob.Min.GetY(), un_y));
    s_blob.Max.Set(
            Max<Real>(s_blob.Max.GetX(), un_x),
            Max<Real>(s_blob.Max.GetY(), un_y));
}

struct SCameraThreadParams {
    unsigned char* TempBuffer;
    unsigned char* ImgBuffer;
    UInt32 Width;
    UInt32 Height;
    TBlobs& WorkBuffer;
    TBlobs& ReadyBuffer;
    TBlobFilters& Filters;
    pthread_mutex_t& ReadyMutex;
    pthread_mutex_t& ImgMutex;
    bool& NewReadings;

    SCameraThreadParams(
            unsigned char* pch_temp_buffer,
            unsigned char* pch_img_buffer,
            UInt32 un_img_width,
            UInt32 un_img_height,
            TBlobs& c_blob_work_buffer,
            TBlobs& c_blob_ready_buffer,
            TBlobFilters& t_blob_filters,
            pthread_mutex_t& t_blob_ready_mutex,
            pthread_mutex_t& t_img_buffer_mutex,
            bool& b_new_blob_readings) :
            TempBuffer(pch_temp_buffer),
            ImgBuffer(pch_img_buffer),
            Width(un_img_width),
            Height(un_img_height),
            WorkBuffer(c_blob_work_buffer),
            ReadyBuffer(c_blob_ready_buffer),
            Filters(t_blob_filters),
            ReadyMutex(t_blob_ready_mutex),
            ImgMutex(t_img_buffer_mutex),
            NewReadings(b_new_blob_readings) {
    }

};

static void* CameraThread(void* pvoid_params) {
    /* Get parameters */
    SCameraThreadParams* ptParams =
            reinterpret_cast<SCameraThreadParams*>(pvoid_params);
    /* Data collection loop */
    while(1) {
        /* Cancellation point */
        pthread_testcancel();
        if(ptParams->ImgBuffer) {
            /* Process frame */
            unsigned char* pchRGBPx;
            unsigned char pchHSVPx[3];
            for(UInt32 x = 0; x < ptParams->Width; ++x) {
                for(UInt32 y = 0; y < ptParams->Height; ++y) {
                    /* Cancellation point */
                    pthread_testcancel();
                    /* Convert RGB pixel to HSV */
                    pchRGBPx = ptParams->ImgBuffer + 3 * (ptParams->Width * y + x);
                    RGBtoHSV(pchHSVPx, pchRGBPx);
                    /* Check if HSV pixel matches a filter */
                    ssize_t nFilter = FilterMatch(ptParams->Filters, pchHSVPx);
                    if(nFilter >= 0) {
                        /* Yes, check if HSV pixel can be added to an existing blob */
                        SBlob* psBlob = FindBlob(ptParams->WorkBuffer,
                                                 x, y,
                                                 ptParams->Filters[nFilter].Tolerance);
                        if(psBlob) {
                            /* Yes, add to blob */
                            AddToBlob(*psBlob, x, y);
                        }
                        else {
                            /* No, create new blob */
                            ptParams->WorkBuffer.push_back(
                                    SBlob(ptParams->Filters[nFilter].Color,
                                          CVector2(x, y)));
                        }
                    }
                }
            }
            /* Done with blobs, make them available */
            pthread_mutex_trylock(&ptParams->ReadyMutex);
            ptParams->NewReadings = true;
            ptParams->ReadyBuffer.swap(ptParams->WorkBuffer);
            pthread_mutex_unlock(&ptParams->ReadyMutex);
            /* Clear work buffer for new image */
            ptParams->WorkBuffer.clear();

            /* Update the Image buffer with the Temp buffer */
            pthread_mutex_trylock(&ptParams->ImgMutex);
            memcpy(&ptParams->ImgBuffer,&ptParams->TempBuffer,sizeof(ptParams->TempBuffer));
            pthread_mutex_unlock(&ptParams->ImgMutex);

        }
        else {
            LOGERR << "[WARNING] Error capturing camera frame: "
                   << std::endl;
        }
    }
}

/****************************************/
/****************************************/

CRealDeepracerCameraSensor::CRealDeepracerCameraSensor(const std::shared_ptr<rclcpp::Node>& pt_node_handle)
        :CRealDeepracerDevice(*pt_node_handle),
        m_bNewBlobReadings(false) {

    m_ptCameraSubscription = pt_node_handle->create_subscription<sensor_msgs::msg::Image>(
            "/camera_pkg/display_mjpeg",
            10,
            std::bind(
                    &CRealDeepracerCameraSensor::CameraCallback,
                    this,
                    std::placeholders::_1 ));
}

/****************************************/
/****************************************/

void CRealDeepracerCameraSensor::CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    pthread_mutex_trylock(&m_tImgBufferMutex);
    memcpy(&m_pchTempBuffer,&msg->data,sizeof(msg->data));
    pthread_mutex_unlock(&m_tImgBufferMutex);
    /* Parse XML */
    m_unWidth = msg->width;
    m_unHeight = msg->height;
}



/****************************************/
/****************************************/


CRealDeepracerCameraSensor::~CRealDeepracerCameraSensor() {
}

/****************************************/
/****************************************/

void CRealDeepracerCameraSensor::Init(TConfigurationNode& t_node) {
    try {

        GetNodeAttributeOrDefault(t_node, "image_width",  m_unWidth,  m_unWidth);
        GetNodeAttributeOrDefault(t_node, "image_height", m_unHeight, m_unHeight);
        m_pchImgBuffer = new unsigned char[3 * m_unWidth * m_unHeight];
        LOG << "[INFO] Camera initialized with image size ("
            << m_unWidth
            << ","
            << m_unHeight
            << ")" << std::endl;
        /* Create data mutex */
        if(pthread_mutex_init(&m_tBlobReadyMutex, NULL) != 0 &&
                pthread_mutex_init(&m_tImgBufferMutex, NULL) != 0) {
            delete[] m_pchImgBuffer;
            THROW_ARGOSEXCEPTION("pthread_mutex_init: " << strerror(errno));
        }
        /* Spawn worker thread */
        SCameraThreadParams sCameraThreadParams(
                m_pchTempBuffer,
                m_pchImgBuffer,
                m_unWidth,
                m_unHeight,
                m_tBlobWorkBuffer,
                m_tBlobReadyBuffer,
                m_tBlobFilters,
                m_tBlobReadyMutex,
                m_tImgBufferMutex,
                m_bNewBlobReadings);
        if(pthread_create(&m_tThread, NULL, CameraThread, &sCameraThreadParams) != 0) {
            pthread_mutex_destroy(&m_tBlobReadyMutex);
            delete[] m_pchImgBuffer;
            THROW_ARGOSEXCEPTION("pthread_create: " << strerror(errno));
        }
        LOG << "[INFO] Camera started" << std::endl;
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the camera", ex);
    }
}

/****************************************/
/****************************************/

void CRealDeepracerCameraSensor::Destroy() {
    /* Release mutex */
    pthread_mutex_unlock(&m_tBlobReadyMutex);
    pthread_mutex_unlock(&m_tImgBufferMutex);
    /* Stop worker thread */
    pthread_cancel(m_tThread);
    pthread_join(m_tThread, NULL);

    /* Destroy mutex */
    pthread_mutex_destroy(&m_tBlobReadyMutex);
    pthread_mutex_destroy(&m_tImgBufferMutex);

    /* Dispose of image buffer */
    delete[] m_pchImgBuffer;
    delete[] m_pchTempBuffer;
    LOG << "[INFO] Camera stopped" << std::endl;
}

/****************************************/
/****************************************/

const unsigned char* CRealDeepracerCameraSensor::GetPixels() const {
    return m_pchImgBuffer;
}

/****************************************/
/****************************************/

void CRealDeepracerCameraSensor::Do(Real f_elapsed_time) {
    /* Take latest reading from worker thread */
    pthread_mutex_trylock(&m_tBlobReadyMutex);
    if(m_bNewBlobReadings) {
        m_tBlobs.swap(m_tBlobReadyBuffer);
        m_bNewBlobReadings = false;
    }
    pthread_mutex_unlock(&m_tBlobReadyMutex);
}

/****************************************/
/****************************************/

bool CRealDeepracerCameraSensor::SBlobFilter::Match(const unsigned char* pch_hsv) {
    return
            Hue.WithinMinBoundIncludedMaxBoundIncluded(pch_hsv[0]) &&
            Saturation.WithinMinBoundIncludedMaxBoundIncluded(pch_hsv[1]) &&
            Value.WithinMinBoundIncludedMaxBoundIncluded(pch_hsv[2]);
}

/****************************************/
/****************************************/