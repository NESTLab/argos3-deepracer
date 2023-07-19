#ifndef ARGOS3_DEEPRACER_REAL_DEEPRACER_CAMERA_SENSOR_H
#define ARGOS3_DEEPRACER_REAL_DEEPRACER_CAMERA_SENSOR_H


class CRealDeepracerCameraSensor {

    CRealDeepracerCameraSensor(std::shared_ptr<SubscriptionT >  sub);
    virtual ~CRealDeepracerCameraSensor();
    virtual void Do(Real f_elapsed_time);
};


#endif //ARGOS3_DEEPRACER_REAL_DEEPRACER_CAMERA_SENSOR_H
