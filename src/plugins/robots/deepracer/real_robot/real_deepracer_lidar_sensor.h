#ifndef ARGOS3_DEEPRACER_REAL_DEEPRACER_LIDAR_SENSOR_H
#define ARGOS3_DEEPRACER_REAL_DEEPRACER_LIDAR_SENSOR_H


class CRealDeepracerLidarSensor {

    CRealDeepracerLidarSensor(std::shared_ptr<SubscriptionT >  sub);
    virtual ~CRealDeepracerLidarSensor();
    virtual void Do(Real f_elapsed_time);
};


#endif //ARGOS3_DEEPRACER_REAL_DEEPRACER_LIDAR_SENSOR_H
