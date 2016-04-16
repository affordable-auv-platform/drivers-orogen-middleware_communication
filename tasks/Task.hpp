#ifndef MIDDLEWARE_COMMUNICATION_TASK_TASK_HPP
#define MIDDLEWARE_COMMUNICATION_TASK_TASK_HPP

#include <iodrivers_base/Driver.hpp>
#include <base/samples/IMUSensors.hpp>
#include <base/Temperature.hpp>

#include "middleware_communication/TaskBase.hpp"

namespace middleware_communication {

class Middleware : public iodrivers_base::Driver {
public:
    static const int MAX_BUFFER_SIZE = 256;

    Middleware() : iodrivers_base::Driver(MAX_BUFFER_SIZE),
                         temperature(),
                         imu(),
                         pressureBar(0.0){}

    virtual ~Middleware() {}

    int extractPacket(uint8_t const *buffer, size_t buffer_size) const;
    void parseMessage(uint8_t const* buffer, size_t size);

    const base::samples::IMUSensors& getImuSensors() const {
        return imu;
    }

    const base::Temperature& getTemperature() const {
        return temperature;
    }

    double getPressureBar() const {
        return pressureBar;
    }

private:

    base::samples::IMUSensors imu;
    base::Temperature temperature;
    double pressureBar;
};


class Task: public TaskBase {
    friend class TaskBase;
private:

    void updateMotors();

    void updateSensors();

protected:
    boost::shared_ptr<Middleware> middleware;
public:
    Task(std::string const& name = "middleware_communication::Task");

    Task(std::string const& name, RTT::ExecutionEngine* engine);

    ~Task();

    bool configureHook();

    bool startHook();

    void updateHook();

    void errorHook();

    void stopHook();

    void cleanupHook();
};

}

#endif

