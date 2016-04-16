#include <iostream>
#include "Task.hpp"

using namespace middleware_communication;

int Middleware::extractPacket(uint8_t const *buffer, size_t buffer_size) const {

    if (buffer[0] != '<') {
        return -1;
    }

    size_t offset = 1;

    while (offset < buffer_size) {
        if (buffer[offset++] == '>') {
            return offset;
        }
    }

    return 0;

}

void Middleware::parseMessage(uint8_t const* buffer, size_t size) {
    //Message format
    //<gyro_x;gyro_y;gyro_z;acc_x;acc_y;acc_z;mag_x;mag_y;mag_z;pressure;temp;>

    float gyro_x, gyro_y, gyro_z;
    float acc_x, acc_y, acc_z;
    float mag_x, mag_y, mag_z;
    float pressure;
    float temp;

    sscanf((const char*)buffer, "<%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;>",
            &gyro_x, &gyro_y, &gyro_z,
            &acc_x, &acc_y, &acc_z,
            &mag_x, &mag_y, &mag_z,
            &pressure,
            &temp);

    imu.time = base::Time::now();
    imu.acc = base::Vector3d(acc_x, acc_y, acc_z);
    imu.mag = base::Vector3d(mag_x, mag_y, mag_z);
    imu.gyro = base::Vector3d(gyro_x, gyro_y, gyro_z);
    temperature = base::Temperature::fromCelsius(temp);
    pressureBar = pressure;
}


/**
 *The Middleware Orogen Task
 */
Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    middleware.reset(new Middleware());
    middleware->setReadTimeout(base::Time::fromSeconds(_timeout.value()));
    middleware->openURI(_device.value());

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    updateMotors();
    updateSensors();
}

void Task::updateSensors()
{
    try {
        if (middleware->writePacket(reinterpret_cast<const uint8_t*>("<L>\n"),4, 100)) {
            std::vector<uint8_t> buffer;
            buffer.resize(Middleware::MAX_BUFFER_SIZE);

            size_t size = middleware->readPacket(&buffer[0], buffer.size());
            middleware->parseMessage(&buffer[0], size);

            const base::samples::IMUSensors& imu = middleware->getImuSensors();
            const base::Temperature& temperature = middleware->getTemperature();
            double pressure = middleware->getPressureBar();

            //Write in output ports
            _imu_sensors_samples.write(imu);
            _temperature_samples.write(temperature);
            _pressure_bar_samples.write(pressure);
        }
    }
    catch (std::runtime_error e) {
        std::cout << e.what() << std::endl;
    }

}

void Task::updateMotors() {
    base::samples::Joints joints;
    if (_motor_inputs.read(joints) == RTT::NewData) {
        std::stringstream ss;
        ss << "<C;";
        for (size_t i = 0; i < joints.size(); i++) ss << joints.elements[i].effort << ";";
        ss << ">\n";
        std::string cmd = ss.str();
        try {
            std::cout << cmd.c_str();
            std::cout << "size: " << cmd.size() << std::endl;
            std::cout << "length: " << cmd.length() << std::endl;
            middleware->writePacket(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.length(), 100);
        }
        catch (std::runtime_error e) {
            std::cout << e.what() << std::endl;
        }
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
