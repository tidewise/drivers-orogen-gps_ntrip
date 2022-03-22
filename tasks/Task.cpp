/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <gps_base/BaseTypes.hpp>
#include <gps_base/rtcm3.hpp>

using namespace gps_ntrip;
using namespace std;

constexpr int PACKETIZER_BUFFER_SIZE = 4096;

struct RTCMPacketizer : public iodrivers_base::Driver {
    RTCMPacketizer()
        : iodrivers_base::Driver(PACKETIZER_BUFFER_SIZE) {}

    int extractPacket(uint8_t const* buffer, size_t size) const override {
        return gps_base::rtcm3::extractPacket(buffer, size);
    }
};

Task::Task(std::string const& name)
: TaskBase(name)
{
    mRTCMPacketizer.reset(new RTCMPacketizer);
    mRTCMPacketizer->openURI("test://");
    mRTCMPacketizerStream = dynamic_cast<iodrivers_base::TestStream*>(
        mRTCMPacketizer->getMainStream()
    );

    mNTRIP.set_report_interval(10);
    mNTRIP.OnReceived([this] (const char *buffer, int size) {
        uint8_t forwardBuffer[4096];

        mRTCMPacketizerStream->pushDataToDriver(vector<uint8_t>(buffer, buffer + size));
        try {
            while (true) {
                size_t size = mRTCMPacketizer->readPacket(
                    forwardBuffer, PACKETIZER_BUFFER_SIZE, base::Time()
                );

                iodrivers_base::RawPacket packet;
                packet.time = base::Time::now();
                packet.data.insert(packet.data.end(), forwardBuffer, forwardBuffer + size);
                _rtcm_out.write(packet);
            }
        }
        catch(iodrivers_base::TimeoutError&) {}
    });
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

    mNTRIP.Init(
        _host.get(), _port.get(),
        _user.get(), _password.get(), _mountpoint.get()
    );

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    state(WAITING_FOR_POSITION);
    return true;
}
void Task::updateHook() {
    TaskBase::updateHook();

    gps_base::Solution gpsSolution;
    if (_gps_solution.read(gpsSolution, false) == RTT::NewData) {
        processCurrentPosition(gpsSolution);
    }

    if (state() == WAITING_FOR_POSITION) {
        return;
    }

    if (!mNTRIP.service_is_running()) {
        return exception(CONNECTION_CLOSED);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    mNTRIP.Stop();
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::processCurrentPosition(gps_base::Solution const& position) {
    switch(position.positionType) {
        case gps_base::AUTONOMOUS:
        case gps_base::DIFFERENTIAL:
        case gps_base::RTK_FIXED:
        case gps_base::RTK_FLOAT:
            break;
        default:
            return;
    }

    mNTRIP.set_location(position.latitude, position.longitude);

    if (state() == WAITING_FOR_POSITION) {
        if (!startClient()) {
            return exception(CONNECTION_FAILED);
        }
        state(RUNNING);
    }
}
bool Task::startClient() {
    if (!mNTRIP.Run()) {
        return false;
    }

    while (!mNTRIP.service_is_running()) {
        usleep(10000);
    }
    return true;
}