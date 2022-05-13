#ifndef DATALINK_HPP
#define DATALINK_HPP

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <thread>

#include <mutex>

class Datalink
{
	public:
	Datalink(std::string name, uint8_t sysid, uint8_t compid, bool heartbeat);
	~Datalink();

	private:
  	mavsdk::Mavsdk dc;
	std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough;
	std::shared_ptr<mavsdk::System> drone;
	std::mutex data_lock;
	std::thread downlink_thread;
	uint64_t uuid;

	void async_thread();
	void configure(uint8_t sysid, uint8_t compid, bool heartbeat);
	void connect();
	std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& dc);
	void start_downlink();
	void downlink_status();
};

#endif //DATALINK
