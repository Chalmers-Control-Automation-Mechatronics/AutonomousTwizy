

#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include "gnss/parser.h"
#include "proto/gnss.pb.h"
#include "proto/gnss_best_pose.pb.h"
#include "proto/gnss_raw_observation.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"
#include "rtcm/rtcm_decode.h"
#include "util/time_conversion.h"

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/piksi.h>

namespace apollo {
namespace drivers {
namespace gnss {

namespace {

constexpr size_t BUFFER_SIZE = 1024;
constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

static sbp_msg_callbacks_node_t heartbeat_callback_node;

// Functions to convert little endian bytes to nice types
s32 bytesToSignedInt(u8 start, u8 msg[]){
	return msg[start] | msg[start + 1] << 8 | msg[start + 2] << 16 | msg[start + 3] << 24;
}

u32 bytesToUnsignedInt(u8 start, u8 msg[]){
	return (u32)bytesToSignedInt(start, msg);
}

s16 bytesToSignedShort(u8 start, u8 msg[]){
	return msg[start] | msg[start + 1] << 8;
}

u16 bytesToUnsignedShort(u8 start, u8 msg[]){
	return (u16)bytesToSignedShort(start, msg);
}

char _buffer[BUFFER_SIZE];
u32 bytes_read;
u32 buffer_data_size;
Parser::MessageType current_message;

}

class SBPParser : public Parser {
public:
	SBPParser();

	virtual MessageType get_message(MessagePtr& message_ptr);

private:
	void buffer_to_sbp_process(char* buffer, u32 len);

	static void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	double _gps_seconds_base = -1.0;

	double _gyro_scale = 0.0;

	double _accel_scale = 0.0;

	float _imu_measurement_span = 0.0;

	int _imu_frame_mapping = 5;

	double _imu_measurement_time_previous = -1.0;

	sbp_state_t s;

	size_t _total_length = 0;

	::apollo::drivers::gnss::Gnss _gnss;
	::apollo::drivers::gnss::GnssBestPose _bestpos;
	::apollo::drivers::gnss::Imu _imu;
	::apollo::drivers::gnss::Ins _ins;
	::apollo::drivers::gnss::InsStat _ins_stat;
	::apollo::drivers::gnss::GnssEphemeris _gnss_ephemeris;
	::apollo::drivers::gnss::EpochObservation _gnss_observation;
};

Parser* Parser::create_sbp() {
	return new SBPParser();
}

SBPParser::SBPParser() {
	current_message = MessageType::NONE;

	_ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

	sbp_state_init(&s);
	sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &this->heartbeat_callback, NULL, &heartbeat_callback_node);
}

char hexstr[4];
char* int_to_hex( uint8_t i )
{
	sprintf(hexstr, "%02x", i);
	return hexstr;
}

/*
SBP Callbacks
*/
void SBPParser::heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	ROS_WARN_STREAM("Heartbeat");

	u32 flags = bytesToUnsignedInt(0, msg);
	ROS_WARN_STREAM("Antenna Present " << ((flags & 0x80000000) > 0 ? "yes" : "no"));

	current_message = MessageType::NONE;
}
/*
End of callbacks
*/

u32 piksi_port_read(u8 *buff, u32 n, void *context){
	(void)context;

	// SBP library calls this function
	// buff is the buffer in which it expects data when it's returned
	// n is the amount of bytes which the library requests
	// the function should return the amount of bytes read

	// Make sure we don't read more bytes than we have
	n = std::min(n, buffer_data_size - bytes_read);

	// Copy our buffer into SBP's buffer
	std::memcpy(buff, _buffer + bytes_read, n);

	bytes_read += n;

	return n;
}

void SBPParser::buffer_to_sbp_process(char* buffer, u32 len){
	bytes_read = 0;
	buffer_data_size = len;

	// Feed the entire buffer until its fully read
	while(bytes_read < len){
		sbp_process(&s, &piksi_port_read);
	}
}

// Gets a parsed protobuf message. The caller must consume the message before
// calling another
// get_message() or update();
Parser::MessageType SBPParser::get_message(MessagePtr& message_ptr) {
	//Once we enter this method, _data and _data_end will be populated with pointers
	//_data points to the beginning of the data (the first byte)
	//_data_end points to beginning + data size
	//Thus, to get the data size we can do (_data_end - _data)

	if (_data == nullptr) {
		return MessageType::NONE;
	}

	// Copy the message data to a buffer
	u32 data_size = (_data_end - _data);
	if(data_size > BUFFER_SIZE){
		ROS_WARN_STREAM("Incoming data is too big for buffer! " << data_size << " bytes (Buffer size is " << BUFFER_SIZE << " bytes)");
		return MessageType::NONE;
	}

	std::memcpy(_buffer, _data, data_size);

	// Reset current message
	// The SBP callbacks will set this variable to the appropriate one
	current_message = MessageType::NONE;

	// Feed the buffer into the SBP library's functions
	buffer_to_sbp_process(_buffer, data_size);

	return current_message;
}

}
}
}