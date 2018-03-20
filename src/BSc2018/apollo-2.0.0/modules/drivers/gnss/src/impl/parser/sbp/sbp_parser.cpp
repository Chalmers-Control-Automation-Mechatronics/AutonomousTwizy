

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

void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	ROS_WARN_STREAM("Heartbeat");

	u32 flags = bytesToUnsignedInt(0, msg);
	ROS_WARN_STREAM("Antenna Present " << ((flags & 0x80000000) > 0 ? "yes" : "no"));
}

char _buffer[BUFFER_SIZE];
u32 bytes_read;

}

class SBPParser : public Parser {
public:
	SBPParser();

	virtual MessageType get_message(MessagePtr& message_ptr);

private:
	//u32 piksi_port_read(u8 *buff, u32 n, void *context);
	void buffer_to_sbp_process(char* buffer, u32 len);

	double _gps_seconds_base = -1.0;

	double _gyro_scale = 0.0;

	double _accel_scale = 0.0;

	float _imu_measurement_span = 0.0;

	int _imu_frame_mapping = 5;

	double _imu_measurement_time_previous = -1.0;

	sbp_state_t s;

	size_t _total_length = 0;

	::apollo::drivers::gnss::Gnss _gnss;
	::apollo::drivers::gnss::Imu _imu;
	::apollo::drivers::gnss::Ins _ins;
};

Parser* Parser::create_sbp() {
	return new SBPParser();
}

SBPParser::SBPParser() {
	_ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

	sbp_state_init(&s);
	sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &heartbeat_callback_node);
}

char hexstr[4];
char* int_to_hex( uint8_t i )
{
	sprintf(hexstr, "%02x", i);
	return hexstr;
}

u32 piksi_port_read(u8 *buff, u32 n, void *context){
	(void)context;

	if((bytes_read + n) > BUFFER_SIZE){
		ROS_WARN_STREAM("requested more bytes than BUFFER_SIZE!");
		return 0;
	}

	// Copy our buffer into SBP's buffer
	std::memcpy(buff, _buffer + bytes_read, n);

	bytes_read += n;

	return n;
}

void SBPParser::buffer_to_sbp_process(char* buffer, u32 len){
	bytes_read = 0;
	// Feed the entire buffer until its fully read
	while(bytes_read < len){
		sbp_process(&s, &piksi_port_read);
	}
}

// Gets a parsed protobuf message. The caller must consume the message before
// calling another
// get_message() or update();
Parser::MessageType SBPParser::get_message(MessagePtr& message_ptr) {
	if (_data == nullptr) {
		return MessageType::NONE;
	}

	// Copy the message data to a buffer
	u32 i = 0;
	while(_data < _data_end){
		if(i >= BUFFER_SIZE){
			ROS_WARN_STREAM("Can't hold more data in buffer");
			break;
		}

		_buffer[i++] = *(_data++);
	}

	// Feed the buffer into the SBP library's functions
	buffer_to_sbp_process(_buffer, i);

	return MessageType::NONE;
}

}
}
}