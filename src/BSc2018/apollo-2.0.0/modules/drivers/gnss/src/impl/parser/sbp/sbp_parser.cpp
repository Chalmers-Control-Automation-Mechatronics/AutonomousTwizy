

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

	//std::vector<uint8_t> _buffer;

	sbp_state_t s;

	size_t _total_length = 0;

	::apollo::drivers::gnss::Gnss _gnss;
	::apollo::drivers::gnss::Imu _imu;
	::apollo::drivers::gnss::Ins _ins;
};

Parser* Parser::create_sbp() {
	return new SBPParser();
}

char _buffer[BUFFER_SIZE];
SBPParser::SBPParser() {
	_ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

	sbp_state_init(&s);
	sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &heartbeat_callback_node);

	ROS_WARN_STREAM("TJENA");
}

char hexstr[4];
char* int_to_hex( uint8_t i )
{
	sprintf(hexstr, "%02x", i);
	return hexstr;
}

u32 bytes_read;

u32 piksi_port_read(u8 *buff, u32 n, void *context){
	(void)context;

	//ROS_WARN_STREAM("sbp_process: Requesting " << n << " bytes");

	if((bytes_read + n) > BUFFER_SIZE)
	{
		ROS_WARN_STREAM("sbp_process: requested more bytes than BUFFER_SIZE!");
		return 0;
	}

	std::memcpy(buff, _buffer + bytes_read, n);

	bytes_read += n;

	return n;
}

void SBPParser::buffer_to_sbp_process(char* buffer, u32 len){
	bytes_read = 0;

	while(bytes_read < len){
		sbp_process(&s, &piksi_port_read);
		//ROS_WARN_STREAM("Bytes read: " << bytes_read);
		//ROS_WARN_STREAM("State " << (&s)->state);
		//ROS_WARN_STREAM("Message Type ")
	}
}

// Gets a parsed protobuf message. The caller must consume the message before
// calling another
// get_message() or update();
Parser::MessageType SBPParser::get_message(MessagePtr& message_ptr) {
	if (_data == nullptr) {
		return MessageType::NONE;
	}

	//ROS_WARN_STREAM("get_message: " << (_data_end - _data));

	u32 i = 0;

	u8 byte;
	while(_data < _data_end){
		byte = *_data;

		//ROS_WARN_STREAM("Byte: " << int_to_hex(byte));

		//_buffer.push_back(byte);
		_buffer[i] = byte;
		i++;

		if(byte == 0x55){
			//ROS_WARN_STREAM("Found preamble");
		}

		_data++;
	}

	//ROS_WARN_STREAM("Filled buffer");

	buffer_to_sbp_process(_buffer, i);



	if(_buffer[0] == 0x55){
		//ROS_WARN_STREAM("Found preamble");
	}else{
		ROS_WARN_STREAM("Could not find preamble");
	}

/*
	while (_data < _data_end) {
		if (_buffer.size() == 0) {  // Looking for SYNC0
			if (*_data == 0x55) {
				ROS_WARN_STREAM("Found SBP preamble");
				_buffer.push_back(*_data);
			}
			++_data;
		} else if (_buffer.size() < 3) {  // Looking for SYNC1
			_buffer.push_back(*_data++);
			continue;
		} else if (_buffer.size() == 3) {  // Looking for SYNC1


			_buffer.push_back(*_data++);
			continue;
		} else if (_buffer.size() == 2) {  // Looking for SYNC2
			switch (*_data) {
				case novatel::SYNC_2_LONG_HEADER:
					_buffer.push_back(*_data++);
					_header_length = sizeof(novatel::LongHeader);
					break;
				case novatel::SYNC_2_SHORT_HEADER:
					_buffer.push_back(*_data++);
					_header_length = sizeof(novatel::ShortHeader);
					break;
				default:
					_buffer.clear();
			}
		} else if (_header_length > 0) {  // Working on header.
			if (_buffer.size() < _header_length) {
				_buffer.push_back(*_data++);
			} else {
				if (_header_length == sizeof(novatel::LongHeader)) {
					_total_length = _header_length + novatel::CRC_LENGTH +
													reinterpret_cast<novatel::LongHeader*>(_buffer.data())
															->message_length;
				} else if (_header_length == sizeof(novatel::ShortHeader)) {
					_total_length =
							_header_length + novatel::CRC_LENGTH +
							reinterpret_cast<novatel::ShortHeader*>(_buffer.data())
									->message_length;
				} else {
					ROS_ERROR("Incorrect _header_length. Should never reach here.");
					_buffer.clear();
				}
				_header_length = 0;
			}
		} else if (_total_length > 0) {
			if (_buffer.size() < _total_length) {  // Working on body.
				_buffer.push_back(*_data++);
				continue;
			}
			MessageType type = prepare_message(message_ptr);
			_buffer.clear();
			_total_length = 0;
			if (type != MessageType::NONE) {
				return type;
			}
		}
	}*/
	return MessageType::NONE;
}

}
}
}