

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

#include <libsbp/acquisition.h>
#include <libsbp/gnss.h>
#include <libsbp/imu.h>
#include <libsbp/mag.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/piksi.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/tracking.h>

namespace apollo {
namespace drivers {
namespace gnss {

namespace {

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;
constexpr size_t BUFFER_SIZE = 1024;
constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t gps_time_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
static sbp_msg_callbacks_node_t orient_euler_callback_node;
static sbp_msg_callbacks_node_t angular_rate_callback_node;
static sbp_msg_callbacks_node_t imu_raw_callback_node;

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

void bytesToDouble(double* out, u8 start, u8 msg[]){
	std::memcpy(out, msg + start, 8);
}

u16 gpsweek = 0;
double getGPSTime(u32 time_of_week, s32 ns_offset = 0){
	if(gpsweek == 0)
		return 0.0;

	return gpsweek * SECONDS_PER_WEEK + time_of_week * 1e-3 + ns_offset * 1e-9;
}

void setGPSWeek(u16 week){
	gpsweek = week;
}

char _buffer[BUFFER_SIZE];
u32 bytes_read;
u32 buffer_data_size;
Parser::MessageType current_message;

::apollo::drivers::gnss::Gnss _gnss;
::apollo::drivers::gnss::GnssBestPose _bestpos;
::apollo::drivers::gnss::Imu _imu;
::apollo::drivers::gnss::Ins _ins;
::apollo::drivers::gnss::InsStat _ins_stat;
::apollo::drivers::gnss::GnssEphemeris _gnss_ephemeris;
::apollo::drivers::gnss::EpochObservation _gnss_observation;

}

class SBPParser : public Parser {
public:
	SBPParser();

	double sven = 3;

	virtual MessageType get_message(MessagePtr& message_ptr);

	void buffer_to_sbp_process(char* buffer, u32 len);

	//u32 piksi_port_read(u8 *buff, u32 n, void *context);

	//static void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	//static void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	//static void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	double _gps_seconds_base = -1.0;

	double _gyro_scale = 0.0;

	double _accel_scale = 0.0;

	float _imu_measurement_span = 0.0;

	int _imu_frame_mapping = 5;

	double _imu_measurement_time_previous = -1.0;

	sbp_state_t s;

	size_t _total_length = 0;
};

Parser* Parser::create_sbp() {
	return new SBPParser();
}

/*
SBP Callbacks
*/
void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	//SBPParser* dis = static_cast<SBPParser*>(context);

	u32 flags = bytesToUnsignedInt(0, msg);

	ROS_WARN_STREAM("GPS Heartbeat (Antenna " << ((flags & 0x80000000) > 0 ? "present" : "not present") << ")");

	current_message = Parser::MessageType::NONE;
}

void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	//SBPParser* dis = static_cast<SBPParser*>(context);

	u16 week_number = bytesToUnsignedShort(0, msg); // GPS week number
	u32 time_of_week = bytesToUnsignedInt(2, msg); // Time of week in milliseconds
	s32 time_of_week_offset = bytesToSignedInt(6, msg); // Nanosecond residual of time of week

	setGPSWeek(week_number);

	double time = getGPSTime(time_of_week, time_of_week_offset);

	_gnss.set_measurement_time(time);
}

char hexstr[4];
char* int_to_hex( uint8_t i )
{
	sprintf(hexstr, "%02x", i);
	return hexstr;
}


void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	assert(len >= sizeof(msg_pos_llh_t));

	msg_pos_llh_t* data = (msg_pos_llh_t*) msg;
/*
	ROS_WARN_STREAM("lat: " << data->lat);
	ROS_WARN_STREAM("lon: " << data->lon);
	ROS_WARN_STREAM("height: " << data->height);

	ROS_WARN_STREAM("n_sats: " << data->n_sats);
	ROS_WARN_STREAM("flags: " << int_to_hex(data->flags));
*/

	//current_message = Parser::MessageType::GNSS;
}

void orient_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_orient_euler_t* data = (msg_orient_euler_t*) msg;

	ROS_WARN_STREAM("Roll: " << data->roll);
	ROS_WARN_STREAM("Pitch: " << data->pitch);
	ROS_WARN_STREAM("Yaw: " << data->yaw);
}

void angular_rate_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_angular_rate_t* data = (msg_angular_rate_t*) msg;

	ROS_WARN_STREAM("X: " << data->x);
	ROS_WARN_STREAM("Y: " << data->y);
	ROS_WARN_STREAM("Z: " << data->z);
}

void imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_imu_raw_t* data = (msg_imu_raw_t*) msg;

	ROS_WARN_STREAM("AccX: " << data->acc_x);
	ROS_WARN_STREAM("AccY: " << data->acc_y);
	ROS_WARN_STREAM("AccZ: " << data->acc_z);
	ROS_WARN_STREAM("GyroX: " << data->gyr_x);
	ROS_WARN_STREAM("GyroY: " << data->gyr_y);
	ROS_WARN_STREAM("GyroZ: " << data->gyr_z);
}
/*
End of callbacks
*/

SBPParser::SBPParser() {
	current_message = MessageType::NONE;

	_ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

	sbp_state_init(&s);
	sbp_state_set_io_context(&s, this);
	sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &heartbeat_callback_node);
	sbp_register_callback(&s, SBP_MSG_GPS_TIME, &gps_time_callback, NULL, &gps_time_callback_node);
	sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_llh_callback, NULL, &pos_llh_callback_node);
	sbp_register_callback(&s, SBP_MSG_ORIENT_EULER, &orient_euler_callback, NULL, &orient_euler_callback_node);
	sbp_register_callback(&s, SBP_MSG_ANGULAR_RATE, &angular_rate_callback, NULL, &angular_rate_callback_node);
	sbp_register_callback(&s, SBP_MSG_IMU_RAW, &imu_raw_callback, NULL, &imu_raw_callback_node);
}

u32 piksi_port_read(u8 *buff, u32 n, void *context){
	SBPParser* dis = static_cast<SBPParser*>(context);

	// SBP library calls this function
	// buff is the buffer in which it expects data when it's returned
	// n is the amount of bytes which the library requests
	// the function should return the amount of bytes read

	//ROS_WARN_STREAM("piksi_port_read " << n);

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
		//auto func = std::bind(&SBPParser::piksi_port_read, this);
		//sbp_process(&s, func);
		//sbp_process(&s, &piksi_port_read);

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

	//ROS_WARN_STREAM("heeej!");

	return current_message;
}

}
}
}