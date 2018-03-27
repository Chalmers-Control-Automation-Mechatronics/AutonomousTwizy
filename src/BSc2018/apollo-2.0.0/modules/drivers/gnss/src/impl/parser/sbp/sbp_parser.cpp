

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

#define GEOIDHEIGHT (35.885) // Geoid height for Chalmers Johanneberg, Gothenburg, Sweden

namespace apollo {
namespace drivers {
namespace gnss {

enum class SBPFixMode : u8 {
	INVALID = 0,
	SPP = 1,
	DIFF_GNSS = 2,
	FLOAT_RTK = 3,
	FIXED_RTK = 4,
	DEAD_RECKONING = 5
};

namespace {

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;
constexpr size_t BUFFER_SIZE = 1024;
constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t gps_time_callback_node;
static sbp_msg_callbacks_node_t utc_time_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
static sbp_msg_callbacks_node_t pos_llh_cov_callback_node;
static sbp_msg_callbacks_node_t orient_euler_callback_node;
static sbp_msg_callbacks_node_t angular_rate_callback_node;
static sbp_msg_callbacks_node_t imu_raw_callback_node;
static sbp_msg_callbacks_node_t vel_ned_callback_node;

// Converts a SBP Fixmode to a Novatel Position/Velocity type
u32 fix_mode_to_position_type(SBPFixMode fixmode){
	switch(fixmode){
		case SBPFixMode::SPP:
			return 16; // SINGLE
		case SBPFixMode::FLOAT_RTK:
			return 32; // L1_FLOAT (not sure if this is correct)
		case SBPFixMode::FIXED_RTK:
			return 48; // L1_INT (not sure if this is correct)
		case SBPFixMode::INVALID:
		default:
			return 0;
	}
}

// Extracts flags from a 32 bit integer
// If you have a flag enum between bit 3 and 5 on integer X for example, you can mask out
// the number you're interested in using this function.
// flag = extract_flags(data->flags, 3, 5);
u32 extract_flags(u32 flag_integer, u32 start, u32 stop){
	u32 a = (1 << start) - 1;
	u32 b = ((1 << stop) - 1) | (1 << stop);
	a = ~a;
	u32 mask = a & b;

	return (flag_integer & mask) >> start;
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

	virtual MessageType get_message(MessagePtr& message_ptr);

	void buffer_to_sbp_process(char* buffer, u32 len);

	sbp_state_t s;
};

Parser* Parser::create_sbp() {
	return new SBPParser();
}

/*
SBP Callbacks
*/
void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	assert(len >= sizeof(msg_heartbeat_t));
	msg_heartbeat_t* data = (msg_heartbeat_t*) msg;

	u8 antenna_present = extract_flags(data->flags, 31, 31);

	ROS_WARN_STREAM("GPS Heartbeat (Antenna " << (antenna_present > 0 ? "present" : "not present") << ")");
}

u32 lasttime = 0;
void utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	assert(len >= sizeof(msg_utc_time_t));
	msg_utc_time_t* data = (msg_utc_time_t*) msg;

	if(data->tow == lasttime)
		return;
	lasttime = data->tow;

	ROS_WARN_STREAM("UTC TOW: " << data->tow << " NS: " << data->ns);
}

double lat;
double lon;
double height;
void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	assert(len >= sizeof(msg_gps_time_t));
	msg_gps_time_t* data = (msg_gps_time_t*) msg;

	setGPSWeek(data->wn);

	//ROS_WARN_STREAM("TOW: " << data->tow << " TOWOff: " << data->ns_residual);

	double time = getGPSTime(data->tow + 0.1, data->ns_residual);

	// testing INS here
	if(_ins.measurement_time() == time)
		return;
	_ins.set_measurement_time(time);

	_ins.set_type(apollo::drivers::gnss::Ins::GOOD);

	_ins.mutable_position()->set_lat(lat);
	_ins.mutable_position()->set_lon(lon);
	_ins.mutable_position()->set_height(height);

	_ins.mutable_euler_angles()->set_x(0);
	_ins.mutable_euler_angles()->set_y(0);
	_ins.mutable_euler_angles()->set_z(0);

	for (int i = 0; i < 9; i += 4) {
		_ins.set_position_covariance(i, 1);
		_ins.set_euler_angles_covariance(i, 1);
		_ins.set_linear_velocity_covariance(i, 1);
	}

	current_message = Parser::MessageType::INS;


	//ROS_WARN_STREAM("GPS Time: " << time_of_week);

	//_gnss.set_measurement_time(time);
}

char hexstr[4];
char* int_to_hex(uint8_t i){
	sprintf(hexstr, "%02x", i);
	return hexstr;
}

bool pos_llh_gnss(msg_pos_llh_t* data){
	double gpstime = getGPSTime(data->tow);
	if(_gnss.measurement_time() == gpstime)
		return false;

	u8 ins_status = extract_flags(data->flags, 3, 4);
	if(ins_status == 1){
		ROS_WARN_STREAM("POS LLH used INS solution!");
	}

	//ROS_WARN_STREAM("POSLLH " << data->tow << " " << std::fixed << gpstime);

	_gnss.set_measurement_time(gpstime);
	_gnss.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

	lat = data->lat;
	lon = data->lon;
	height = data->height;

	_gnss.mutable_position()->set_lat(data->lat);
	_gnss.mutable_position()->set_lon(data->lon);
	_gnss.mutable_position()->set_height(data->height);
	_gnss.mutable_position_std_dev()->set_x(data->h_accuracy * 1e-3); // these accuracies may be in the wrong coordinate system
	_gnss.mutable_position_std_dev()->set_y(data->h_accuracy * 1e-3);
	_gnss.mutable_position_std_dev()->set_z(data->v_accuracy * 1e-3);

	_gnss.set_num_sats(data->n_sats);

	SBPFixMode fixmode = static_cast<SBPFixMode>(data->flags & 0x7);

	_gnss.set_solution_status(0); // Used by Novatel to tell INS status
	_gnss.set_position_type(fix_mode_to_position_type(fixmode)); // Used by Novatel to tell Solution Status

	switch(fixmode){
		case SBPFixMode::SPP:
			_gnss.set_type(apollo::drivers::gnss::Gnss::SINGLE);
			break;
		case SBPFixMode::DIFF_GNSS:
			ROS_WARN_STREAM("SBP uses DGNSS but apollo no support :(");
			break;
		case SBPFixMode::FLOAT_RTK:
			_gnss.set_type(apollo::drivers::gnss::Gnss::RTK_FLOAT);
			break;
		case SBPFixMode::FIXED_RTK:
			_gnss.set_type(apollo::drivers::gnss::Gnss::RTK_INTEGER);
			break;
		case SBPFixMode::INVALID:
		default:
			_gnss.set_type(apollo::drivers::gnss::Gnss::INVALID);
	}

	return true;
}

bool pos_llh_bestpos(msg_pos_llh_t* data){
	double gpstime = getGPSTime(data->tow);
	if(_bestpos.measurement_time() == gpstime)
		return false;

	_bestpos.set_measurement_time(getGPSTime(data->tow));
	_bestpos.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

	double wgs84_height = data->height;
	float undulation = GEOIDHEIGHT;
	double msl_height = wgs84_height - undulation;

	_bestpos.set_latitude(data->lat);
	_bestpos.set_longitude(data->lon);
	_bestpos.set_height_msl(msl_height);
	_bestpos.set_undulation(undulation);
	_bestpos.set_datum_id(apollo::drivers::gnss::DatumId::WGS84);
	_bestpos.set_latitude_std_dev(data->h_accuracy * 1e-3);
	_bestpos.set_longitude_std_dev(data->h_accuracy * 1e-3);
	_bestpos.set_height_std_dev(data->v_accuracy * 1e-3);
	_bestpos.set_num_sats_in_solution(data->n_sats);

	return true;
}

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	assert(len >= sizeof(msg_pos_llh_t));
	msg_pos_llh_t* data = (msg_pos_llh_t*) msg;

	if(pos_llh_gnss(data)){
		current_message = Parser::MessageType::GNSS;
		return;
	}

	if(pos_llh_bestpos(data)){
		current_message = Parser::MessageType::BEST_GNSS_POS;
		return;
	}
}

void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	assert(len >= sizeof(msg_vel_ned_t));
	msg_vel_ned_t* data = (msg_vel_ned_t*) msg;

	double gpstime = getGPSTime(data->tow);
	if(_gnss.measurement_time() == gpstime)
		return;

	u8 ins_status = extract_flags(data->flags, 3, 4);
	if(ins_status == 1){
		ROS_WARN_STREAM("VEL NED used INS solution!");
	}

	_gnss.set_measurement_time(getGPSTime(data->tow));
	_gnss.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

	_gnss.mutable_linear_velocity()->set_x(data->e * 1e-3);
	_gnss.mutable_linear_velocity()->set_y(data->n * 1e-3);
	_gnss.mutable_linear_velocity()->set_z(-data->d * 1e-3);

	_gnss.mutable_linear_velocity_std_dev()->set_x(data->h_accuracy * 1e-3); // these accuracies may be in the wrong coordinate system
	_gnss.mutable_linear_velocity_std_dev()->set_y(data->h_accuracy * 1e-3);
	_gnss.mutable_linear_velocity_std_dev()->set_z(data->v_accuracy * 1e-3);

	_gnss.set_num_sats(data->n_sats);

	current_message = Parser::MessageType::GNSS;
}

void orient_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	assert(len >= sizeof(msg_orient_euler_t));
	//msg_orient_euler_t* data = (msg_orient_euler_t*) msg;

	ROS_WARN_STREAM("It's sending INS data!!");
}

void angular_rate_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	assert(len >= sizeof(msg_angular_rate_t));
	//msg_angular_rate_t* data = (msg_angular_rate_t*) msg;

	ROS_WARN_STREAM("It's sending INS data!!");
}

void imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_imu_raw_t* data = (msg_imu_raw_t*) msg;

	//ROS_WARN_STREAM("IMU TOW: " << data->tow << " TOWFrac: " << data->tow_f);

	double gpstime = getGPSTime(data->tow);
	if(_imu.measurement_time() == gpstime)
		return;
	_imu.set_measurement_time(gpstime);
	_imu.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

	_imu.mutable_linear_acceleration()->set_x(-data->acc_x * 1e-3);
	_imu.mutable_linear_acceleration()->set_y(data->acc_y * 1e-3);
	_imu.mutable_linear_acceleration()->set_z(data->acc_z * 1e-3);

	_imu.mutable_angular_velocity()->set_x(-data->gyr_x * 1e-3);
	_imu.mutable_angular_velocity()->set_y(data->gyr_y * 1e-3);
	_imu.mutable_angular_velocity()->set_z(data->gyr_z * 1e-3);

	current_message = Parser::MessageType::IMU;
}
/*
End of callbacks
*/

void emptycallback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;
}

SBPParser::SBPParser() {
	current_message = MessageType::NONE;

	_ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
	_ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

	sbp_state_init(&s);
	sbp_state_set_io_context(&s, this);
	sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &heartbeat_callback_node);
	sbp_register_callback(&s, SBP_MSG_GPS_TIME, &gps_time_callback, NULL, &gps_time_callback_node);
	//sbp_register_callback(&s, SBP_MSG_UTC_TIME, &utc_time_callback, NULL, &utc_time_callback_node);
	sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_llh_callback, NULL, &pos_llh_callback_node);
	sbp_register_callback(&s, SBP_MSG_ORIENT_EULER, &orient_euler_callback, NULL, &orient_euler_callback_node);
	sbp_register_callback(&s, SBP_MSG_ANGULAR_RATE, &angular_rate_callback, NULL, &angular_rate_callback_node);
	sbp_register_callback(&s, SBP_MSG_IMU_RAW, &imu_raw_callback, NULL, &imu_raw_callback_node);
	sbp_register_callback(&s, SBP_MSG_VEL_NED, &vel_ned_callback, NULL, &vel_ned_callback_node);
}

u32 piksi_port_read(u8 *buff, u32 n, void *context){
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

	switch(current_message){
		case MessageType::GNSS:
			message_ptr = &_gnss;
			break;
		case MessageType::BEST_GNSS_POS:
			message_ptr = &_bestpos;
			break;
		case MessageType::IMU:
			message_ptr = &_imu;
			break;
		case MessageType::INS:
			message_ptr = &_ins;
			break;
		default:
			break;
	}

	return current_message;
}

}
}
}