

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

#include <kalman/time_update.h>
#include <kalman/messure_update_g.h>
#include <kalman/messure_update.h>
#include <kalman/q2e.h>
#include <kalman/q2e_initialize.h>
#include <kalman/q2e_terminate.h>
#include <kalman/ll2utm_c.h>
#include <kalman/utm2ll_c.h>
#include <kalman/Qq.h>
#include <kalman/P2EulerCov.h>
#include <kalman/P2EulerCov_initialize.h>
#include <kalman/P2EulerCov_terminate.h>

#include <kalman/ll2utm_c_initialize.h>
#include <kalman/ll2utm_initialize.h>
#include <kalman/messure_update_g_initialize.h>
#include <kalman/messure_update_initialize.h>
#include <kalman/rt_nonfinite.h>

#include <kalman/ll2utm_c_terminate.h>
#include <kalman/ll2utm_terminate.h>
#include <kalman/messure_update_g_terminate.h>
#include <kalman/messure_update_terminate.h>


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

enum class SBPSignalCode : u8 {
	GPS_L1CA = 0,
	GPS_L2CM = 1,
	SBAS_L1CA = 2,
	GLO_L1CA = 3,
	GLO_L2CA = 4,
	GPS_L1P = 5,
	GPS_L2P = 6
};

namespace {

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;
constexpr size_t BUFFER_SIZE = 1024;
constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t gps_time_callback_node;
//static sbp_msg_callbacks_node_t utc_time_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
//static sbp_msg_callbacks_node_t pos_llh_cov_callback_node;
static sbp_msg_callbacks_node_t orient_euler_callback_node;
static sbp_msg_callbacks_node_t angular_rate_callback_node;
static sbp_msg_callbacks_node_t imu_raw_callback_node;
static sbp_msg_callbacks_node_t vel_ned_callback_node;
static sbp_msg_callbacks_node_t obs_callback_node;
static sbp_msg_callbacks_node_t eph_gps_callback_node;
static sbp_msg_callbacks_node_t eph_glo_callback_node;

// Constants and globals for Kalman Filter
double xStateVector[8] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
double pCovVector[64];
const int UTM_ZONE = 32;
bool initPosSet = false;

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

// A helper that fills an Point3D object (which uses the FLU frame) using RFU
// measurements.
inline void rfu_to_flu(double r, double f, double u,
                       ::apollo::common::Point3D* flu) {
  flu->set_x(f);
  flu->set_y(-r);
  flu->set_z(u);
}

void signal_code_to_band_id(SBPSignalCode code,
							apollo::drivers::gnss::GnssBandID &bandid,
							apollo::drivers::gnss::GnssType &gnsstype,
							apollo::drivers::gnss::PseudoType &pseudotype){
	// Band ID
	switch(code){
		case SBPSignalCode::GPS_L1CA:
		case SBPSignalCode::GPS_L1P:
			bandid = apollo::drivers::gnss::GnssBandID::GPS_L1;
			break;
		case SBPSignalCode::GPS_L2CM:
		case SBPSignalCode::GPS_L2P:
			bandid = apollo::drivers::gnss::GnssBandID::GPS_L2;
			break;
		case SBPSignalCode::GLO_L1CA:
			bandid = apollo::drivers::gnss::GnssBandID::GLO_G1;
			break;
		case SBPSignalCode::GLO_L2CA:
			bandid = apollo::drivers::gnss::GnssBandID::GLO_G2;
			break;
		default:
			bandid = apollo::drivers::gnss::GnssBandID::BAND_UNKNOWN;
			break;
	}

	// Constellation
	switch(code){
		case SBPSignalCode::GPS_L1CA:
		case SBPSignalCode::GPS_L1P:
		case SBPSignalCode::GPS_L2CM:
		case SBPSignalCode::GPS_L2P:
			gnsstype = apollo::drivers::gnss::GnssType::GPS_SYS;
			break;
		case SBPSignalCode::GLO_L1CA:
		case SBPSignalCode::GLO_L2CA:
			gnsstype = apollo::drivers::gnss::GnssType::GLO_SYS;
			break;
		default:
			gnsstype = apollo::drivers::gnss::GnssType::SYS_UNKNOWN;
			break;
	}

	// Precision Type
	switch(code){
		case SBPSignalCode::GPS_L1CA:
		case SBPSignalCode::GPS_L2CM:
		case SBPSignalCode::GLO_L1CA:
		case SBPSignalCode::GLO_L2CA:
			pseudotype = apollo::drivers::gnss::PseudoType::CORSE_CODE;
			break;
		case SBPSignalCode::GPS_L1P:
		case SBPSignalCode::GPS_L2P:
			pseudotype = apollo::drivers::gnss::PseudoType::PRECISION_CODE;
			break;
		default:
			pseudotype = apollo::drivers::gnss::PseudoType::CODE_UNKNOWN;
			break;
	}

	return;
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

void init_kalman();
void update_kalman_gps(msg_pos_llh_t* data);

class SBPParser : public Parser {
public:
	SBPParser();

	virtual MessageType get_message(MessagePtr& message_ptr);

	void buffer_to_sbp_process(char* buffer, u32 len);

	sbp_state_t s;
};

Parser* Parser::create_sbp() {
	init_kalman();

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

	//ROS_WARN_STREAM("UTC TOW: " << data->tow << " NS: " << data->ns);
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

	update_kalman_gps(data);

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



void init_kalman(){
	for (int i = 0; i<64; i++){
		if(i%9 == 0){
			pCovVector[i] = 1.0;
		}
		else {
			pCovVector[i] = 0.0;
		}
	}


	xStateVector[0] = 0;
	xStateVector[1] = 0;
	xStateVector[2] = 0;
	xStateVector[3] = 0;
	xStateVector[4] = 1;
	xStateVector[5] = 0;
	xStateVector[6] = 0;
	xStateVector[7] = 0;

	ROS_WARN_STREAM("Init Kalman successful");
}

void update_kalman_gps(msg_pos_llh_t* data) {

	double lat = data->lat;
	double lon = data->lon;
	double height = data->height;


	double Rp[9] = {(data->h_accuracy/1000.0)*(data->h_accuracy/1000.0), 0.0, 0.0,
					0.0, (data->h_accuracy/1000.0)*(data->h_accuracy/1000.0), 0.0,
					0.0, 0.0, (data->v_accuracy/1000.0)*(data->v_accuracy/1000.0)};

	//ROS_WARN_STREAM("h_accuracy: " << Rp[0] << "      h_accuracy: " << Rp[8]);


	// Covert to UTM
	double x, y;
	ll2utm_c_initialize();
	ll2utm_c(lat, lon, UTM_ZONE, &x, &y);
	ll2utm_c_terminate();

	if(!initPosSet || xStateVector[3]>5 || xStateVector[3] < - 5)  {

		xStateVector[0] = x;
		xStateVector[1] = y;
		xStateVector[2] = height;
		xStateVector[3] = 0;

		initPosSet = true;

		ROS_WARN_STREAM("GNSS pos init: DONE");

		return;
	}

	double pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = height;

	messure_update_initialize();
	messure_update(xStateVector, pCovVector, pos, Rp);
	messure_update_terminate();

}

bool imu_imu_raw(msg_imu_raw_t* data){
	double gpstime = getGPSTime(data->tow);
	if((gpstime - _imu.measurement_time()) < 1e-3) // Don't accept any messages older than our current
		return false;
	//ROS_WARN_STREAM("RAW dt: " << gpstime-_imu.measurement_time());
	_imu.set_measurement_time(gpstime);
	_imu.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

	_imu.mutable_linear_acceleration()->set_x(-data->acc_x * 1e-3);
	_imu.mutable_linear_acceleration()->set_y(data->acc_y * 1e-3);
	_imu.mutable_linear_acceleration()->set_z(data->acc_z * 1e-3);

	_imu.mutable_angular_velocity()->set_x(-data->gyr_x * 1e-3);
	_imu.mutable_angular_velocity()->set_y(data->gyr_y * 1e-3);
	_imu.mutable_angular_velocity()->set_z(data->gyr_z * 1e-3);

	return true;
}

bool imu_ins(msg_imu_raw_t* data){
	//current_message = Parser::MessageType::INS;

	double gpstime = getGPSTime(data->tow);
	if((gpstime - _ins.measurement_time()) < 1e-3) // Don't accept any messages older than our current
		return false;

	double deltaT = gpstime - _ins.measurement_time();
	_ins.set_measurement_time(gpstime);
	_ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

	if(deltaT > 1e2) // DeltaT will be very big first frame due to there not being any proper time to compare with
		return false;

	// Do the Kalman dance

	// Switched x and y aacording to IMU orientation.
	double Ra[9] = {(1.0e-5)*0.0802, 0,0,
			0,(1.0e-5)*0.0654, 0,
			0, 0, (1.0e-5)*0.1236};

	double Rw[9] = {(1.0e-6)*0.4305, 0, 0,
			0, (1.0e-6)*0.3410, 0,
			0, 0, (1.0e-6)*0.3449};


	// Cordinate transformation accoriding to IMU orientation in car.
	// The IMU is mounteted with -x north, z down.
	// According to the model y is north, z up, x east.
	double gyr[3];
	gyr[0] = -1 * (double)data->gyr_y * RT_PI/180.0/32.8;
	gyr[1] = -1 * (double)data->gyr_x * RT_PI/180.0/32.8;
	gyr[2] = -1 * (double)data->gyr_z * RT_PI/180.0/32.8;

	double acc[3];
	acc[0] = -1 * (double)data->acc_y / 4096.0*9.82;
	acc[1] = -1 * (double)data->acc_x / 4096.0*9.82;
	acc[2] = -1 * (double)data->acc_z / 4096.0*9.82;

	messure_update_initialize();


	time_update(xStateVector, pCovVector, deltaT, gyr, acc, Ra, Rw);
	messure_update_g(xStateVector, pCovVector, acc, Ra);
	messure_update_terminate();


	// Calculate lon, lat from UTM-coords in xStatevector from EKF
	double latitude, longitude;
	utm2ll_c(xStateVector[0], xStateVector[1], UTM_ZONE, &latitude, &longitude);


	// Calculate eueler angles form quaternion in xStateVector from EKF.
	double euler[3];
	double quaternion[4];
	quaternion[0] = xStateVector[4];
	quaternion[1] = xStateVector[5];
	quaternion[2] = xStateVector[6];
	quaternion[3] = xStateVector[7];

	q2e_initialize();
	q2e(quaternion, euler);
	q2e_terminate();

	//ROS_WARN_STREAM("lat: " << latitude << " lon: " << longitude);
	//ROS_WARN_STREAM("x[0]: " << xStateVector[0] << " x[1]: " << xStateVector[0]);

	_ins.mutable_position()->set_lat(latitude);
	_ins.mutable_position()->set_lon(longitude);
	_ins.mutable_position()->set_height(xStateVector[2]);

	// Euler angles with intrinsic rotation sequence ZYX
	_ins.mutable_euler_angles()->set_z(euler[0]);
	_ins.mutable_euler_angles()->set_y(euler[1]);
	_ins.mutable_euler_angles()->set_x(euler[2]);


	double Qrotation[9];
	Qq(quaternion, Qrotation);

	double xVel = Qrotation[1]*xStateVector[3];
	double yVel = Qrotation[4]*xStateVector[3];
	double zVel = Qrotation[7]*xStateVector[3];

	_ins.mutable_linear_velocity()->set_x(xVel);
	_ins.mutable_linear_velocity()->set_y(yVel);
	_ins.mutable_linear_velocity()->set_z(zVel);

	rfu_to_flu(gyr[0], gyr[1], gyr[2], _ins.mutable_angular_velocity());

	double body_acc[3];

	// Remove gravity.
	body_acc[0] = acc[0] - Qrotation[2]*9.82;
	body_acc[1] = acc[1] - Qrotation[5]*9.82;
	body_acc[2] = acc[2] - Qrotation[8]*9.82;

	rfu_to_flu(body_acc[0], body_acc[1], body_acc[2], _ins.mutable_linear_acceleration());


	// Pos cov
	_ins.set_position_covariance(0, pCovVector[0]);
	_ins.set_position_covariance(1, pCovVector[1]);
	_ins.set_position_covariance(2, pCovVector[2]);
	_ins.set_position_covariance(3, pCovVector[8]);
	_ins.set_position_covariance(4, pCovVector[9]);
	_ins.set_position_covariance(5, pCovVector[10]);
	_ins.set_position_covariance(6, pCovVector[16]);
	_ins.set_position_covariance(7, pCovVector[17]);
	_ins.set_position_covariance(8, pCovVector[18]);

	// Euler angles cov
	double eulCov[9];
	P2EulerCov_initialize();
	P2EulerCov(pCovVector, quaternion, eulCov);
	P2EulerCov_terminate();
	//quatCov[0] = pCovVector[ ]

	for(int i = 0; i < 9; i++){
		_ins.set_euler_angles_covariance(i, eulCov[i]);
	}

	// TODO: Fix these covar matrixes
	for (int i = 0; i < 9; i += 4) {
		_ins.set_linear_velocity_covariance(i, pCovVector[27]);
	}



	// TODO: Fix INS-status

	if( (xStateVector[3] > 10) || (xStateVector[3] < -10 ) || (xStateVector[2]>120) || (xStateVector[2]<90)){
		_ins.set_type(apollo::drivers::gnss::Ins::INVALID);
	}
	else {
		_ins.set_type(apollo::drivers::gnss::Ins::GOOD);
	}

	//_ins.set_type(apollo::drivers::gnss::Ins::GOOD);


	ROS_WARN_STREAM("Euler: " << " Z:" << euler[0] << " Y:" << euler[1] << " X:" << euler[2]);
	ROS_WARN_STREAM("Pos: " << " lon: " << longitude << ", lat: " << latitude << ", height: " << xStateVector[2] << "vel: " << xStateVector[3]);


	return true;
}

void imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_imu_raw_t* data = (msg_imu_raw_t*) msg;

	///ROS_WARN_STREAM("IMU TOW: " << data->tow << " TOWFrac: " << data->tow_f);

	if(imu_imu_raw(data)){
		current_message = Parser::MessageType::IMU;
		return;
	}

	if(imu_ins(data)){
		current_message = Parser::MessageType::INS;
		return;
	}
}

void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	observation_header_t* header = (observation_header_t*) msg;

	sbp_gps_time_t* header_time = (sbp_gps_time_t*) &header->t;
	setGPSWeek(header_time->wn);
	double gpstime = getGPSTime(header_time->tow, header_time->ns_residual);

	if((gpstime - _gnss_observation.gnss_second_s()) < 1e-2)
		return;

	/*
	// vetifan hur den här fungerar
	u8 n_obs = header->n_obs;
	*/

	//räknar ut det manuellt istället
	u8 n_obs = (len - 11.0) / 17.0; // 11 bytes header, 17 bytes per obs

	_gnss_observation.Clear();
	_gnss_observation.set_receiver_id(0); // Rover

	_gnss_observation.set_gnss_time_type(apollo::drivers::gnss::GPS_TIME);
	_gnss_observation.set_gnss_week(header_time->wn);
	_gnss_observation.set_gnss_second_s(gpstime);

	_gnss_observation.set_sat_obs_num(n_obs);

	for(int i = 0; i < n_obs; i++){
		packed_obs_content_t* obs = (packed_obs_content_t*) &msg[17 * i + 11];

		apollo::drivers::gnss::GnssBandID band_id;
		apollo::drivers::gnss::GnssType gnss_type;
		apollo::drivers::gnss::PseudoType pseudo_type;

		signal_code_to_band_id(static_cast<SBPSignalCode>(obs->sid.code),
			band_id,
			gnss_type,
			pseudo_type);

		auto sat_obs = _gnss_observation.add_sat_obs();
		sat_obs->set_sat_prn(obs->sid.sat);
		sat_obs->set_sat_sys(gnss_type);

		sat_obs->set_band_obs_num(1);
		auto band_obs = sat_obs->add_band_obs();
		band_obs->set_pseudo_type(pseudo_type);
		band_obs->set_band_id(band_id);
		band_obs->set_frequency_value(0);
		band_obs->set_pseudo_range(obs->P * 2e-2);
		band_obs->set_carrier_phase(obs->L.i + obs->L.f / 256.0);
		band_obs->set_doppler(obs->D.i + obs->D.f / 256.0);
		band_obs->set_loss_lock_index(obs->lock);
		band_obs->set_snr(obs->cn0 / 4.0);
	}

	current_message = Parser::MessageType::OBSERVATION;
}

void eph_gps_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_ephemeris_gps_t* data = (msg_ephemeris_gps_t*) msg;
	ephemeris_common_content_t* common = (ephemeris_common_content_t*) &data->common;

	_gnss_ephemeris.set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);

	apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
		_gnss_ephemeris.mutable_keppler_orbit();

	keppler_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);
	keppler_orbit->set_gnss_time_type(
		apollo::drivers::gnss::GnssTimeType::GPS_TIME);

	double toetime = getGPSTime(common->toe.tow);
	if((toetime - keppler_orbit->toe()) < 1e-2)
		return;

	setGPSWeek(common->toe.wn);
	keppler_orbit->set_toe(toetime);

	setGPSWeek(data->toc.wn);
	keppler_orbit->set_toc(getGPSTime(data->toc.tow));

	keppler_orbit->set_sat_prn(common->sid.sat);
	keppler_orbit->set_week_num(data->toc.wn);
	keppler_orbit->set_af0(data->af0);
	keppler_orbit->set_af1(data->af1);
	keppler_orbit->set_af2(data->af2);
	keppler_orbit->set_iode(data->iode);
	keppler_orbit->set_deltan(data->dn);
	keppler_orbit->set_m0(data->m0);
	keppler_orbit->set_e(data->ecc);
	keppler_orbit->set_roota(data->sqrta);
	keppler_orbit->set_cic(data->c_ic);
	keppler_orbit->set_cis(data->c_is);
	keppler_orbit->set_crc(data->c_rc);
	keppler_orbit->set_crs(data->c_rs);
	keppler_orbit->set_cuc(data->c_uc);
	keppler_orbit->set_cus(data->c_us);
	keppler_orbit->set_omega0(data->omega0);
	keppler_orbit->set_omega(data->w);
	keppler_orbit->set_omegadot(data->omegadot);
	keppler_orbit->set_i0(data->inc);
	keppler_orbit->set_idot(data->inc_dot);
	keppler_orbit->set_accuracy(common->ura);
	keppler_orbit->set_health(common->health_bits);
	keppler_orbit->set_tgd(data->tgd);
	keppler_orbit->set_iodc(data->iodc);

	current_message = Parser::MessageType::GPSEPHEMERIDES;
}

void eph_glo_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_ephemeris_glo_t* data = (msg_ephemeris_glo_t*) msg;
	ephemeris_common_content_t* common = (ephemeris_common_content_t*) &data->common;

	_gnss_ephemeris.set_gnss_type(apollo::drivers::gnss::GnssType::GLO_SYS);

	apollo::drivers::gnss::GlonassOrbit* glonass_orbit =
		_gnss_ephemeris.mutable_glonass_orbit();

	glonass_orbit->set_gnss_time_type(
		apollo::drivers::gnss::GnssTimeType::GLO_TIME);

	glonass_orbit->set_slot_prn(common->sid.sat);

	u32 toetow = common->toe.tow;
	if((toetow - glonass_orbit->week_second_s()) < 1e-2)
		return;

	setGPSWeek(common->toe.wn);
	glonass_orbit->set_toe(getGPSTime(common->toe.tow));

	glonass_orbit->set_frequency_no(data->fcn);
	glonass_orbit->set_week_num(common->toe.wn); // this should be TOC but SBP doesn't supply
	glonass_orbit->set_week_second_s(common->toe.tow); // this should be TOC but SBP doesn't supply
	//glonass_orbit->set_tk(data->Tk); // UH OH
	glonass_orbit->set_clock_offset(-data->tau);
	glonass_orbit->set_clock_drift(data->gamma);

	glonass_orbit->set_health(common->health_bits);
	glonass_orbit->set_position_x(data->pos[0]);
	glonass_orbit->set_position_y(data->pos[1]);
	glonass_orbit->set_position_z(data->pos[2]);

	glonass_orbit->set_velocity_x(data->vel[0]);
	glonass_orbit->set_velocity_y(data->vel[1]);
	glonass_orbit->set_velocity_z(data->vel[2]);

	glonass_orbit->set_accelerate_x(data->acc[0]);
	glonass_orbit->set_accelerate_y(data->acc[1]);
	glonass_orbit->set_accelerate_z(data->acc[2]);

	//glonass_orbit->set_infor_age(data->age); // UH OH

	current_message = Parser::MessageType::GLOEPHEMERIDES;
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
	sbp_register_callback(&s, SBP_MSG_OBS, &obs_callback, NULL, &obs_callback_node);
	sbp_register_callback(&s, SBP_MSG_EPHEMERIS_GPS, &eph_gps_callback, NULL, &eph_gps_callback_node);
	sbp_register_callback(&s, SBP_MSG_EPHEMERIS_GLO, &eph_glo_callback, NULL, &eph_glo_callback_node);
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
		case MessageType::OBSERVATION:
			message_ptr = &_gnss_observation;
			break;
		case MessageType::GPSEPHEMERIDES:
		case MessageType::GLOEPHEMERIDES:
			message_ptr = &_gnss_ephemeris;
			break;
		default:
			break;
	}

	return current_message;
}

}
}
}
