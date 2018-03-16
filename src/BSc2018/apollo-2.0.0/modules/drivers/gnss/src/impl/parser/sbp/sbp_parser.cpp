
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/piksi.h>

class SBPParser : public Parser {
public:
    SBPParser();

    virtual MessageType get_message(MessagePtr& message_ptr);

private:
    bool verify_checksum();

    Parser::MessageType prepare_message(MessagePtr& message_ptr);

    // The handle_xxx functions return whether a message is ready.
    bool handle_esf_raw(const ublox::EsfRaw* raw, size_t data_size);
    bool handle_esf_ins(const ublox::EsfIns* ins);
    bool handle_hnr_pvt(const ublox::HnrPvt* pvt);
    bool handle_nav_att(const ublox::NavAtt *att);
    bool handle_nav_pvt(const ublox::NavPvt* pvt);
    bool handle_nav_cov(const ublox::NavCov *cov);
    bool handle_rxm_rawx(const ublox::RxmRawx *raw);

    double _gps_seconds_base = -1.0;

    double _gyro_scale = 0.0;

    double _accel_scale = 0.0;

    float _imu_measurement_span = 0.0;

    int _imu_frame_mapping = 5;

    double _imu_measurement_time_previous = -1.0;

    std::vector<uint8_t> _buffer;

    size_t _total_length = 0;

    ::apollo::drivers::gnss::Gnss _gnss;
    ::apollo::drivers::gnss::Imu _imu;
    ::apollo::drivers::gnss::Ins _ins;
};

Parser* Parser::create_sbp() {
  return new SBPParser();
}

SBPParser::SBPParser() {
  _buffer.reserve(BUFFER_SIZE);
  _ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  _ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  _ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

  ROS_WARN_STREAM("TJENA");
}
