// cmdvel_to_blvr.cpp (low-latency, thread-safe Modbus, IMU RELIABLE QoS optional)
#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include <modbus/modbus.h>
#ifdef MODBUS_RTU_RS485
  #include <modbus/modbus-rtu.h> // optional, for RS485 mode / RTS control
#endif

using std::placeholders::_1;
using namespace std::chrono_literals;

// ================= Helpers =================
static inline std::array<uint16_t, 2> i32_to_regs_be(int32_t v){
    uint32_t u = static_cast<uint32_t>(v);
    return {static_cast<uint16_t>((u >> 16) & 0xFFFF),
            static_cast<uint16_t>(u & 0xFFFF)};
}
static inline int32_t regs_be_to_i32(uint16_t hi, uint16_t lo){
    uint32_t u = (static_cast<uint32_t>(hi) << 16) | static_cast<uint32_t>(lo);
    return static_cast<int32_t>(u);
}
static inline geometry_msgs::msg::Quaternion yaw_to_quat(double yaw){
    geometry_msgs::msg::Quaternion q;
    double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    q.x = 0.0; q.y = 0.0; q.z = sy; q.w = cy;
    return q;
}

// ================= Thread-safe Modbus wrapper =================
class Mvc01Modbus {
public:
    Mvc01Modbus(const std::string &port, int baud, char parity, int stopbits, int slave,
                double resp_timeout_s, double byte_timeout_s,
                bool rs485_mode, bool rts_up)
        : port_(port), baud_(baud), stopbits_(stopbits), slave_(slave),
          parity_(parity), resp_timeout_s_(resp_timeout_s), byte_timeout_s_(byte_timeout_s),
          rs485_mode_(rs485_mode), rts_up_(rts_up) {}

    ~Mvc01Modbus(){ close(); }

    void connect(){
        std::lock_guard<std::mutex> lk(mx_);
        connect_unlocked();
    }

    void close(){
        std::lock_guard<std::mutex> lk(mx_);
        if (ctx_) { modbus_close(ctx_); modbus_free(ctx_); ctx_ = nullptr; }
    }

    bool ok() const {
        std::lock_guard<std::mutex> lk(mx_);
        return ctx_ != nullptr;
    }

    // Do fn() with exclusive access; on error, reconnect once and retry.
    template <typename Fn>
    auto with_retry(Fn &&fn){
        std::lock_guard<std::mutex> lk(mx_);
        try {
            return fn(ctx_);
        } catch (...) {
            connect_unlocked();
            return fn(ctx_);
        }
    }

    // Typed ops (auto-locked)
    void write_u32(int addr, int32_t val){
        with_retry([&](modbus_t* c){
            auto regs = i32_to_regs_be(val);
            int rc = modbus_write_registers(c, addr, 2, regs.data());
            if (rc != 2) throw std::runtime_error("modbus_write_registers failed");
            return 0;
        });
    }

    void write_regs(int addr, const uint16_t* regs, int count){
        with_retry([&](modbus_t* c){
            int rc = modbus_write_registers(c, addr, count, const_cast<uint16_t*>(regs));
            if (rc != count) throw std::runtime_error("modbus_write_registers block failed");
            return 0;
        });
    }

    int32_t read_i32(int addr){
        return with_retry([&](modbus_t* c){
            uint16_t regs[2] = {0,0};
            int rc = modbus_read_registers(c, addr, 2, regs);
            if (rc != 2) throw std::runtime_error("modbus_read_registers failed");
            return regs_be_to_i32(regs[0], regs[1]);
        });
    }

    void read_block(int addr, int count, uint16_t *out){
        with_retry([&](modbus_t* c){
            int rc = modbus_read_registers(c, addr, count, out);
            if (rc != count) throw std::runtime_error("modbus_read_registers block failed");
            return 0;
        });
    }

private:
    void connect_unlocked(){
        if (ctx_) { modbus_close(ctx_); modbus_free(ctx_); ctx_ = nullptr; }
        ctx_ = modbus_new_rtu(port_.c_str(), baud_, parity_, 8, stopbits_);
        if (!ctx_) throw std::runtime_error("modbus_new_rtu failed");
#ifdef MODBUS_RTU_RS485
        if (rs485_mode_) {
            if (modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485) == -1) {
                // warn only; some drivers don't support
            }
            modbus_rtu_set_rts(ctx_, rts_up_ ? MODBUS_RTU_RTS_UP : MODBUS_RTU_RTS_DOWN);
        }
#endif
        if (modbus_set_slave(ctx_, slave_) == -1) throw std::runtime_error("modbus_set_slave failed");

        auto tv = [](double s){
            uint32_t us = static_cast<uint32_t>(s * 1e6);
            return timeval{ static_cast<time_t>(us/1000000), static_cast<suseconds_t>(us%1000000) };
        };
        auto tv_resp = tv(resp_timeout_s_);
        auto tv_byte = tv(byte_timeout_s_);
        modbus_set_response_timeout(ctx_, tv_resp.tv_sec, tv_resp.tv_usec);
        modbus_set_byte_timeout(ctx_, tv_byte.tv_sec, tv_byte.tv_usec);

        if (modbus_connect(ctx_) == -1) {
            modbus_free(ctx_); ctx_ = nullptr;
            throw std::runtime_error("modbus_connect failed");
        }
    }

    // config
    std::string port_;
    int baud_, stopbits_, slave_;
    char parity_;
    double resp_timeout_s_, byte_timeout_s_;
    bool rs485_mode_, rts_up_;

    // state
    modbus_t *ctx_{nullptr};
    mutable std::mutex mx_;
};

// ================= Node =================
class CmdVelToBLVR : public rclcpp::Node {
public:
    CmdVelToBLVR() : Node("cmd_vel_to_blvr")
    {
        // ===== Params =====
        port_        = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_    = this->declare_parameter<int>("baudrate", 230400);
        parity_s_    = this->declare_parameter<std::string>("parity", "E");
        stopbits_    = this->declare_parameter<int>("stopbits", 1);
        slave_id_    = this->declare_parameter<int>("slave_id", 1);

        // low-latency timeouts
        resp_timeout_s_ = this->declare_parameter<double>("response_timeout_s", 0.040); // 40 ms
        byte_timeout_s_ = this->declare_parameter<double>("byte_timeout_s",     0.002); // 2 ms

        rs485_mode_  = this->declare_parameter<bool>("rs485_mode", false);
        rts_up_      = this->declare_parameter<bool>("rts_up", true);

        cmd_topic_   = this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
        estop_topic_ = this->declare_parameter<std::string>("estop_topic", "/estop");
        cmd_rate_hz_ = std::max(5.0, this->declare_parameter<double>("cmd_rate_hz", 100.0));
        poll_rate_hz_= std::max(1.0, this->declare_parameter<double>("poll_rate_hz", 30.0));
        cmd_timeout_s_= this->declare_parameter<double>("cmd_timeout_s", 0.3);

        // scaling & limits
        lin_scale_   = this->declare_parameter<double>("linear_scale", 1.0);
        ang_scale_   = this->declare_parameter<double>("angular_scale", 1.0);
        vx_limit_    = this->declare_parameter<double>("vx_limit", 0.5);
        w_limit_     = this->declare_parameter<double>("w_limit", 1.68);

        // frames
        odom_frame_  = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_  = this->declare_parameter<std::string>("base_frame", "base_link");
        imu_frame_   = this->declare_parameter<std::string>("imu_frame", "imu_link");

        // IMU QoS parameter (default RELIABLE untuk hilangkan warning QoS mismatch)
        imu_reliable_qos_ = this->declare_parameter<bool>("imu_reliable_qos", true);

        // Optional trigger usage
        use_trigger_ = this->declare_parameter<bool>("use_trigger", true);

        // Monitor register params
        addr_disp_fb_ = this->declare_parameter<int>("addr_disp_fb", 2144);
        addr_disp_lr_ = this->declare_parameter<int>("addr_disp_lr", 2146);
        addr_wodo_x_  = this->declare_parameter<int>("addr_wodo_x", 2138);
        addr_wodo_y_  = this->declare_parameter<int>("addr_wodo_y", 2140);
        addr_wodo_th_ = this->declare_parameter<int>("addr_wodo_th", 2142);
        addr_godo_x_  = this->declare_parameter<int>("addr_godo_x", 2064);
        addr_godo_y_  = this->declare_parameter<int>("addr_godo_y", 2066);
        addr_godo_yaw_= this->declare_parameter<int>("addr_godo_yaw", 2070);
        addr_imu_ax_  = this->declare_parameter<int>("addr_imu_ax", 2084);
        addr_imu_ay_  = this->declare_parameter<int>("addr_imu_ay", 2086);
        addr_imu_az_  = this->declare_parameter<int>("addr_imu_az", 2088);
        addr_imu_gx_  = this->declare_parameter<int>("addr_imu_gx", 2090);
        addr_imu_gy_  = this->declare_parameter<int>("addr_imu_gy", 2092);
        addr_imu_gz_  = this->declare_parameter<int>("addr_imu_gz", 2094);
        addr_imu_tmp_ = this->declare_parameter<int>("addr_imu_tmp", 2096);

        addr_cmd_vx_  = this->declare_parameter<int>("addr_cmd_vx", 2100);
        addr_cmd_vlr_ = this->declare_parameter<int>("addr_cmd_vlr", 2102);
        addr_cmd_w_   = this->declare_parameter<int>("addr_cmd_w", 2104);
        addr_fbk_vx_  = this->declare_parameter<int>("addr_fbk_vx", 2106);
        addr_fbk_vlr_ = this->declare_parameter<int>("addr_fbk_vlr", 2108);
        addr_fbk_w_   = this->declare_parameter<int>("addr_fbk_w", 2110);

        addr_ctl_tmp_ = this->declare_parameter<int>("addr_ctl_tmp", -1);
        addr_supply_v_= this->declare_parameter<int>("addr_supply_v", -1);
        addr_pwr_cnt_ = this->declare_parameter<int>("addr_power_on", -1);
        addr_boot_ms_ = this->declare_parameter<int>("addr_boot_ms", -1);
        addr_torque1_ = this->declare_parameter<int>("addr_torque1", -1);
        addr_torque2_ = this->declare_parameter<int>("addr_torque2", -1);
        addr_torque3_ = this->declare_parameter<int>("addr_torque3", -1);
        addr_torque4_ = this->declare_parameter<int>("addr_torque4", -1);
        torque_scale_ = this->declare_parameter<double>("torque_scale", 0.001);

        // ===== Modbus =====
        mb_ = std::make_unique<Mvc01Modbus>(port_, baudrate_, parity_s_.empty() ? 'E' : parity_s_[0],
                                            stopbits_, slave_id_, resp_timeout_s_, byte_timeout_s_,
                                            rs485_mode_, rts_up_);
        mb_->connect();
        RCLCPP_INFO(get_logger(), "Modbus open on %s", port_.c_str());

        // Set mode Vx-ω (toleran gagal awal)
        try { mb_->write_u32(1986, 1); } // ADDR_MODE
        catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "MODE write failed on startup: %s (will continue)", e.what());
        }

        // ===== ROS IO =====
        auto sensor_qos = rclcpp::SensorDataQoS();

        // Callback groups
        cb_cmd_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_poll_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Subscribers
        {
            rclcpp::SubscriptionOptions so;
            so.callback_group = cb_cmd_;
            cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                cmd_topic_, rclcpp::QoS(1).best_effort(),
                std::bind(&CmdVelToBLVR::cmd_cb, this, _1), so);
        }
        {
            rclcpp::SubscriptionOptions so;
            so.callback_group = cb_cmd_;
            estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                estop_topic_, rclcpp::QoS(10).best_effort(),
                std::bind(&CmdVelToBLVR::estop_cb, this, _1), so);
        }

        // Publishers
        odom_wheel_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", sensor_qos);
        odom_gyro_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom_gyro", sensor_qos);

        // IMU publisher with selectable reliability (default RELIABLE)
        auto imu_qos = rclcpp::SensorDataQoS();
        if (imu_reliable_qos_) {
            imu_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        }
        imu_pub_      = this->create_publisher<sensor_msgs::msg::Imu>("/imu", imu_qos);

        imu_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu_temperature", sensor_qos);
        ctl_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/controller_temperature", sensor_qos);
        supply_pub_   = this->create_publisher<std_msgs::msg::Float32>("/supply_voltage", 10);
        power_on_pub_ = this->create_publisher<std_msgs::msg::Int32>("/power_on_count", 10);
        uptime_pub_   = this->create_publisher<std_msgs::msg::Float32>("/uptime_s", 10);
        disp_pub_     = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/displacement", sensor_qos);
        fbk_speed_pub_= this->create_publisher<geometry_msgs::msg::TwistStamped>("/feedback_speed", 10);
        torque_pub_   = this->create_publisher<std_msgs::msg::Float32MultiArray>("/torque_pct", 10);

        // Timers (attach callback group; API kompatibel)
        cmd_timer_  = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / cmd_rate_hz_)),
            std::bind(&CmdVelToBLVR::cmd_loop, this),
            cb_cmd_);

        poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / poll_rate_hz_)),
            std::bind(&CmdVelToBLVR::poll_loop, this),
            cb_poll_);

        last_cmd_time_ = now();
        RCLCPP_INFO(get_logger(), "cmd_vel_to_blvr ready: listening on %s", cmd_topic_.c_str());
    }

    ~CmdVelToBLVR() override {
        safe_stop();
        mb_.reset();
    }

private:
    // Registers
    static constexpr int ADDR_MODE = 1986;
    static constexpr int ADDR_VX   = 1988; // 0.001 m/s
    static constexpr int ADDR_W    = 1990; // 1e-6 rad/s
    static constexpr int ADDR_TRG  = 2030;

    // /cmd_vel callback
    void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
        std::lock_guard<std::mutex> lk(m_);
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        have_cmd_ = true;
    }

    // /estop callback
    void estop_cb(const std_msgs::msg::Bool::SharedPtr msg){
        std::lock_guard<std::mutex> lk(m_);
        estop_latched_ = msg->data;
    }

    // Combined write: Vx + W in one frame
    void write_cmd_vx_w(int32_t vx_units, int32_t w_units){
        uint16_t regs[4];
        auto vx = i32_to_regs_be(vx_units);
        auto w  = i32_to_regs_be(w_units);
        regs[0] = vx[0]; regs[1] = vx[1];
        regs[2] = w[0];  regs[3] = w[1];
        mb_->write_regs(ADDR_VX, regs, 4);
    }

    // Command loop
    void cmd_loop(){
        try{
            double vx_cmd = 0.0, w_cmd = 0.0;
            {
                std::lock_guard<std::mutex> lk(m_);
                bool stale = (now() - last_cmd_time_).seconds() > cmd_timeout_s_;
                if (have_cmd_ && !stale && !estop_latched_){
                    vx_cmd = std::clamp(last_cmd_.linear.x * lin_scale_, -vx_limit_, vx_limit_);
                    w_cmd  = std::clamp(last_cmd_.angular.z * ang_scale_, -w_limit_, w_limit_);
                }else{
                    vx_cmd = 0.0; w_cmd = 0.0;
                }
            }
            int32_t vx_units = static_cast<int32_t>(std::llround(vx_cmd / 0.001));
            int32_t w_units  = static_cast<int32_t>(std::llround(w_cmd  / 1e-6));

            mb_->with_retry([&](modbus_t*){ write_cmd_vx_w(vx_units, w_units); return 0; });

            if (use_trigger_) {
                mb_->write_u32(ADDR_TRG, 1);
            }

        }catch(const std::exception &e){
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cmd error: %s", e.what());
            try{ safe_stop(); }catch(...){}
        }
    }

    // Scales
    static inline double regs_to_m(uint16_t hi, uint16_t lo){ return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-3; }
    static inline double regs_to_u6rad(uint16_t hi, uint16_t lo){ return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-6; }
    static inline double regs_to_acc(uint16_t hi, uint16_t lo){ return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-3; } // m/s^2
    static inline double regs_to_rps(uint16_t hi, uint16_t lo){ return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-6; } // rad/s
    static inline double regs_to_mps(uint16_t hi, uint16_t lo){ return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-3; } // m/s
    static inline double regs_to_c(uint16_t hi, uint16_t lo){ return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-3; }   // degC

    double read_scaled(int addr, double s){
        int32_t v = mb_->read_i32(addr);
        return static_cast<double>(v) * s;
    }

    // Round-robin polling
    void poll_loop(){
        try{
            auto stamp = this->now();
            switch ((phase_++) % 3) {
                case 0: { // Wheel odom + displacement
                    uint16_t r1[6];
                    mb_->read_block(addr_wodo_x_, 6, r1);
                    double x = regs_to_m(r1[0], r1[1]);
                    double y = regs_to_m(r1[2], r1[3]);
                    double th= regs_to_u6rad(r1[4], r1[5]);
                    nav_msgs::msg::Odometry ow;
                    ow.header.stamp = stamp; ow.header.frame_id = odom_frame_; ow.child_frame_id = base_frame_;
                    ow.pose.pose.position.x = x; ow.pose.pose.position.y = y; ow.pose.pose.orientation = yaw_to_quat(th);
                    odom_wheel_pub_->publish(ow);

                    uint16_t r2[4];
                    mb_->read_block(addr_disp_fb_, 4, r2);
                    geometry_msgs::msg::Vector3Stamped v;
                    v.header.stamp = stamp; v.header.frame_id = base_frame_;
                    v.vector.x = regs_to_m(r2[0], r2[1]);
                    v.vector.y = regs_to_m(r2[2], r2[3]);
                    v.vector.z = 0.0;
                    disp_pub_->publish(v);
                } break;

                case 1: { // IMU block
                    uint16_t r[14];
                    mb_->read_block(addr_imu_ax_, 14, r);
                    sensor_msgs::msg::Imu imu;
                    imu.header.stamp = stamp; imu.header.frame_id = imu_frame_;
                    imu.linear_acceleration.x = regs_to_acc(r[0], r[1]);
                    imu.linear_acceleration.y = regs_to_acc(r[2], r[3]);
                    imu.linear_acceleration.z = regs_to_acc(r[4], r[5]);
                    imu.angular_velocity.x = regs_to_rps(r[6], r[7]);
                    imu.angular_velocity.y = regs_to_rps(r[8], r[9]);
                    imu.angular_velocity.z = regs_to_rps(r[10], r[11]);
                    imu.orientation_covariance[0] = -1.0;
                    imu_pub_->publish(imu);

                    sensor_msgs::msg::Temperature t;
                    t.header.stamp = stamp; t.header.frame_id = imu_frame_;
                    t.temperature = regs_to_c(r[12], r[13]);
                    imu_temp_pub_->publish(t);
                } break;

                case 2: { // Gyro odom + feedback + optionals
                    uint16_t r1[8];
                    mb_->read_block(addr_godo_x_, 8, r1);
                    double x = regs_to_m(r1[0], r1[1]);
                    double y = regs_to_m(r1[2], r1[3]);
                    double yaw = regs_to_u6rad(r1[6], r1[7]);
                    nav_msgs::msg::Odometry og;
                    og.header.stamp = stamp; og.header.frame_id = odom_frame_; og.child_frame_id = base_frame_;
                    og.pose.pose.position.x = x; og.pose.pose.position.y = y; og.pose.pose.orientation = yaw_to_quat(yaw);
                    odom_gyro_pub_->publish(og);

                    uint16_t fb[6];
                    mb_->read_block(2492, 6, fb);  // sesuaikan jika alamat berbeda
                    double vx_fb = regs_to_mps(fb[0], fb[1]);
                    double wz_fb = regs_to_rps(fb[2], fb[3]);
                    double vy_fb = regs_to_mps(fb[4], fb[5]);
                    geometry_msgs::msg::TwistStamped tw;
                    tw.header.stamp = stamp;
                    tw.twist.linear.x = vx_fb;
                    tw.twist.linear.y = vy_fb;
                    tw.twist.angular.z = wz_fb;
                    fbk_speed_pub_->publish(tw);

                    if (addr_ctl_tmp_ > 0){
                        sensor_msgs::msg::Temperature t;
                        t.header.stamp = stamp; t.header.frame_id = "controller";
                        t.temperature = read_scaled(addr_ctl_tmp_, 1e-3);
                        ctl_temp_pub_->publish(t);
                    }
                    if (addr_supply_v_ > 0){
                        std_msgs::msg::Float32 v;
                        v.data = read_scaled(addr_supply_v_, 1e-3);
                        supply_pub_->publish(v);
                    }
                    if (addr_pwr_cnt_ > 0){
                        std_msgs::msg::Int32 c;
                        c.data = mb_->read_i32(addr_pwr_cnt_);
                        power_on_pub_->publish(c);
                    }
                    if (addr_boot_ms_ > 0){
                        std_msgs::msg::Float32 s;
                        s.data = static_cast<float>(read_scaled(addr_boot_ms_, 1e-3));
                        uptime_pub_->publish(s);
                    }
                    if (addr_torque1_ > 0 || addr_torque2_ > 0 || addr_torque3_ > 0 || addr_torque4_ > 0){
                        std_msgs::msg::Float32MultiArray arr;
                        if (addr_torque1_ > 0) arr.data.push_back(static_cast<float>(read_scaled(addr_torque1_, torque_scale_)));
                        if (addr_torque2_ > 0) arr.data.push_back(static_cast<float>(read_scaled(addr_torque2_, torque_scale_)));
                        if (addr_torque3_ > 0) arr.data.push_back(static_cast<float>(read_scaled(addr_torque3_, torque_scale_)));
                        if (addr_torque4_ > 0) arr.data.push_back(static_cast<float>(read_scaled(addr_torque4_, torque_scale_)));
                        torque_pub_->publish(arr);
                    }
                } break;
            }
        }catch(const std::exception &e){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "poll error: %s (auto-retry)", e.what());
            // mb_->with_retry akan reconnect saat operasi berikutnya
        }
    }

    void safe_stop(){
        if (!mb_ || !mb_->ok()) return;
        try{ mb_->write_u32(ADDR_VX, 0); }catch(...){}
        try{ mb_->write_u32(ADDR_W,  0); }catch(...){}
        try{ mb_->write_u32(ADDR_TRG,1); }catch(...){}
    }

    // State
    std::mutex m_;
    geometry_msgs::msg::Twist last_cmd_;
    rclcpp::Time last_cmd_time_;
    bool have_cmd_{false};
    bool estop_latched_{false};

    // Modbus
    std::unique_ptr<Mvc01Modbus> mb_;

    // Params
    std::string port_, parity_s_;
    int baudrate_{230400}, stopbits_{1}, slave_id_{1};
    double resp_timeout_s_{0.040}, byte_timeout_s_{0.002};
    bool rs485_mode_{false}, rts_up_{true};
    double cmd_rate_hz_{100.0}, poll_rate_hz_{30.0}, cmd_timeout_s_{0.3};
    double lin_scale_{1.0}, ang_scale_{1.0}, vx_limit_{0.5}, w_limit_{1.68};
    std::string cmd_topic_{"/cmd_vel"}, estop_topic_{"/estop"};
    std::string odom_frame_{"odom"}, base_frame_{"base_link"}, imu_frame_{"imu_link"};
    bool imu_reliable_qos_{true};
    bool use_trigger_{true};

    // Monitor addresses
    int addr_disp_fb_, addr_disp_lr_;
    int addr_wodo_x_, addr_wodo_y_, addr_wodo_th_;
    int addr_godo_x_, addr_godo_y_, addr_godo_yaw_;
    int addr_imu_ax_, addr_imu_ay_, addr_imu_az_, addr_imu_gx_, addr_imu_gy_, addr_imu_gz_, addr_imu_tmp_;
    int addr_cmd_vx_, addr_cmd_vlr_, addr_cmd_w_, addr_fbk_vx_, addr_fbk_vlr_, addr_fbk_w_;
    int addr_ctl_tmp_, addr_supply_v_, addr_pwr_cnt_, addr_boot_ms_;
    int addr_torque1_, addr_torque2_, addr_torque3_, addr_torque4_;
    double torque_scale_;
    int phase_{0};

    // ROS IO
    rclcpp::CallbackGroup::SharedPtr cb_cmd_, cb_poll_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_wheel_pub_, odom_gyro_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp_pub_, ctl_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr supply_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr power_on_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr uptime_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr disp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr fbk_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_pub_;
    rclcpp::TimerBase::SharedPtr cmd_timer_, poll_timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToBLVR>();
    rclcpp::executors::MultiThreadedExecutor exec; // parallels callbacks
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
