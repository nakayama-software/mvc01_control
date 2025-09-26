// blvr.cpp
#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include <modbus/modbus.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

// ================= Helpers =================
static inline std::array<uint16_t, 2> i32_to_regs_be(int32_t v)
{
    uint32_t u = static_cast<uint32_t>(v);
    return {static_cast<uint16_t>((u >> 16) & 0xFFFF),
            static_cast<uint16_t>(u & 0xFFFF)};
}
static inline int32_t regs_be_to_i32(uint16_t hi, uint16_t lo)
{
    uint32_t u = (static_cast<uint32_t>(hi) << 16) | static_cast<uint32_t>(lo);
    return static_cast<int32_t>(u);
}
static inline geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sy;
    q.w = cy;
    return q;
}

// ================= Modbus wrapper =================
class Mvc01Modbus
{
public:
    Mvc01Modbus(const std::string &port, int baud, char parity, int stopbits, int slave, double timeout_s)
        : port_(port), baud_(baud), stopbits_(stopbits), slave_(slave), parity_(parity), timeout_s_(timeout_s) {}

    ~Mvc01Modbus() { close(); }

    void connect()
    {
        close();
        ctx_ = modbus_new_rtu(port_.c_str(), baud_, parity_, 8, stopbits_);
        if (!ctx_)
            throw std::runtime_error("modbus_new_rtu failed");
        if (modbus_set_slave(ctx_, slave_) == -1)
            throw std::runtime_error("modbus_set_slave failed");
        const uint32_t usec = static_cast<uint32_t>(timeout_s_ * 1e6);
        struct timeval tv{static_cast<time_t>(usec / 1000000), static_cast<suseconds_t>(usec % 1000000)};
        modbus_set_response_timeout(ctx_, tv.tv_sec, tv.tv_usec);
        modbus_set_byte_timeout(ctx_, tv.tv_sec, tv.tv_usec);
        if (modbus_connect(ctx_) == -1)
        {
            modbus_free(ctx_);
            ctx_ = nullptr;
            throw std::runtime_error("modbus_connect failed");
        }
    }

    void close()
    {
        if (ctx_)
        {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }
    }

    bool ok() const { return ctx_ != nullptr; }

    // retry once on failure
    template <typename Fn>
    auto with_retry(Fn &&fn)
    {
        try
        {
            return fn();
        }
        catch (...)
        {
            connect();
            return fn();
        }
    }

    void write_u32(int addr, int32_t val)
    {
        auto regs = i32_to_regs_be(val);
        int rc = modbus_write_registers(ctx_, addr, 2, regs.data());
        if (rc != 2) {
            int err = errno;
            std::string msg = modbus_strerror(err);
            throw std::runtime_error("modbus_write_registers failed: " + msg);
        }
    }

    int32_t read_i32(int addr)
    {
        uint16_t regs[2] = {0, 0};
        int rc = modbus_read_registers(ctx_, addr, 2, regs);
        if (rc != 2)
            throw std::runtime_error("modbus_read_registers failed");
        return regs_be_to_i32(regs[0], regs[1]);
    }

    void read_block(int addr, int count, uint16_t *out)
    {
        int rc = modbus_read_registers(ctx_, addr, count, out);
        if (rc != count)
            throw std::runtime_error("modbus_read_registers block failed");
    }

private:
    std::string port_;
    int baud_, stopbits_, slave_;
    char parity_;
    double timeout_s_;
    modbus_t *ctx_{nullptr};
};

// ================= Node =================
class BLVR : public rclcpp::Node
{
public:
    BLVR() : Node("blvr")
    {
        // Params
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter<int>("baudrate", 230400);
        parity_s_ = this->declare_parameter<std::string>("parity", "E");
        stopbits_ = this->declare_parameter<int>("stopbits", 1);
        slave_id_ = this->declare_parameter<int>("slave_id", 1);
        timeout_s_ = this->declare_parameter<double>("timeout_s", 2.0);
        lin_scale_ = this->declare_parameter<double>("linear_scale", 0.6);
        ang_scale_ = this->declare_parameter<double>("angular_scale", 2.5);
        vx_limit_ = this->declare_parameter<double>("vx_limit", 0.5);
        w_limit_ = this->declare_parameter<double>("w_limit", 1.68);
        cmd_rate_hz_ = std::max(5.0, this->declare_parameter<double>("cmd_rate_hz", 50.0));
        poll_rate_hz_ = std::max(1.0, this->declare_parameter<double>("poll_rate_hz", 20.0));
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        imu_frame_ = this->declare_parameter<std::string>("imu_frame", "imu_link");

        // Monitor register params
        addr_disp_fb_ = this->declare_parameter<int>("addr_disp_fb", 2144);
        addr_disp_lr_ = this->declare_parameter<int>("addr_disp_lr", 2146);
        addr_wodo_x_ = this->declare_parameter<int>("addr_wodo_x", 2138);
        addr_wodo_y_ = this->declare_parameter<int>("addr_wodo_y", 2140);
        addr_wodo_th_ = this->declare_parameter<int>("addr_wodo_th", 2142);
        addr_godo_x_ = this->declare_parameter<int>("addr_godo_x", 2064);
        addr_godo_y_ = this->declare_parameter<int>("addr_godo_y", 2066);
        addr_godo_yaw_ = this->declare_parameter<int>("addr_godo_yaw", 2070);
        addr_imu_ax_ = this->declare_parameter<int>("addr_imu_ax", 2084);
        addr_imu_ay_ = this->declare_parameter<int>("addr_imu_ay", 2086);
        addr_imu_az_ = this->declare_parameter<int>("addr_imu_az", 2088);
        addr_imu_gx_ = this->declare_parameter<int>("addr_imu_gx", 2090);
        addr_imu_gy_ = this->declare_parameter<int>("addr_imu_gy", 2092);
        addr_imu_gz_ = this->declare_parameter<int>("addr_imu_gz", 2094);
        addr_imu_tmp_ = this->declare_parameter<int>("addr_imu_tmp", 2096);
        addr_cmd_vx_ = this->declare_parameter<int>("addr_cmd_vx", 2100);
        addr_cmd_vlr_ = this->declare_parameter<int>("addr_cmd_vlr", 2102);
        addr_cmd_w_ = this->declare_parameter<int>("addr_cmd_w", 2104);
        addr_fbk_vx_ = this->declare_parameter<int>("addr_fbk_vx", 2106);
        addr_fbk_vlr_ = this->declare_parameter<int>("addr_fbk_vlr", 2108);
        addr_fbk_w_ = this->declare_parameter<int>("addr_fbk_w", 2110);
        addr_ctl_tmp_ = this->declare_parameter<int>("addr_ctl_tmp", -1);
        addr_supply_v_ = this->declare_parameter<int>("addr_supply_v", -1);
        addr_pwr_cnt_ = this->declare_parameter<int>("addr_power_on", -1);
        addr_boot_ms_ = this->declare_parameter<int>("addr_boot_ms", -1);
        addr_torque1_ = this->declare_parameter<int>("addr_torque1", -1);
        addr_torque2_ = this->declare_parameter<int>("addr_torque2", -1);
        addr_torque3_ = this->declare_parameter<int>("addr_torque3", -1);
        addr_torque4_ = this->declare_parameter<int>("addr_torque4", -1);
        torque_scale_ = this->declare_parameter<double>("torque_scale", 0.001);

        // Modbus
        mb_ = std::make_unique<Mvc01Modbus>(port_, baudrate_, parity_s_.empty() ? 'E' : parity_s_[0],
                                            stopbits_, slave_id_, timeout_s_);
        mb_->connect();
        RCLCPP_INFO(get_logger(), "Modbus open on %s", port_.c_str());
        mb_->with_retry([&] { mb_->write_u32(ADDR_MODE, 1); return 0; }); // Vx-ω mode

        // ROS IO
        auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", cmd_qos, std::bind(&BLVR::cmd_cb, this, _1));

        auto sensor_qos = rclcpp::SensorDataQoS();
        odom_wheel_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", sensor_qos);
        odom_gyro_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_gyro", sensor_qos);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", sensor_qos);
        imu_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu_temperature", sensor_qos);
        ctl_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/controller_temperature", sensor_qos);
        supply_pub_ = this->create_publisher<std_msgs::msg::Float32>("/supply_voltage", 10);
        power_on_pub_ = this->create_publisher<std_msgs::msg::Int32>("/power_on_count", 10);
        uptime_pub_ = this->create_publisher<std_msgs::msg::Float32>("/uptime_s", 10);
        disp_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/displacement", sensor_qos);
        cmd_speed_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/command_speed", 10);
        fbk_speed_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/feedback_speed", 10);
        torque_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/torque_pct", 10);

        // Timers
        cmd_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / cmd_rate_hz_)),
                                             std::bind(&BLVR::cmd_loop, this));
        poll_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / poll_rate_hz_)),
                                              std::bind(&BLVR::poll_loop, this));

        last_cmd_time_ = now();
        RCLCPP_INFO(get_logger(), "blvr ready");
    }

    ~BLVR() override
    {
        safe_stop();
        mb_.reset();
    }

private:
    // Command registers
    static constexpr int ADDR_MODE = 1986;
    static constexpr int ADDR_VX = 1988; // 0.001 m/s
    static constexpr int ADDR_W = 1990;  // 1e-6 rad/s
    static constexpr int ADDR_TRG = 2030;

    // Cmd callback
    void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(m_);
        last_cmd_ = *msg;
        last_cmd_time_ = now();
    }

    // Command loop
    void cmd_loop()
    {
        try
        {
            double vx_cmd = 0.0, w_cmd = 0.0;
            {
                std::lock_guard<std::mutex> lk(m_);
                bool stale = (now() - last_cmd_time_).seconds() > 0.5;
                if (!stale)
                {
                    vx_cmd = std::clamp(last_cmd_.linear.x, -vx_limit_, vx_limit_);
                    w_cmd = std::clamp(last_cmd_.angular.z, -w_limit_, w_limit_);
                }
            }
            int32_t vx_units = static_cast<int32_t>(std::llround(vx_cmd / 0.001));
            int32_t w_units = static_cast<int32_t>(std::llround(w_cmd / 1e-6));
            mb_->with_retry([&] { mb_->write_u32(ADDR_VX, vx_units); return 0; });
            mb_->with_retry([&] { mb_->write_u32(ADDR_W, w_units); return 0; });
            mb_->with_retry([&] { mb_->write_u32(ADDR_TRG, 1); return 0; });
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cmd error: %s", e.what());
            try { safe_stop(); } catch (...) {}
        }
    }

    // Poll loop
    void poll_loop()
    {
        try
        {
            auto stamp = this->now();

            // Wheel odom block: 2138..2143 (6 regs)
            {
                uint16_t r[6];
                mb_->with_retry([&]{ mb_->read_block(addr_wodo_x_, 6, r); return 0; });
                double x = regs_to_m(r[0], r[1]);
                double y = regs_to_m(r[2], r[3]);
                double th = regs_to_u6rad(r[4], r[5]);
                nav_msgs::msg::Odometry ow;
                ow.header.stamp = stamp;
                ow.header.frame_id = odom_frame_;
                ow.child_frame_id = base_frame_;
                ow.pose.pose.position.x = x;
                ow.pose.pose.position.y = y;
                ow.pose.pose.orientation = yaw_to_quat(th);
                odom_wheel_pub_->publish(ow);
            }

            // Gyro odom block: 2064..2071 (8 regs)
            {
                uint16_t r[8];
                mb_->with_retry([&]{ mb_->read_block(addr_godo_x_, 8, r); return 0; });
                double x = regs_to_m(r[0], r[1]);
                double y = regs_to_m(r[2], r[3]);
                double yaw = regs_to_u6rad(r[6], r[7]);
                nav_msgs::msg::Odometry og;
                og.header.stamp = stamp;
                og.header.frame_id = odom_frame_;
                og.child_frame_id = base_frame_;
                og.pose.pose.position.x = x;
                og.pose.pose.position.y = y;
                og.pose.pose.orientation = yaw_to_quat(yaw);
                odom_gyro_pub_->publish(og);
            }

            // IMU block
            {
                uint16_t r[14];
                mb_->with_retry([&]{ mb_->read_block(addr_imu_ax_, 14, r); return 0; });
                sensor_msgs::msg::Imu imu;
                imu.header.stamp = stamp;
                imu.header.frame_id = imu_frame_;
                imu.linear_acceleration.x = regs_to_acc(r[0], r[1]);
                imu.linear_acceleration.y = regs_to_acc(r[2], r[3]);
                imu.linear_acceleration.z = regs_to_acc(r[4], r[5]);
                imu.angular_velocity.x = regs_to_rps(r[6], r[7]);
                imu.angular_velocity.y = regs_to_rps(r[8], r[9]);
                imu.angular_velocity.z = regs_to_rps(r[10], r[11]);
                imu.orientation_covariance[0] = -1.0;
                imu_pub_->publish(imu);

                sensor_msgs::msg::Temperature t;
                t.header.stamp = stamp;
                t.header.frame_id = imu_frame_;
                t.temperature = regs_to_c(r[12], r[13]);
                imu_temp_pub_->publish(t);
            }

            // Cmd/Fbk speed block: 2100..2111 (12 regs)
            {
                uint16_t fb[6];
                mb_->with_retry([&]
                                { mb_->read_block(2492, 6, fb); return 0; });

                double vx_fb = regs_to_mps(fb[0], fb[1]); // 0.001 m/s
                double wz_fb = regs_to_rps(fb[2], fb[3]); // 1e-6 rad/s
                double vy_fb = regs_to_mps(fb[4], fb[5]); // 0.001 m/s

                geometry_msgs::msg::TwistStamped tw;
                tw.header.stamp = stamp;
                tw.twist.linear.x = vx_fb;
                tw.twist.linear.y = vy_fb;
                tw.twist.angular.z = wz_fb;
                fbk_speed_pub_->publish(tw);
            }

            // Displacement block: 2144..2147 (4 regs)
            {
                uint16_t r[4];
                mb_->with_retry([&]
                                { mb_->read_block(addr_disp_fb_, 4, r); return 0; });
                geometry_msgs::msg::Vector3Stamped v;
                v.header.stamp = stamp;
                v.header.frame_id = base_frame_;
                v.vector.x = regs_to_m(r[0], r[1]);
                v.vector.y = regs_to_m(r[2], r[3]);
                v.vector.z = 0.0;
                disp_pub_->publish(v);
            }

            // Optional monitors
            if (addr_ctl_tmp_ > 0)
            {
                sensor_msgs::msg::Temperature t;
                t.header.stamp = stamp;
                t.header.frame_id = "controller";
                t.temperature = read_scaled(addr_ctl_tmp_, 1e-3);
                ctl_temp_pub_->publish(t);
            }
            if (addr_supply_v_ > 0)
            {
                std_msgs::msg::Float32 v;
                v.data = read_scaled(addr_supply_v_, 1e-3);
                supply_pub_->publish(v);
            }
            if (addr_pwr_cnt_ > 0)
            {
                std_msgs::msg::Int32 c;
                c.data = mb_->with_retry([&]
                                         { return mb_->read_i32(addr_pwr_cnt_); });
                power_on_pub_->publish(c);
            }
            if (addr_boot_ms_ > 0)
            {
                std_msgs::msg::Float32 s;
                s.data = static_cast<float>(read_scaled(addr_boot_ms_, 1e-3));
                uptime_pub_->publish(s);
            }
            if (addr_torque1_ > 0 || addr_torque2_ > 0 || addr_torque3_ > 0 || addr_torque4_ > 0)
            {
                std_msgs::msg::Float32MultiArray arr;
                if (addr_torque1_ > 0)
                    arr.data.push_back(static_cast<float>(read_scaled(addr_torque1_, torque_scale_)));
                if (addr_torque2_ > 0)
                    arr.data.push_back(static_cast<float>(read_scaled(addr_torque2_, torque_scale_)));
                if (addr_torque3_ > 0)
                    arr.data.push_back(static_cast<float>(read_scaled(addr_torque3_, torque_scale_)));
                if (addr_torque4_ > 0)
                    arr.data.push_back(static_cast<float>(read_scaled(addr_torque4_, torque_scale_)));
                torque_pub_->publish(arr);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "poll error: %s (retry)", e.what());
            try { mb_->connect(); } catch (...) { RCLCPP_ERROR(get_logger(), "modbus reconnect failed"); }
        }
    }

    // Scales
    static inline double regs_to_m(uint16_t hi, uint16_t lo) { return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-3; }
    static inline double regs_to_u6rad(uint16_t hi, uint16_t lo) { return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-6; }
    static inline double regs_to_acc(uint16_t hi, uint16_t lo) { return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-3; } // m/s^2
    static inline double regs_to_rps(uint16_t hi, uint16_t lo) { return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-6; } // rad/s
    static inline double regs_to_mps(uint16_t hi, uint16_t lo) { return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-3; } // m/s
    static inline double regs_to_c(uint16_t hi, uint16_t lo) { return static_cast<double>(regs_be_to_i32(hi, lo)) * 1e-3; }   // degC

    double read_scaled(int addr, double s)
    {
        int32_t v = mb_->with_retry([&]
                                    { return mb_->read_i32(addr); });
        return static_cast<double>(v) * s;
    }

    void safe_stop()
    {
        if (!mb_ || !mb_->ok())
            return;
        try { mb_->with_retry([&]{ mb_->write_u32(ADDR_VX,0); return 0; }); } catch (...) {}
        try { mb_->with_retry([&]{ mb_->write_u32(ADDR_W, 0); return 0; }); } catch (...) {}
        try { mb_->with_retry([&]{ mb_->write_u32(ADDR_TRG,1); return 0; }); } catch (...) {}
    }

    // State
    std::mutex m_;
    geometry_msgs::msg::Twist last_cmd_;
    rclcpp::Time last_cmd_time_;

    // Modbus
    std::unique_ptr<Mvc01Modbus> mb_;

    // Params
    std::string port_, parity_s_;
    int baudrate_{230400}, stopbits_{1}, slave_id_{1};
    double timeout_s_{2.0}, cmd_rate_hz_{50.0}, poll_rate_hz_{20.0};
    double lin_scale_{0.6}, ang_scale_{2.5}, vx_limit_{0.5}, w_limit_{1.68};
    std::string odom_frame_{"odom"}, base_frame_{"base_link"}, imu_frame_{"imu_link"};

    // Monitor addresses
    int addr_disp_fb_, addr_disp_lr_;
    int addr_wodo_x_, addr_wodo_y_, addr_wodo_th_;
    int addr_godo_x_, addr_godo_y_, addr_godo_yaw_;
    int addr_imu_ax_, addr_imu_ay_, addr_imu_az_, addr_imu_gx_, addr_imu_gy_, addr_imu_gz_, addr_imu_tmp_;
    int addr_cmd_vx_, addr_cmd_vlr_, addr_cmd_w_, addr_fbk_vx_, addr_fbk_vlr_, addr_fbk_w_;
    int addr_ctl_tmp_, addr_supply_v_, addr_pwr_cnt_, addr_boot_ms_;
    int addr_torque1_, addr_torque2_, addr_torque3_, addr_torque4_;
    double torque_scale_;

    // ROS IO
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_wheel_pub_, odom_gyro_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp_pub_, ctl_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr supply_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr power_on_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr uptime_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr disp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_speed_pub_, fbk_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_pub_;
    rclcpp::TimerBase::SharedPtr cmd_timer_, poll_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BLVR>());
    rclcpp::shutdown();
    return 0;
}
