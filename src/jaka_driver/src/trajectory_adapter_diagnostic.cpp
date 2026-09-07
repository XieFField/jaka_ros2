#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "jaka_driver/trajectory_utils.hpp"

namespace
{

const std::vector<std::string> kJointNames{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};

const std::vector<double> kHomePositions{
    -3.160921066038505,
    1.6080484013290657,
    -2.6790895150928824,
    2.6720179543589007,
    0.027910401738015497,
    -2.4753825550678896};

struct Options
{
    std::string preset{"joint1_negative_0p02"};
    std::string input_path;
    std::string output_path{"/tmp/jaka_servo_schedule_preview.csv"};
    std::vector<double> actual_initial_positions;
    unsigned int maximum_step_num{50U};
    std::size_t maximum_samples{50000U};
    double maximum_start_error{0.002};
    double maximum_velocity{std::numeric_limits<double>::infinity()};
    double maximum_acceleration{std::numeric_limits<double>::infinity()};
    double velocity_deadband{1e-9};
};

std::vector<std::string> split(const std::string & text, char delimiter)
{
    std::vector<std::string> fields;
    std::stringstream stream(text);
    std::string field;
    while (std::getline(stream, field, delimiter))
    {
        fields.push_back(field);
    }
    if (!text.empty() && text.back() == delimiter)
    {
        fields.emplace_back();
    }
    return fields;
}

double parse_double(const std::string & text, const std::string & label)
{
    std::size_t parsed = 0U;
    const double value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value))
    {
        throw std::invalid_argument(label + " 必须是有限数值");
    }
    return value;
}

std::vector<double> parse_joint_vector(const std::string & text)
{
    const auto fields = split(text, ',');
    if (fields.size() != kJointNames.size())
    {
        throw std::invalid_argument("--actual-initial 必须包含六个逗号分隔的关节角");
    }
    std::vector<double> values;
    values.reserve(fields.size());
    for (const auto & field : fields)
    {
        values.push_back(parse_double(field, "关节角"));
    }
    return values;
}

void print_usage(const char * program)
{
    std::cout
        << "用法: " << program << " [选项]\n"
        << "  --preset joint1_negative_0p02|joint1_negative_0p10\n"
        << "  --input PATH              读取原始 Goal CSV，覆盖 preset\n"
        << "  --output PATH             输出 servo_j 调度 CSV\n"
        << "  --actual-initial q1,...,q6 诊断 Goal 起点与实际起点连续性\n"
        << "  --maximum-step-num N      单次 servo_j 最大 step_num，范围 1..50\n"
        << "  --maximum-start-error RAD 起点连续性门限\n"
        << "  --maximum-velocity RAD_S  六轴隐含速度门限\n"
        << "  --maximum-acceleration RAD_S2 六轴隐含加速度门限\n"
        << "\n输入 CSV 必需列: time_from_start_s,joint_1,...,joint_6\n"
        << "可选列: velocity_joint_1..6,acceleration_joint_1..6\n";
}

Options parse_options(int argc, char ** argv)
{
    Options options;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h")
        {
            print_usage(argv[0]);
            std::exit(0);
        }
        if (index + 1 >= argc)
        {
            throw std::invalid_argument("选项缺少值: " + argument);
        }
        const std::string value = argv[++index];
        if (argument == "--preset")
        {
            options.preset = value;
        }
        else if (argument == "--input")
        {
            options.input_path = value;
        }
        else if (argument == "--output")
        {
            options.output_path = value;
        }
        else if (argument == "--actual-initial")
        {
            options.actual_initial_positions = parse_joint_vector(value);
        }
        else if (argument == "--maximum-step-num")
        {
            const auto parsed = std::stoul(value);
            if (parsed == 0U || parsed > jaka_driver::kMaximumServoStepNum)
            {
                throw std::invalid_argument("--maximum-step-num 必须在 1..50 内");
            }
            options.maximum_step_num = static_cast<unsigned int>(parsed);
        }
        else if (argument == "--maximum-samples")
        {
            options.maximum_samples = std::stoul(value);
        }
        else if (argument == "--maximum-start-error")
        {
            options.maximum_start_error = parse_double(value, argument);
        }
        else if (argument == "--maximum-velocity")
        {
            options.maximum_velocity = parse_double(value, argument);
        }
        else if (argument == "--maximum-acceleration")
        {
            options.maximum_acceleration = parse_double(value, argument);
        }
        else if (argument == "--velocity-deadband")
        {
            options.velocity_deadband = parse_double(value, argument);
        }
        else
        {
            throw std::invalid_argument("未知选项: " + argument);
        }
    }
    if (options.output_path.empty() || options.maximum_samples == 0U ||
        options.maximum_start_error < 0.0 || options.maximum_velocity <= 0.0 ||
        options.maximum_acceleration <= 0.0 || options.velocity_deadband < 0.0)
    {
        throw std::invalid_argument("诊断门限或输出路径无效");
    }
    return options;
}

void set_duration(double seconds, builtin_interfaces::msg::Duration & duration)
{
    const auto whole = static_cast<int32_t>(std::floor(seconds));
    duration.sec = whole;
    duration.nanosec = static_cast<uint32_t>(
        std::llround((seconds - static_cast<double>(whole)) * 1e9));
    if (duration.nanosec == 1000000000U)
    {
        ++duration.sec;
        duration.nanosec = 0U;
    }
}

trajectory_msgs::msg::JointTrajectory make_preset(const std::string & name)
{
    double delta = 0.0;
    double duration = 0.0;
    if (name == "joint1_negative_0p02")
    {
        delta = -0.02;
        duration = 2.0;
    }
    else if (name == "joint1_negative_0p10")
    {
        delta = -0.10;
        duration = 5.0;
    }
    else
    {
        throw std::invalid_argument("未知 preset: " + name);
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = kJointNames;
    constexpr std::size_t kIntervals = 20U;
    for (std::size_t index = 0U; index <= kIntervals; ++index)
    {
        const double ratio = static_cast<double>(index) /
            static_cast<double>(kIntervals);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = kHomePositions;
        point.positions[0] += delta * ratio;
        point.velocities.assign(kJointNames.size(), 0.0);
        point.accelerations.assign(kJointNames.size(), 0.0);
        if (index > 0U && index < kIntervals)
        {
            point.velocities[0] = delta / duration;
        }
        set_duration(duration * ratio, point.time_from_start);
        trajectory.points.push_back(std::move(point));
    }
    return trajectory;
}

trajectory_msgs::msg::JointTrajectory read_goal_csv(const std::string & path)
{
    std::ifstream stream(path);
    if (!stream)
    {
        throw std::runtime_error("无法打开输入 CSV: " + path);
    }
    std::string line;
    if (!std::getline(stream, line))
    {
        throw std::runtime_error("输入 CSV 为空");
    }
    const auto header = split(line, ',');
    const auto find_column = [&header](const std::string & name) {
            const auto iterator = std::find(header.begin(), header.end(), name);
            if (iterator == header.end())
            {
                return std::numeric_limits<std::size_t>::max();
            }
            return static_cast<std::size_t>(std::distance(header.begin(), iterator));
        };
    const auto time_column = find_column("time_from_start_s");
    if (time_column == std::numeric_limits<std::size_t>::max())
    {
        throw std::runtime_error("输入 CSV 缺少 time_from_start_s");
    }
    std::vector<std::size_t> position_columns;
    std::vector<std::size_t> velocity_columns;
    std::vector<std::size_t> acceleration_columns;
    for (const auto & name : kJointNames)
    {
        position_columns.push_back(find_column(name));
        velocity_columns.push_back(find_column("velocity_" + name));
        acceleration_columns.push_back(find_column("acceleration_" + name));
    }
    if (std::any_of(
            position_columns.begin(), position_columns.end(),
            [](std::size_t column) {
                return column == std::numeric_limits<std::size_t>::max();
            }))
    {
        throw std::runtime_error("输入 CSV 缺少一个或多个 joint_1..joint_6 列");
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = kJointNames;
    std::size_t line_number = 1U;
    while (std::getline(stream, line))
    {
        ++line_number;
        if (line.empty())
        {
            continue;
        }
        const auto fields = split(line, ',');
        const auto read_column = [&fields, line_number](std::size_t column) {
                if (column >= fields.size())
                {
                    throw std::runtime_error(
                        "CSV 第 " + std::to_string(line_number) + " 行列数不足");
                }
                return parse_double(fields[column], "CSV 数值");
            };
        trajectory_msgs::msg::JointTrajectoryPoint point;
        const double time = read_column(time_column);
        if (time < 0.0)
        {
            throw std::runtime_error("CSV time_from_start_s 不能为负");
        }
        set_duration(time, point.time_from_start);
        for (const auto column : position_columns)
        {
            point.positions.push_back(read_column(column));
        }
        const auto optional_values_present =
            [&fields, line_number](
            const std::vector<std::size_t> & columns,
            const std::string & label) {
                std::size_t present = 0U;
                for (const auto column : columns)
                {
                    if (column != std::numeric_limits<std::size_t>::max() &&
                        column < fields.size() && !fields[column].empty())
                    {
                        ++present;
                    }
                }
                if (present != 0U && present != columns.size())
                {
                    throw std::runtime_error(
                        "CSV 第 " + std::to_string(line_number) +
                        " 行的 " + label + " 必须六轴全有或全空");
                }
                return present == columns.size();
            };
        const bool has_velocities = optional_values_present(
            velocity_columns, "velocity");
        const bool has_accelerations = optional_values_present(
            acceleration_columns, "acceleration");
        if (has_velocities)
        {
            for (const auto column : velocity_columns)
            {
                point.velocities.push_back(read_column(column));
            }
        }
        if (has_accelerations)
        {
            for (const auto column : acceleration_columns)
            {
                point.accelerations.push_back(read_column(column));
            }
        }
        trajectory.points.push_back(std::move(point));
    }
    if (trajectory.points.empty())
    {
        throw std::runtime_error("输入 CSV 没有数据行");
    }
    return trajectory;
}

}  // namespace

int main(int argc, char ** argv)
{
    try
    {
        const auto options = parse_options(argc, argv);
        const auto trajectory = options.input_path.empty() ?
            make_preset(options.preset) : read_goal_csv(options.input_path);
        std::string validation_error;
        if (!jaka_driver::validate_trajectory(
                trajectory, kJointNames, 300.0, validation_error))
        {
            throw std::runtime_error("原始 Goal 无效: " + validation_error);
        }
        std::vector<double> actual_initial = options.actual_initial_positions;
        if (actual_initial.empty())
        {
            actual_initial = trajectory.points.front().positions;
        }
        const auto schedule = jaka_driver::build_queued_servo_schedule(
            trajectory, kJointNames, actual_initial,
            jaka_driver::kJakaServoInterpolationCycle,
            options.maximum_step_num, options.maximum_samples);
        if (!schedule.valid)
        {
            throw std::runtime_error("转换失败: " + schedule.error);
        }
        const auto diagnostics = jaka_driver::analyze_queued_servo_schedule(
            schedule, actual_initial, options.velocity_deadband);
        if (!diagnostics.valid)
        {
            throw std::runtime_error("诊断失败: " + diagnostics.error);
        }
        std::string csv_error;
        if (!jaka_driver::write_queued_servo_schedule_csv(
                options.output_path, schedule, kJointNames, csv_error))
        {
            throw std::runtime_error(csv_error);
        }

        std::cout << std::fixed << std::setprecision(9)
                  << "TRAJECTORY ADAPTER PREVIEW: source_points="
                  << trajectory.points.size()
                  << ", servo_segments=" << schedule.setpoints.size()
                  << ", planned_duration=" << schedule.planned_duration
                  << " s, scheduled_duration=" << schedule.scheduled_duration
                  << " s, duration_error=" << diagnostics.duration_error
                  << " s, step_num=[" << diagnostics.minimum_step_num
                  << ',' << diagnostics.maximum_step_num << "]\n";
        bool passed = diagnostics.maximum_start_position_error <=
            options.maximum_start_error;
        for (std::size_t joint = 0U; joint < kJointNames.size(); ++joint)
        {
            std::cout << kJointNames[joint]
                      << ": max_velocity="
                      << diagnostics.maximum_absolute_velocity[joint]
                      << " rad/s, max_acceleration="
                      << diagnostics.maximum_absolute_acceleration[joint]
                      << " rad/s^2, positive_segments="
                      << diagnostics.positive_velocity_segments[joint]
                      << ", negative_segments="
                      << diagnostics.negative_velocity_segments[joint]
                      << ", sign_changes="
                      << diagnostics.velocity_sign_changes[joint] << '\n';
            passed = passed &&
                diagnostics.maximum_absolute_velocity[joint] <=
                options.maximum_velocity &&
                diagnostics.maximum_absolute_acceleration[joint] <=
                options.maximum_acceleration;
        }
        const double joint1_delta = schedule.setpoints.back().positions[0] -
            schedule.source_start_positions[0];
        if (joint1_delta < -options.velocity_deadband &&
            diagnostics.positive_velocity_segments[0] != 0U)
        {
            passed = false;
        }
        std::cout << "start_error="
                  << diagnostics.maximum_start_position_error
                  << " rad at "
                  << kJointNames[diagnostics.maximum_start_error_joint]
                  << ", csv=" << options.output_path << '\n';
        std::cout << (passed ?
            "TRAJECTORY ADAPTER OFFLINE: PASS\n" :
            "TRAJECTORY ADAPTER OFFLINE: FAIL\n");
        return passed ? 0 : 3;
    }
    catch (const std::exception & exception)
    {
        std::cerr << "TRAJECTORY ADAPTER OFFLINE: ERROR: "
                  << exception.what() << '\n';
        return 2;
    }
}
