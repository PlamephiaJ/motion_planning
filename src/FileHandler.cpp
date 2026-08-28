/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-23 19:12:35
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-07-01 11:50:00
 * @Description: File handler source file.
 */

#include "../include/motion_planning/FileHandler.hpp"

#include <stdexcept>

FileHandler::FileHandler(const std::string& file_path) : file_path(file_path) {}

const std::string& FileHandler::get_file_path() const
{
    return file_path;
}

CSVHandler::CSVHandler(const std::string& file_path) : FileHandler(file_path) {}

std::vector<geometry_msgs::msg::Point> CSVHandler::read_waypoint_list_from_csv()
{
    std::ifstream file;
    std::vector<geometry_msgs::msg::Point> waypoints_mapframe;
    file.open(file_path);
    if (!file.is_open())
    {
        throw std::invalid_argument("Couldn't open the file.");
    }

    std::string line;
    // pass title row.
    std::getline(file, line);
    std::size_t line_number = 1;
    while (std::getline(file, line))
    {
        ++line_number;
        if (line.find_first_not_of(" \t\r") == std::string::npos)
        {
            continue;
        }
        std::stringstream ss(line);
        std::string x_token;
        std::string y_token;
        if (!std::getline(ss, x_token, ',') ||
            !std::getline(ss, y_token, ','))
        {
            throw std::runtime_error(
                "Couldn't read x,y waypoint at CSV line " +
                std::to_string(line_number) + ".");
        }
        const double x = std::stod(x_token);
        const double y = std::stod(y_token);
        geometry_msgs::msg::Point point_mapframe;
        point_mapframe.x = x;
        point_mapframe.y = y;
        point_mapframe.z = 0.0;
        waypoints_mapframe.emplace_back(point_mapframe);
    }
    file.close();

    return waypoints_mapframe;
}

std::vector<PosAndSpeed> CSVHandler::read_waypoint_and_speed_list_from_csv()
{
    std::ifstream file;
    std::vector<PosAndSpeed> position_and_speed_list;
    file.open(file_path);
    if (!file.is_open())
    {
        throw std::invalid_argument("Couldn't open the file.");
    }

    std::string line;
    // pass title row.
    std::getline(file, line);
    std::size_t line_number = 1;
    while (std::getline(file, line))
    {
        ++line_number;
        if (line.find_first_not_of(" \t\r") == std::string::npos)
        {
            continue;
        }
        std::stringstream ss(line);
        std::string x_token;
        std::string y_token;
        std::string speed_token;
        if (!std::getline(ss, x_token, ',') ||
            !std::getline(ss, y_token, ',') ||
            !std::getline(ss, speed_token, ','))
        {
            throw std::runtime_error(
                "Couldn't read x,y,speed waypoint at CSV line " +
                std::to_string(line_number) + ".");
        }
        const double x = std::stod(x_token);
        const double y = std::stod(y_token);
        const double speed = std::stod(speed_token);

        PosAndSpeed pas;
        pas.position.x = x;
        pas.position.y = y;
        pas.position.z = 0.0;
        pas.speed = speed;

        position_and_speed_list.emplace_back(pas);
    }
    file.close();

    return position_and_speed_list;
}
