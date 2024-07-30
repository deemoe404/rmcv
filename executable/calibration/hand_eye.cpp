//
// Created by Sam Cheung on 7/23/24.
//

#include "rmcv.h"
#include "rmcv_hardware.h"

struct serial_package
{
    rm::camp target;
    double pitch, yaw, roll;
};

int main()
{
    bool stop = false;

    rm::parallel_queue<serial_package> serial_queue;
    std::thread serial_thread([&serial_queue, &stop]
    {
        rm::serial_port serial;
        bool status = serial.initialize("/dev/ttyUSB0", B460800);

        int error_counter = 0;
        while (status && !stop)
        {
            if (error_counter > 10)
            {
                serial.destroyed();
                status = serial.initialize("/dev/ttyUSB0", B460800);
                error_counter = 0;
            }

            unsigned char buffer[256];
            serial.receive(buffer, 24);

            if (buffer[0] != 0x38 ||
                buffer[23] != rm::lookup_CRC(buffer, 23))
            {
                std::cout << "error" << std::endl;
                error_counter++;
                continue;
            }

            const rm::camp target_camp = buffer[1] & 0x01 ? rm::camp::CAMP_RED : rm::camp::CAMP_BLUE;

            float pitch, yaw, roll;
            std::memcpy(&yaw, buffer + 3, sizeof(float));
            std::memcpy(&pitch, buffer + 11, sizeof(float));
            std::memcpy(&roll, buffer + 15, sizeof(float));

            if (!serial_queue.empty()) serial_queue.tryPop();
            serial_queue.push({
                target_camp,
                static_cast<double>(pitch) * 180.0f / CV_PI,
                static_cast<double>(yaw) * 180.0f / CV_PI,
                static_cast<double>(roll) * 180.0f / CV_PI
            });

            std::cout << "pitch: " << pitch << " yaw: " << yaw << " roll: " << roll << std::endl;
        }
    });

    std::thread frame_thread([&serial_queue, &stop]
    {
        rm::hardware::daheng camera;
        auto status = camera.initialize("KE0210010004", false, 4000, 1);
        int i = 0;
        cv::Mat gryo_data_read;
        while (status && !stop)
        {
            cv::Mat image = camera.capture(true, true);
            if (image.empty()) break;

            imshow("preview", image);
            auto key = cv::waitKey(1);

            if (key == 'c')
            {
                imwrite("data/20240630/" + std::to_string(i++) + ".png", image);
                auto package = serial_queue.pop();
                cv::Mat tmp = (cv::Mat_<double>(1, 3, CV_64F) << package->pitch, package->yaw, package->roll);
                gryo_data_read.push_back(tmp);
            }

            if (key == 'q')
            {
                cv::FileStorage fs("data/20240630.xml", cv::FileStorage::WRITE);
                fs << "data" << gryo_data_read;
                fs.release();
                stop = true;
                break;
            }
        }
    });

    serial_thread.join();
    frame_thread.join();

    cv::Size pattern_size(11, 8);
    std::vector<cv::Point3f> objp(pattern_size.width * pattern_size.height);
    std::generate(objp.begin(), objp.end(), [n = 0, &pattern_size]() mutable -> cv::Point3f
    {
        const int i = n / pattern_size.width, j = n++ % pattern_size.width;
        return {static_cast<float>(j) * 30, static_cast<float>(i) * 30, 0};
    });

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
    for (int i = 1; i <= 30; i++)
    {
        cv::Mat gray = imread("data/2079/" + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE);
        if (std::vector<cv::Point2f> corners;
            findChessboardCorners(gray, pattern_size, corners))
        {
            cornerSubPix(gray, corners, cv::Size(14, 14), cv::Size(-1, -1), criteria);
            object_points.push_back(objp);
            image_points.push_back(corners);
        }
        else std::cout << "error" << std::endl;
    }

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    const auto error =
        calibrateCamera(object_points, image_points, {1280, 1024}, cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << cameraMatrix << std::endl << distCoeffs << std::endl << error << std::endl;

    cv::FileStorage fs("data/2079.xml", cv::FileStorage::READ);
    cv::Mat gryo_data;
    fs["data"] >> gryo_data;
    fs.release();

    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    for (int i{0}; i < rvecs.size(); i++)
    {
        const double z = gryo_data.at<double>(i, 1),
                     y = gryo_data.at<double>(i, 0),
                     x = gryo_data.at<double>(i, 2);

        auto r = rm::utils::euler2matrix(x, y, z);

        R_gripper2base.push_back(r);
        t_gripper2base.push_back(cv::Mat::zeros(3, 1,CV_64F));
    }

    cv::Mat R_cam2gripper, t_cam2gripper;
    calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs, R_cam2gripper, t_cam2gripper);
    std::cout << R_cam2gripper << std::endl << t_cam2gripper << std::endl;

    for (int i{0}; i < rvecs.size(); i++)
    {
        const double z = gryo_data.at<double>(i, 1),
                     y = gryo_data.at<double>(i, 0),
                     x = gryo_data.at<double>(i, 2);

        auto r = rm::utils::euler2matrix(x, y, z);

        cv::Mat h_gripper2camera = cv::Mat::eye(4, 4, CV_64F);
        R_cam2gripper.copyTo(h_gripper2camera(cv::Rect(0, 0, 3, 3)));
        t_cam2gripper.copyTo(h_gripper2camera(cv::Rect(3, 0, 1, 3)));

        cv::Mat camera_position = cv::Mat::ones(4, 1, CV_64F);
        tvecs[i].copyTo(camera_position(cv::Rect(0, 0, 1, 3)));

        cv::Mat h_base2gripper = cv::Mat::eye(4, 4, CV_64F);
        r.copyTo(h_base2gripper(cv::Rect(0, 0, 3, 3)));

        cv::Mat world_position = h_base2gripper * (h_gripper2camera * camera_position);
        std::cout << "x: " << world_position.at<double>(0) << " y: " << world_position.at<double>(1) << " z: " <<
            world_position.at<double>(2) << std::endl;
    }

    return 0;
}
