//
// Created by Sam Cheung on 7/23/24.
//

#include "rmcv.h"

int main()
{
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
        const double x = gryo_data.at<double>(i, 2) * CV_PI / 180.0f,
                     y = gryo_data.at<double>(i, 0) * CV_PI / 180.0f,
                     z = gryo_data.at<double>(i, 1) * CV_PI / 180.0f;

        auto r = rm::utils::euler2matrix(x, y, z);

        R_gripper2base.push_back(r);
        t_gripper2base.push_back(cv::Mat::zeros(3, 1,CV_64F));
    }

    cv::Mat R_cam2gripper, t_cam2gripper;
    calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs, R_cam2gripper, t_cam2gripper);
    std::cout << R_cam2gripper << std::endl << t_cam2gripper << std::endl;

    for (int i{0}; i < rvecs.size(); i++)
    {
        const double z = gryo_data.at<double>(i, 1) * CV_PI / 180.0f,
                     y = gryo_data.at<double>(i, 0) * CV_PI / 180.0f,
                     x = gryo_data.at<double>(i, 2) * CV_PI / 180.0f;

        auto r = rm::utils::euler2matrix(x, y, z);

        // cv::Mat offset = cv::Mat::eye(4, 4, CV_64F);
        // R_cam2gripper.copyTo(offset(cv::Rect(0, 0, 3, 3)));
        // t_cam2gripper.copyTo(offset(cv::Rect(3, 0, 1, 3)));

        cv::Mat h_gripper2camera = (cv::Mat_<double>(4, 4, CV_64F) <<
            0, 1, 0, 62.0f,
            0, 0, -1, 43.85f,
            1, 0, 0, 124.0f,
            0, 0, 0, 1
        );

        cv::Mat camera_position = cv::Mat::ones(4, 1, CV_64F);
        tvecs[i].copyTo(camera_position(cv::Rect(0, 0, 1, 3)));

        cv::Mat h_base2gripper = cv::Mat::eye(4, 4, CV_64F);
        r.copyTo(h_base2gripper(cv::Rect(0, 0, 3, 3)));

        cv::Mat world_position = h_base2gripper.inv() * (h_gripper2camera.inv() * camera_position);
        std::cout << "x: " << world_position.at<double>(0) << " y: " << world_position.at<double>(1) << " z: " <<
            world_position.at<double>(2) << std::endl;
    }

    return 0;
}
