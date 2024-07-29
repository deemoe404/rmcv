//
// Created by Sam Cheung on 7/26/24.
//

#include "rmcv.h"
#include "rmcv_hardware.h"

struct serial_package
{
    rm::camp target;
    double pitch, yaw, roll;
};

struct frame_package
{
    int64 timestamp;
    serial_package package;
    cv::Mat image;
};

int main()
{
    rm::parameters red, blue;

    rm::parallel_queue<serial_package> serial_queue;
    std::thread serial_thread([&serial_queue]
    {
        rm::serial_port serial;
        bool status = serial.initialize("/dev/ttyUSB0", B230400);

        int error_counter = 0;
        while (status)
        {
            if (error_counter > 10)
            {
                serial.destroyed();
                status = serial.initialize("/dev/ttyUSB0", B230400);
                error_counter = 0;
            }

            unsigned char buffer[256];
            serial.receive(buffer, 20);

            if (buffer[0] != 0x38 ||
                buffer[19] != rm::lookup_CRC(buffer, 19))
            {
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
        }
    });

    rm::parallel_queue<frame_package> frame_queue;
    std::thread frame_thread([&frame_queue, &serial_queue]
    {
        rm::hardware::daheng camera;
        const bool status = camera.initialize("KE0210010004", false, 4000, 1);

        while (status)
        {
            cv::Mat image = camera.capture(true, true);
            if (image.empty()) break;

            if (!frame_queue.empty()) frame_queue.tryPop();
            auto package = serial_queue.pop();
            frame_queue.push({cv::getTickCount(), *package, image});
        }
    });

    rm::parallel_queue<std::vector<rm::armour>> armour_queue;
    std::thread process_thread([&frame_queue, &armour_queue, &red, &blue]
    {
        const cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load("svm.xml");
        cv::Mat h_gripper2camera = (cv::Mat_<double>(4, 4, CV_64F) <<
            0, 1, 0, 62.0f,
            0, 0, -1, 43.85f,
            1, 0, 0, 124.0f,
            0, 0, 0, 1
        );

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3, CV_64F) <<
            1769.74867569615f, 0, 597.7484170363178f,
            0, 1765.772729849818f, 489.3341735750148f,
            0, 0, 1
        );
        cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5, CV_64F) <<
            -0.05190751755633928f,
            0.2697759328983004f,
            -0.001655826154699588f,
            -0.005314602725138653f,
            -0.6497586500456966f
        );

        while (1)
        {
            const auto frame = frame_queue.pop();
            const auto parms = frame->package.target == rm::camp::CAMP_RED ? red : blue;

            const auto contours = extract_color(frame->image, frame->package.target, parms.extraction_lower_bound);
            auto [positive_lightblobs, negtive_lightblobs] =
                filter_lightblobs(
                    contours,
                    parms.lightblob_tilt_max,
                    parms.lightblob_ratio,
                    parms.lightblob_area,
                    frame->package.target);

            std::vector<rm::armour> armours;
            filter_armours(positive_lightblobs, armours, parms.armour_angle_difference_max, parms.armour_shear_max,
                           parms.armour_lenght_ratio_max, frame->package.target);

            for (auto armour : armours)
            {
                auto [rvec, tvec] = rm::solve_PnP(armour.vertices, camera_matrix, dist_coeffs, {60, 60});
                auto icon = rm::affine_correction(frame->image, armour.icon, {20, 20});
                auto flat = rm::utils::flatten_image(icon, CV_32FC1, {20, 20});

                const float result = svm->predict(flat);
                auto r = rm::utils::euler2matrix(frame->package.roll, frame->package.pitch, frame->package.yaw);

                cv::Mat camera_position = cv::Mat::ones(4, 1, CV_64F);
                tvec.copyTo(camera_position(cv::Rect(0, 0, 1, 3)));

                cv::Mat h_base2gripper = cv::Mat::eye(4, 4, CV_64F);
                r.copyTo(h_base2gripper(cv::Rect(0, 0, 3, 3)));

                cv::Mat world_position = h_base2gripper.inv() * (h_gripper2camera.inv() * camera_position);
                armour.position = {
                    world_position.at<double>(0), world_position.at<double>(1), world_position.at<double>(2)
                };
            }
            armour_queue.push(armours);
        }
    });

    std::thread predict_thread([&armour_queue]
    {
        std::vector<rm::armour> tracking;

        while (1)
        {
            auto armours = armour_queue.pop();
            if (armours->empty()) continue;

            if (tracking.empty())
            {
                tracking = *armours; // first reconition
            }
            else
            {
                for (const auto armour : tracking)
                {
                    int index = 0;
                    float max_iou = 0;
                    bool lost = false;

                    for (int i{0}; i < armours->size(); i++)
                    {
                        if (armours->at(i).identity != armour.identity) // what to do?
                        {
                        }

                        float iou = 0;
                        // calculate iou between armours[i] and armour
                        if (max_iou < iou && iou > 0.5) // adjust threshold
                        {
                            max_iou = iou;
                            index = i;

                            lost = false;
                        }
                    }
                    if (lost) // what to do?
                    {
                    }

                    // update target using armours[index] e.g: target.update(armours[index]).location
                }
            }

            // deside a target to shoot at
        }
    });

    // write serial message thread
    std::thread response_thread([&frame_queue]
    {
    });
}
