//
// Created by Sam Cheung on 7/26/24.
//

#include "rmcv.h"
#include "rmcv_hardware.h"

cv::Mat cammat = (cv::Mat_<double>(3, 3) <<
    1782.672144409928f, 0.0f, 598.8983414505224f,
    0.0f, 1783.860175007369f, 523.4209809658056f,
    0.0f, 0.0f, 1.0f);
cv::Mat discof = (cv::Mat_<double>(1, 5) <<
    -0.03436366268485048f, 0.1953669264956857f, 0.0001485060439399386f, -0.003814875777013483f, -
    0.3181808766352414f);
cv::Mat h_gripper2camera = (cv::Mat_<double>(4, 4) <<
    0.009496335714993975f, -0.01011098215449402f, -0.9999037891956705f, -278.4485137621207f,
    0.9997472036560227f, -0.02028397331158516f, 0.009699959209097164f, -130.1949669375455f,
    -0.02038009788866046f, -0.999743131242504f, 0.00991580271497039f, 43.81158762150466f,
    0.0f, 0.0f, 0.0f, 1.0f);

const cv::Ptr<cv::ml::SVM> svm_red = cv::ml::SVM::load("svm.xml");
const cv::Ptr<cv::ml::SVM> svm_blue = cv::ml::SVM::load("svm.xml");

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

void serial_function(rm::parallel_queue<serial_package>& serial_queue);

void frame_function(rm::parallel_queue<serial_package>& serial_queue, rm::parallel_queue<frame_package>& frame_queue);

void process_function(rm::parallel_queue<frame_package>& frame_queue,
                      rm::parallel_queue<std::vector<rm::armour>>& armour_queue,
                      rm::parallel_queue<cv::Mat>& debug_queue);

int main()
{
    rm::parallel_queue<serial_package> serial_queue;
    std::thread serial_thread(serial_function, std::ref(serial_queue));

    rm::parallel_queue<frame_package> frame_queue;
    std::thread frame_thread(frame_function, std::ref(serial_queue), std::ref(frame_queue));

    rm::parallel_queue<std::vector<rm::armour>> armour_queue;
    rm::parallel_queue<cv::Mat> debug_queue;
    std::thread process_thread(process_function, std::ref(frame_queue), std::ref(armour_queue), std::ref(debug_queue));

    std::thread debug_thread([](rm::parallel_queue<cv::Mat>& debug_queue)
    {
        while (1)
        {
            const auto image = debug_queue.pop();
            cv::Mat smoll;
            resize(*image, smoll, cv::Size(640, 480));
            imshow("debug", smoll);
            cv::waitKey(1);
        }
    }, std::ref(debug_queue));

    serial_thread.join();
    frame_thread.join();
    process_thread.join();
    debug_thread.join();
}

void serial_function(rm::parallel_queue<serial_package>& serial_queue)
{
    rm::serial_port serial;
    bool status = serial.initialize("/dev/ttyUSB0", B460800);

    int error_counter = 0;
    while (status)
    {
        unsigned char buffer[256];
        serial.receive(buffer, 24);

        if (buffer[0] != 0x38 ||
            buffer[23] != rm::lookup_CRC(buffer, 23))
        {
            if (error_counter++ > 10)
            {
                serial.destroyed();
                status = serial.initialize("/dev/ttyUSB0", B460800);
                error_counter = 0;
            }
            std::cout << "error: " << error_counter << std::endl;
            continue;
        }

        float pitch, yaw, roll;
        std::memcpy(&yaw, buffer + 3, sizeof(float));
        std::memcpy(&pitch, buffer + 11, sizeof(float));
        std::memcpy(&roll, buffer + 15, sizeof(float));

        if (!serial_queue.empty()) serial_queue.tryPop();
        serial_queue.push({
            buffer[1] & 0x01 ? rm::camp::CAMP_RED : rm::camp::CAMP_BLUE,
            static_cast<double>(pitch) * CV_PI / 180.0f,
            static_cast<double>(yaw) * CV_PI / 180.0f,
            static_cast<double>(roll) * CV_PI / 180.0f
        });
    }
}

void frame_function(rm::parallel_queue<serial_package>& serial_queue, rm::parallel_queue<frame_package>& frame_queue)
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
}

void process_function(rm::parallel_queue<frame_package>& frame_queue,
                      rm::parallel_queue<std::vector<rm::armour>>& armour_queue,
                      rm::parallel_queue<cv::Mat>& debug_queue)
{
    while (1)
    {
        const auto frame = frame_queue.pop();
        const auto r = rm::utils::euler2matrix(frame->package.roll, frame->package.pitch, frame->package.yaw);

        cv::Mat h_base2gripper = cv::Mat::eye(4, 4, CV_64F);
        r.copyTo(h_base2gripper(cv::Rect(0, 0, 3, 3)));

        auto contours = extract_color(frame->image, rm::CAMP_BLUE, 80);
        auto [positive, negtive] =
            filter_lightblobs(contours, 70, {1.5, 80}, {10, 99999}, rm::CAMP_BLUE);
        auto armours =
            filter_armours(positive, 12, 22, 0.9, rm::CAMP_BLUE);

        for (auto& armour : armours)
        {
            cv::Mat icon = rm::affine_correction(frame->image, armour.icon, {20, 20});
            int label = static_cast<int>(svm_red->predict(icon));
            armour.identity.insert({label, 1});

            auto [rvec, tvec] =
                rm::solve_PnP(armour.vertices, cammat, discof, {27, 27});
            cv::Mat camera_position = cv::Mat::ones(4, 1, CV_64F);
            tvec.copyTo(camera_position(cv::Rect(0, 0, 1, 3)));

            cv::Mat world_position = h_base2gripper * (h_gripper2camera * camera_position);
            armour.position = cv::Point3d(world_position.at<double>(0, 0),
                                          world_position.at<double>(1, 0),
                                          world_position.at<double>(2, 0));

            armour.timespan = frame->timestamp;
        }

        cv::Mat debug;
        frame->image.copyTo(debug);
        rm::debug::draw_lightblobs(positive, negtive, debug, -1);
        rm::debug::draw_armours(armours, debug, -1);

        if (!debug_queue.empty()) debug_queue.tryPop();
        debug_queue.push(debug);
    }
}
