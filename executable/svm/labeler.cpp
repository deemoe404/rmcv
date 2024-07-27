//
// Created by Sam Cheung on 7/20/24.
//

#include "rmcv.h"

class video_controller
{
    int frame_index = 0;
    rm::range<int> frame_range = {0, 0};
    cv::VideoCapture capture;

public:
    cv::Mat frame;

    explicit video_controller(const std::string& video_path)
    {
        capture.open(video_path);
        frame_range = {0, static_cast<int>(capture.get(cv::CAP_PROP_FRAME_COUNT)) - 1};
        update(0);
    }

    bool update(const int offset)
    {
        bool rested = false;
        if (!frame_range.contains(frame_index + offset))
        {
            frame_index = offset > 0 ? frame_range.lower_bound : frame_range.upper_bound;
            rested = true;
        }

        frame_index = frame_index + offset;
        capture.set(cv::CAP_PROP_POS_FRAMES, frame_index);
        capture >> frame;

        return !rested;
    }

    void reset()
    {
        frame_index = 0;
    }
};

///
/// @param original_frame 
/// @param parameters 
/// @param target_camp 
/// @return 
auto reconizer(const cv::Mat& original_frame, const rm::parameters& parameters, const rm::color_camp target_camp)
    -> std::tuple<cv::Mat, std::vector<rm::lightblob>, std::vector<rm::contour>, std::vector<rm::armour>>
{
    cv::Mat binary;
    std::vector<rm::contour> contours;

    extract_color(original_frame, binary, target_camp, parameters.extraction_lower_bound);
    findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<rm::lightblob> lightblobs_positive;
    std::vector<rm::contour> lightblobs_negative;
    filter_lightblobs(contours, lightblobs_positive, lightblobs_negative, parameters.lightblob_tilt_max,
                      parameters.lightblob_ratio, parameters.lightblob_area, rm::CAMP_RED);

    std::vector<rm::armour> armours;
    filter_armours(lightblobs_positive, armours, parameters.armour_angle_difference_max,
                   parameters.armour_shear_max, parameters.armour_lenght_ratio_max, rm::CAMP_RED);

    return {binary, lightblobs_positive, lightblobs_negative, armours};
}

int main()
{
    video_controller video("./videos/output3.avi");

    rm::parameters red;
    red.extraction_lower_bound = 80;
    red.lightblob_tilt_max = 70;
    red.lightblob_ratio = {1.5, 80};
    red.lightblob_area = {10, 99999};
    red.armour_angle_difference_max = 999;
    red.armour_shear_max = 999;
    red.armour_lenght_ratio_max = 0.01;

    // read svm.xml
    const cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load("svm.xml");

    bool exit = false, playing = false;
    while (!exit)
    {
        auto [binary, lightblobs_positive, lightblobs_negative, armours] =
            reconizer(video.frame, red, rm::CAMP_RED);

        std::vector<cv::Mat> icon_images;
        for (auto armour : armours)
        {
            cv::Mat tmp;
            rm::CalcRatio(video.frame, tmp, armour.icon, {80, 80});
            icon_images.push_back(tmp);
        }

        int i = 0;
        cv::Mat debug_icon_images(80, 80 * static_cast<int>(icon_images.size()), CV_8UC3);
        for (const auto& n : icon_images)
        {
            n.copyTo(debug_icon_images(cv::Rect(80 * i++, 0, 80, 80)));
        }

        cv::Mat debug;
        video.frame.copyTo(debug);
        rm::debug::DrawArmours(armours, debug, -1);
        rm::debug::draw_lightblobs(lightblobs_positive, lightblobs_negative, debug, -1);

        cv::Mat combined = cv::Mat::zeros(debug.rows + 80, debug.cols + binary.cols, debug.type());
        debug.copyTo(combined(cv::Rect(0, 0, debug.cols, debug.rows)));
        cvtColor(binary, binary, cv::COLOR_GRAY2BGR);
        binary.copyTo(combined(cv::Rect(debug.cols, 0, binary.cols, binary.rows)));

        if (i > 0)
            debug_icon_images.copyTo(combined(cv::Rect(0, debug.rows, debug_icon_images.cols, debug_icon_images.rows)));

        imshow("Debug", combined);

        if (playing) video.update(1);

        if (const auto key_press = cv::waitKey(0); key_press == 27) exit = true;
        else if (key_press == ' ') playing = !playing;
        else if (key_press == ']') video.update(+001);
        else if (key_press == '[') video.update(-001);
        else if (key_press == '\\')video.update(+100);
        else if (key_press == 'p') video.update(-100);
    }

    cv::destroyAllWindows();
    return 0;
}
