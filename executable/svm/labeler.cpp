//
// Created by Sam Cheung on 7/20/24.
//

#include "include/rmcv.h"

class video_controller
{
    int index = 0;
    cv::VideoCapture capture;
    rm::range<int> frame_range;

public:
    cv::Mat frame;

    explicit video_controller(const std::string& VideoPath)
        : capture(VideoPath),
          frame_range(0, static_cast<int>(capture.get(cv::CAP_PROP_FRAME_COUNT)) - 1)
    {
        update(0);
    }

    void update(const int offset)
    {
        if (!frame_range.contains(index + offset))
            index = offset > 0 ? frame_range.lower_bound : frame_range.upper_bound;

        index = index + offset;
        capture.set(cv::CAP_PROP_POS_FRAMES, index);
        capture >> frame;
    }
};

int main()
{
    video_controller video("./videos/output.avi");

    rm::parameters red;
    red.lower_bound = {32, 32, 0};

    bool exit = false, playing = false;
    while (!exit)
    {
        cv::Mat binary;
        std::vector<rm::Contour> contours;
        std::vector<rm::LightBlob> lightBlobs;
        std::vector<rm::Armour> armours;

        inRange(video.frame, red.lower_bound, cv::Scalar(255, 255, 255), binary);
        morphologyEx(binary, binary, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));

        // ExtractColor(video.frame, binary, rm::CAMP_RED, 80, false, {5, 5});
        findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        FindLightBlobs(contours, lightBlobs, 2, 12, 25, 80, 1500, video.frame, false);
        FindArmour(lightBlobs, armours, 10, 24, 0.15, 0.45, 0.65, rm::CAMP_RED);

        std::vector<cv::Mat> num;
        for (auto armour : armours)
        {
            cv::Mat tmp;
            rm::CalcRatio(video.frame, tmp, armour.icon, {20, 20});
            num.push_back(tmp);
        }

        int i = 0;
        for (const auto& n : num)
        {
            imshow(std::to_string(i++), n);
            auto now = cv::getTickCount();
            imwrite(std::to_string(now) + ".png", n);
        }

        cv::Mat debug;
        video.frame.copyTo(debug);
        rm::debug::DrawArmours(armours, debug, -1);
        rm::debug::DrawLightBlobs(lightBlobs, debug, -1);
        drawContours(debug, contours, -1, cv::Scalar(0, 255, 0), 1);

        imshow("Frame", debug);
        imshow("Binary", binary);

        if (playing) video.update(1);

        if (const auto key_press = cv::waitKey(1); key_press == 27) exit = true;
        else if (key_press == ' ') playing = !playing;
        else if (key_press == ']') video.update(+01);
        else if (key_press == '[') video.update(-01);
        else if (key_press == '\\')video.update(+10);
        else if (key_press == 'p') video.update(-10);
        else if (key_press == '1') red.lower_bound[0]--;
        else if (key_press == '2') red.lower_bound[0]++;
        else if (key_press == '3') red.lower_bound[1]--;
        else if (key_press == '4') red.lower_bound[1]++;
        else if (key_press == '5') red.lower_bound[2]--;
        else if (key_press == '6') red.lower_bound[2]++;

        std::cout << "Lower Bound: " << red.lower_bound << std::endl;
    }

    cv::destroyAllWindows();
    return 0;
}
