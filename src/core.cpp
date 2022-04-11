//
// Created by yaione on 2/26/2022.
//

#include "rmcv/core/core.h"

namespace rm {
    LightBar::LightBar(cv::RotatedRect box, rm::CampType camp) : angle(
            box.angle > 90 ? box.angle - 90 : box.angle + 90), camp(camp), center(box.center) {
        VerticesRectify(box, this->vertices, RECT_TALL);

        this->size = {min(box.size.height, box.size.width), max(box.size.height, box.size.width)};
    }

    Armour::Armour(std::vector<rm::LightBar> lightBars, float rank, rm::CampType camp, rm::ForceType force) : camp(
            camp), rank(rank), force(force) {
        if (lightBars.size() != 2) {
            throw std::runtime_error("Armour must be initialized with 2 rm::LightBar (s).");
        }

        // sort light bars left to right
        std::sort(lightBars.begin(), lightBars.end(), [](rm::LightBar lightBar1, rm::LightBar lightBar2) {
            return lightBar1.center.x < lightBar2.center.x;
        });

        int i = 0, j = 3;
        for (auto lightBar: lightBars) {
            vertices[i++] = lightBar.vertices[j--];
            vertices[i++] = lightBar.vertices[j--];
        }

        float distanceL = rm::PointDistance(vertices[0], vertices[1]);
        float distanceR = rm::PointDistance(vertices[2], vertices[3]);
        float offsetL = round((distanceL / 0.50f - distanceL) / 2);
        float offsetR = round((distanceR / 0.50f - distanceR) / 2);
        ExCord(vertices[0], vertices[1], offsetL, icon[0], icon[1]);
        ExCord(vertices[3], vertices[2], offsetR, icon[3], icon[2]);

        rm::CalcPerspective(vertices, vertices);
    }

    ShootFactor::ShootFactor(float pitchAngle, float yawAngle, double estimateAirTime) : pitchAngle(pitchAngle),
                                                                                         yawAngle(yawAngle),
                                                                                         estimateAirTime(
                                                                                                 estimateAirTime) {

    }

    Package::Package(rm::CampType camp, rm::AimMode mode, unsigned char speed, float pitch, const cv::Mat &inputFrame,
                     const cv::Mat &inputBinary) : camp(camp), mode(mode), speed(speed), pitch(pitch) {
        inputFrame.copyTo(this->frame);
        inputBinary.copyTo(this->binary);
    }

    Package::Package(const shared_ptr<rm::Package> &input) : camp(input->camp), mode(input->mode), speed(input->speed),
                                                             pitch(input->pitch) {
        input->frame.copyTo(this->frame);
        input->binary.copyTo(this->binary);
        this->armours = std::vector<rm::Armour>(input->armours);
    }
}
