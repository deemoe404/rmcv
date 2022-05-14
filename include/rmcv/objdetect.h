//
// Created by yaione on 3/1/2022.
//

#ifndef RM_STANDARD2022_OBJDETECT_H
#define RM_STANDARD2022_OBJDETECT_H

#include "core/core.h"

namespace rm {
    enum CompensateMode {
        COMPENSATE_NONE = 0,    // No compensate
        COMPENSATE_CLASSIC = 1, // Use rm::NewtonIteration & rm::ProjectileMotionFD for compensate
        COMPENSATE_NI = 2       // Use classic Newton's laws of motion for compensate
    };

    /// Judge a contour if it's a light blob with the given qualifications.
    /// \param contour       The contour.
    /// \param minRatio      Minimal aspect ratio.
    /// \param maxRatio      Maximal aspect ratio.
    /// \param tiltAngle     Maximal tilt angle.
    /// \param minArea       Minimal contour area.
    /// \param maxArea       Maximal contour area.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light blob outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    bool
    JudgeLightBlob(const std::vector<cv::Point> &contour, float minRatio, float maxRatio, float tilAngle, float minArea,
                   float maxArea, bool useFitEllipse = true);

    /// Fit light blobs from a set of contours with the given color. (Each contours must contains more than 6 points.)
    /// \param contours      The set of contours.
    /// \param lightBlobs     Output light blobs.
    /// \param minRatio      Minimal aspect ratio.
    /// \param maxRatio      Maximal aspect ratio.
    /// \param tiltAngle     Maximal tilt angle.
    /// \param minArea       Minimal contour area.
    /// \param maxArea       Maximal contour area.
    /// \param frame         Source frame. Used for auto color detection.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light blob outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    void
    FindLightBlobs(std::vector<std::vector<cv::Point>> &contours, std::vector<rm::LightBlob> &lightBlobs, float minRatio,
                   float maxRatio, float tiltAngle, float minArea, float maxArea, rm::CampType camp = rm::CAMP_RED,
                   bool useFitEllipse = true);

    /// Fit light blobs from a set of contours. Auto detect light blob color. (Each contours must contains more than 6
    /// points.)
    /// \param contours      The set of contours.
    /// \param lightBlobs     Output light blobs.
    /// \param minRatio      Minimal aspect ratio.
    /// \param maxRatio      Maximal aspect ratio.
    /// \param tiltAngle     Maximal tilt angle.
    /// \param minArea       Minimal contour area.
    /// \param maxArea       Maximal contour area.
    /// \param frame         Source frame. Used for auto color detection.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light blob outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    void
    FindLightBlobs(std::vector<std::vector<cv::Point>> &contours, std::vector<rm::LightBlob> &lightBlobs, float minRatio,
                   float maxRatio, float tiltAngle, float minArea, float maxArea, cv::Mat &frame,
                   bool useFitEllipse = true);

    /// Fit light blobs from a binary image. Auto detect light blob color. (Each contours must contains more than 6
    /// points.)
    /// \param binary        Source binary image. This function would consider the external contours only.
    /// \param lightBlobs     Output light blobs.
    /// \param minRatio      Minimal aspect ratio.
    /// \param maxRatio      Maximal aspect ratio.
    /// \param tiltAngle     Maximal tilt angle.
    /// \param minArea       Minimal contour area.
    /// \param maxArea       Maximal contour area.
    /// \param frame         Source frame. Used for auto color detection.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light blob outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    void FindLightBlobs(cv::Mat &binary, std::vector<rm::LightBlob> &lightBlobs, float minRatio, float maxRatio,
                        float tiltAngle, float minArea, float maxArea, cv::Mat &frame, bool useFitEllipse = true);

    /// Detect if there is a interference over all light blobs at the given pair of light blobs.
    /// \param lightBlobs  All light blobs on the frame.
    /// \param leftIndex  Index of the left light blob.
    /// \param rightIndex Index of the right light blob.
    /// \return True if there is a interference;
    bool LightBlobInterference(std::vector<rm::LightBlob> &lightBlobs, int leftIndex, int rightIndex);

    /// Fit armours from a set of light blobs.
    /// \param lightBlobs   The set of light blobs
    /// \param armours     Output armours
    /// \param maxAngleDif Maximum angle difference between two light blobs.
    /// \param errAngle    Maximum angle between the over all rect and the two light blobs.
    /// \param minBoxRatio Minimum ratio of the armour box.
    /// \param maxBoxRatio Maximum ratio of the armour box.
    /// \param lenRatio    Maximum length ratio between two light blobs.
    /// \param ownCamp     Own camp.
    /// \param frameSize   Frame size. When this parameter is specified, the distance between armour and the frame
    ///                    center would be used to sort the output.
    /// \param filter      Exclude one if two armours sharing a same light blob, exclude the armour witch minimum height
    ///                    along two light blobs is smaller.
    void FindArmour(std::vector<rm::LightBlob> &lightBlobs, std::vector<rm::Armour> &armours, float maxAngleDif,
                    float errAngle, float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType ownCamp,
                    cv::Size2f frameSize, bool filter = true);

    /// Fit armours from a set of light blobs.
    /// \param lightBlobs   The set of light blobs
    /// \param armours     Output armours
    /// \param maxAngleDif Maximum angle difference between two light blobs.
    /// \param errAngle    Maximum angle between the over all rect and the two light blobs.
    /// \param minBoxRatio Minimum ratio of the armour box.
    /// \param maxBoxRatio Maximum ratio of the armour box.
    /// \param lenRatio    Maximum length ratio between two light blobs.
    /// \param ownCamp     Own camp.
    /// \param attention   Attention point. When this parameter is specified, the distance between armour and the
    ///                    attention point would be used to sort the output.
    /// \param filter      Exclude one if two armours sharing a same light blob, exclude the armour witch minimum height
    ///                    along two light blobs is smaller.
    void FindArmour(std::vector<rm::LightBlob> &lightBlobs, std::vector<rm::Armour> &armours, float maxAngleDif,
                    float errAngle, float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType ownCamp,
                    cv::Point2f attention, bool filter = true);

    /// Calculate the shoot factor using Newton Iteration.
    /// \param target      The target armour.
    /// \param shootFactor Output shoot factor.
    /// \param g           Acceleration of gravity (m/s^2).
    /// \param v0          The initial speed of bullet (m/s).
    /// \param offset      Offset between camera and barrel (cm).
    /// \param deltaHeight Height difference between barrel and target (cm).
    void
    SolveShootFactor(cv::Mat &translationVector, rm::ShootFactor &shootFactor, double g, double v0, double deltaHeight,
                     cv::Point2f offset = {0, 0}, rm::CompensateMode mode = rm::COMPENSATE_NONE);

    void SolveCameraPose(cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &output);
}

#endif //RM_STANDARD2022_OBJDETECT_H
