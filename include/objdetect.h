//
// Created by yaione on 3/1/2022.
//

#ifndef RMCV_OBJDETECT_H
#define RMCV_OBJDETECT_H

#include "core/core.h"

namespace rm {
    /// Match a contour if it's a light blob with the given condition.
    /// \param contour    Input contour (more than 6 points).
    /// \param minRatio   Minimal aspect ratio.
    /// \param maxRatio   Maximal aspect ratio.
    /// \param tiltAngle  Maximal tilt angle.
    /// \param minArea    Minimal contour area.
    /// \param maxArea    Maximal contour area.
    /// \param fitEllipse Use cv::fitEllipseDirect to define light blob outlines instead of cv::minAreaRect when set
    ///                   to true. Usually recommended when source frame was over exposed.
    /// \return True if condition matches.
    bool MatchLightBlob(const rm::Contour &contour, float minRatio, float maxRatio, float tiltAngle, float minArea,
                        float maxArea, cv::RotatedRect &lightBlobBox, bool fitEllipse = true);

    /// Find light blobs from a set of contours. Auto detect light blob camp.
    /// \param contours   Input contour (more than 6 points).
    /// \param lightBlobs Output light blobs.
    /// \param minRatio   Minimal aspect ratio.
    /// \param maxRatio   Maximal aspect ratio.
    /// \param tiltAngle  Maximal tilt angle.
    /// \param minArea    Minimal contour area.
    /// \param maxArea    Maximal contour area.
    /// \param source     Source image (3 channels RGB).
    /// \param fitEllipse Use cv::fitEllipseDirect to box light blob instead of cv::minAreaRect when True.
    void FindLightBlobs(std::vector<rm::Contour> &contours, std::vector<rm::LightBlob> &lightBlobs, float minRatio,
                        float maxRatio, float tiltAngle, float minArea, float maxArea, const cv::Mat &source,
                        bool fitEllipse = true);

    /// Find light blobs from a set of contours.
    /// \param contours   Input contour (more than 6 points).
    /// \param lightBlobs Output light blobs.
    /// \param minRatio   Minimal aspect ratio.
    /// \param maxRatio   Maximal aspect ratio.
    /// \param tiltAngle  Maximal tilt angle.
    /// \param minArea    Minimal contour area.
    /// \param maxArea    Maximal contour area.
    /// \param camp       Light blobs camp.
    /// \param fitEllipse Use cv::fitEllipseDirect to box light blob instead of cv::minAreaRect when True.
    void FindLightBlobs(std::vector<rm::Contour> &contours, std::vector<rm::LightBlob> &lightBlobs, float minRatio,
                        float maxRatio, float tiltAngle, float minArea, float maxArea, rm::CampType camp,
                        bool fitEllipse = true);

    /// Find guid light
    /// \param lightBlobs
    /// \param source
    /// \return
    int FindGuidLight(const std::vector<rm::LightBlob> &lightBlobs, const cv::Mat &source);

    /// Detect if there is a overlap over all light blobs at the given pair of light blobs.
    /// \param lightBlobs All light blobs on the frame (must be sorted ascending by x).
    /// \param leftIndex  Index of the left light blob.
    /// \param rightIndex Index of the right light blob.
    /// \return True if there is a overlap;
    bool LightBlobOverlap(std::vector<rm::LightBlob> &lightBlobs, int leftIndex, int rightIndex);

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
                    float errAngle, float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType enemy,
                    bool filter = true);
}

#endif //RMCV_OBJDETECT_H
