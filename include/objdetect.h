//
// Created by yaione on 3/1/2022.
//

#ifndef RMCV_OBJDETECT_H
#define RMCV_OBJDETECT_H

#include "core.h"

namespace rm
{
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
    bool MatchLightBlob(const rm::contour& contour, float minRatio, float maxRatio, float tiltAngle, float minArea,
                        float maxArea, cv::RotatedRect& lightBlobBox, bool fitEllipse = true);

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
    void FindLightBlobs(std::vector<contour>& contours, std::vector<lightblob>& lightBlobs, float minRatio,
                        float maxRatio, float tiltAngle, float minArea, float maxArea, const cv::Mat& source,
                        bool fitEllipse = true);

    /// Find light blobs from a set of contours.
    /// \param contours    Input contour (more than 6 points).
    /// \param positive    Output positive light blobs.
    /// \param negative    Output negative light blobs.
    /// \param tilt_max    Maximal tilt angle.
    /// \param ratio_range Aspect ratio range.
    /// \param area_range  Area range.
    /// \param enemy       Enemy camp.
    void filter_lightblobs(const std::vector<contour>& contours, std::vector<lightblob>& positive,
                           std::vector<contour>& negative, float tilt_max, range<float> ratio_range,
                           range<double> area_range, color_camp enemy);

    /// Find guid light
    /// \param lightBlobs
    /// \param source
    /// \return
    int FindGuidLight(const std::vector<rm::lightblob>& lightBlobs, const cv::Mat& source);

    /// Detect if there is a overlap over all light blobs at the given pair of light blobs.
    /// \param lightBlobs All light blobs on the frame (must be sorted ascending by x).
    /// \param leftIndex  Index of the left light blob.
    /// \param rightIndex Index of the right light blob.
    /// \return True if there is a overlap;
    bool LightBlobOverlap(const std::vector<rm::lightblob>& lightBlobs, int leftIndex, int rightIndex);

    /// Fit armours from a set of light blobs.
    /// \param lightblobs           Input light blobs.
    /// \param armours              Output armours
    /// \param angle_difference_max Maximal angle difference between two light blobs.
    /// \param shear_max            Maximal shear between two light blobs with the rectangle.
    /// \param lenght_ratio_max     Maximal length ratio between two light blobs.
    /// \param enemy                Enemy camp.
    void filter_armours(std::vector<lightblob>& lightblobs, std::vector<armour>& armours,
                        float angle_difference_max, float shear_max, float lenght_ratio_max, color_camp enemy);
}

#endif //RMCV_OBJDETECT_H
