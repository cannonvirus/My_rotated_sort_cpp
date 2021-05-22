/**
 * SORT: A Simple, Online and Realtime Tracker
 */

#include <iostream>
#include <fstream>
#include <map>
#include <random>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "tracker.h"
#include "utils.h"



int main(int argc, const char *argv[]) {
    Tracker tracker;

    for (size_t i = 0; i >= 0; i++) {
        auto frame_index = i + 1;
        std::cout << "************* NEW FRAME ************* " << std::endl;

        /*** Run SORT tracker ***/
        // const auto &detections = all_detections[i];
        std::vector<My_RotatedRect> bbox_per_frame;

        for (float i=0; i<5; i++) {
            // bbox_per_frame.emplace_back(cv::Point2f(30*i, 30*i), cv::Point2f(20*i, 40*i), i/10);
            bbox_per_frame.emplace_back(My_RotatedRect{10*i, 20*i, 30*i, 40*i, i/10, 25*i, 25*i});
        }

        tracker.Run(bbox_per_frame);
        const auto tracks = tracker.GetTracks();

        for (auto &trk : tracks) {
            const auto &bbox = trk.second.GetStateAsBbox();

            if (trk.second.coast_cycles_ < kMaxCoastCycles
            && (trk.second.hit_streak_ >= kMinHits || frame_index < kMinHits)) {
                // Print to terminal for debugging
                std::cout << frame_index << "," << trk.first << "," << bbox.center_x << "," << bbox.center_y
                            << "," << bbox.width << "," << bbox.height << "," << bbox.angle << "," << bbox.landmarksX1 << "," << bbox.landmarksY1
                            << " Hit Streak = " << trk.second.hit_streak_
                            << " Coast Cycles = " << trk.second.coast_cycles_ << std::endl;
            }
        }
    } // end of iterating all frames
    return 0;
}
