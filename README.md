## Introduction
회전된 상자의 IOU를 계산하여 SORT + Hungarian 알고리즘을 접목시킨 방법

Kuhn-Munkres (Hungarian) Algorithm in C++ is forked from:
https://github.com/saebyn/munkres-cpp

## Dependencies
- Jetson Xavier
- OpenCV 4.2 up

## Docker run
```bash
#!/bin/bash
sudo xhost +local:root

sudo docker run --name rsort \
--gpus all \
--net=host \
--privileged \
--ipc=host \
-it intflow/nvidia-odtk:3090_Rsort bash
```

## Install Library
```bash
# apt-get install libboost-all-dev -y # boost를 이용한 IOU 계산법이 아닌 다른 계산법 적용
apt-get install libeigen3-dev # 회전된 box의 좌표값을 행렬식으로 구하기 위한 라이브러리
apt-get install build-essential gdb # Cmakefile Debug를 위한 라이브러리
```

## 사용방법
- main.cpp 참고

```c++
Tracker tracker; // Tracker 선언

// INPUT 데이터의 형식은 다음과 같음
// bbox_per_frame 에 emplace_back으로 데이터 making
struct My_RotatedRect {
    float center_x; // Left X 좌표 (추후 INPUT data가 고정 될 경우 명칭 변경)
    float center_y; // Top Y 좌표 (추후 INPUT data가 고정 될 경우 명칭 변경)
    float width; // width
    float height; // height
    float angle; // angle -90 ~ +90
    float landmarksX1; // 계승해야 하는 좌표
    float landmarksY1; // ''
};

// Tracker 추적
tracker.Run(bbox_per_frame);

// Tracker number 얻는 방법
const auto tracks = tracker.GetTracks();
for (auto &trk : tracks) {
    // box 정보를 추출 : My_RotatedRect 형태
    const auto &bbox = trk.second.GetStateAsBbox();

    if (trk.second.coast_cycles_ < kMaxCoastCycles
    && (trk.second.hit_streak_ >= kMinHits || frame_index < kMinHits)) {
        // Print to terminal for debugging
        std::cout << frame_index << "," << trk.first << "," << bbox.center_x << "," << bbox.center_y
                    << "," << bbox.width << "," << bbox.height << "," << bbox.angle << "," << bbox.landmarksX1 << "," << bbox.landmarksY1
                    << " Hit Streak = " << trk.second.hit_streak_
                    << " Coast Cycles = " << trk.second.coast_cycles_ << std::endl;
    }
    // trk.first : tracker number
    // trk.second.hit_streak_ : 연속으로 추적한 횟수
    // trk.second.coast_cycles_ : 연속으로 추적 못한 횟수

```



## References
1. https://github.com/abewley/sort
2. https://github.com/mcximing/sort-cpp
3. https://github.com/saebyn/munkres-cpp
