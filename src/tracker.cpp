#include "tracker.h"

// #include "opencv2/opencv.hpp"
// #include "eigen3/Eigen/Dense"
// #include "math.h"
#include <iostream>
// #include <vector>
#include <algorithm>

#include <stdio.h>
#include <string.h>


Tracker::Tracker() {
    id_ = 0;
}

cv::RotatedRect *bbox = NULL;

// //! IOU 계산 시 사용하는 변수 선언
// Eigen::MatrixXd trk_corners;
// Eigen::MatrixXd trk_R;
// Eigen::MatrixXd trk_result;
// Eigen::MatrixXd corners;
// Eigen::MatrixXd R;
// Eigen::MatrixXd result; 
// double result_x_array[8];
// double min_result_x;
// double result_y_array[8];
// double min_result_y;

// std::vector<point_xy> points1;
// std::vector<point_xy> points2;
// boost::geometry::model::polygon<point_xy> polygon1; 
// boost::geometry::model::polygon<point_xy> polygon2; 
// std::vector<polygon> union_output;
// std::vector<polygon> intersection_output;

// float union_area;
// float intersection_area;
// float iou;
// //!



typedef struct Point{
    double x;
    double y;
}Point;

typedef struct Line{
    Point p1;
    Point p2;
}Line;

float new_iou;

long long dist(const Point* p1, const Point* p2){
    return (long long)(p1->x - p2->x)*(p1->x - p2->x) + (long long)(p1->y - p2->y)*(p1->y - p2->y);
}

int ccw(const Point* p1, const Point* p2, const Point* p3){

    int cross_product = (p2->x - p1->x)*(p3->y - p1->y) - (p3->x - p1->x)*(p2->y - p1->y);

    if (cross_product > 0){
        return 1;
    }
    else if (cross_product < 0){
        return -1;
    }
    else{
        return 0;
    }
}

// right가 left의 반시계 방향에 있으면 true이다.
// true if right is counterclockwise to left.
Point p;
int SortComparator(const Point* left, const Point* right){
    int ret;
    int direction = ccw(&p, left, right);
    if(direction == 0){
        ret = (dist(&p, left) < dist(&p, right));
    }
    else if(direction == 1){
        ret = 1;
    }
    else{
        ret = 0;
    }
    return ret;
}

void QuickSort(Point* a, int lo, int hi){
    if(hi - lo <= 0){
        return;
    }

    // 현재 배열 범위의 중앙값을 피벗으로 선택한다.
    // Select the median as pivot in the current array range.
    Point pivot = a[lo + (hi-lo+1)/2];
    int i = lo, j = hi;

    // 정복 과정
    // Conquer process
    while(i <= j){
        // 피벗의 왼쪽에는 comparator(타겟, "피벗")을 만족하지 않는 인덱스를 선택 (i)
        // On the left side of the pivot, select an index that doesn't satisfy the comparator(target, "pivot"). (i)
        while(SortComparator(&a[i], &pivot)) i++;
        
        // 피벗의 오른쪽에는 comparator("피벗", 타겟)을 만족하지 않는 인덱스를 선택 (j)
        // On the right side of the pivot, select an index that doesn't satisfy the comparator("pivot", target). (j)
        while(SortComparator(&pivot, &a[j])) j--;
        // (i > j) 피벗의 왼쪽에는 모든 값이 피벗보다 작고 피벗의 오른쪽에는 모든 값이 피벗보다 큰 상태가 되었음.
        // (i > j) On the left side of the pivot, all values are smaller than the pivot, and on the right side of the pivot, all values are larger than the pivot.
        if(i > j){
            break;
        }

        // i번째 값은 피벗 보다 크고 j번째 값은 피벗보다 작으므로 두 값을 스왑한다.
        // The i-th value is larger than the pivot and the j-th value is smaller than the pivot, so swap the two values.
        Point temp = a[i];
        a[i] = a[j];
        a[j] = temp;

        // 인덱스 i를 1증가 시키고 인덱스 j를 1 감소 시켜서 탐색 범위를 안쪽으로 좁힌다.
        // Narrow the search inward by increasing index i by one and decreasing index j by one.
        i++;
        j--;
    }

    // 분할 과정
    // Divide process
    QuickSort(a, lo, j);
    QuickSort(a, i, hi);
}



// right가 left 보다 크면 true를 반환합니다.
// if right is greater than left, it is true
int LineComparator(Point left, Point right){
    int ret;
    if(left.x == right.x){
        ret = (left.y <= right.y);
    }
    else{
        ret = (left.x <= right.x);
    }
    return ret;
}

// 입력받은 Point 구조체의 값을 교환합니다.
// Exchanges the value of the input Point structure.
void swap(Point* p1, Point* p2){
    Point temp;
    temp = *p1;
    *p1 = *p2;
    *p2 = temp;
}

// 두 선분이 교차하면 1을 교차하지 않으면 0을 반환합니다.
// If the two segments intersect, they will return 1 if they do not intersect 0.
int LineIntersection(Line l1, Line l2){
    int ret;
    // l1을 기준으로 l2가 교차하는 지 확인한다.
    // Check if l2 intersects based on l1.
    int l1_l2 = ccw(&l1.p1, &l1.p2, &l2.p1) * ccw(&l1.p1, &l1.p2, &l2.p2);
    // l2를 기준으로 l1이 교차하는 지 확인한다.
    // Check if l1 intersects based on l2.
    int l2_l1 = ccw(&l2.p1, &l2.p2, &l1.p1) * ccw(&l2.p1, &l2.p2, &l1.p2);

    ret = (l1_l2 < 0) && (l2_l1 < 0);

    return ret;
}

int PolygonInOut(Point p, int num_vertex, Point* vertices){

    int ret = 0;

    // 마지막 꼭지점과 첫번째 꼭지점이 연결되어 있지 않다면 오류를 반환한다.
    // If the last vertex and the first vertex are not connected, an error is returned.
    if (vertices[0].x != vertices[num_vertex].x || vertices[0].y != vertices[num_vertex].y){
        printf("Last vertex and first vertex are not connected.");
        return -1;
    }

    for(int i = 0; i < num_vertex; ++i){
        
        // 점 p가 i번째 꼭지점과 i+1번째 꼭지점을 이은 선분 위에 있는 경우
        // If point p is on the line connecting the i and i + 1 vertices
        if( ccw(&vertices[i], &vertices[i+1], &p) == 0 ){
            int min_x = MIN(vertices[i].x, vertices[i+1].x);
            int max_x = MAX(vertices[i].x, vertices[i+1].x);
            int min_y = MIN(vertices[i].y, vertices[i+1].y);
            int max_y = MAX(vertices[i].y, vertices[i+1].y);

            // 점 p가 선분 내부의 범위에 있는 지 확인
            // Determine if point p is in range within line segment
            if(min_x <= p.x && p.x <= max_x && min_y <= p.y && p.y <= max_y){
                return 1;
            }
        }
        else{;}
    }

    // 다각형 외부에 임의의 점과 점 p를 연결한 선분을 만든다.
    // Create a line segment connecting a random point outside the polygon and point p.
    Point outside_point;
    outside_point.x = 1; outside_point.y = 1234567;
    Line l1;
    l1.p1 = outside_point;
    l1.p2 = p;

    // 앞에서 만든 선분과 다각형을 이루는 선분들이 교차되는 갯수가 센다.
    // Count the number of intersections between the previously created line segments and the polygonal segments.
    for(int i = 0; i < num_vertex; ++i){
        Line l2;
        l2.p1 = vertices[i];
        l2.p2 = vertices[i+1];
        ret += LineIntersection(l1, l2);
    }

    // 교차한 갯수가 짝수인지 홀수인지 확인한다.
    // Check if the number of crossings is even or odd.
    ret = ret % 2;
    return ret;
}

// (p1, p2)를 이은 직선과 (p3, p4)를 이은 직선의 교차점을 구하는 함수
// Function to get intersection point with line connecting points (p1, p2) and another line (p3, p4).
Point IntersectionPoint(const Point* p1, const Point* p2, const Point* p3, const Point* p4){

    Point ret;

    ret.x = ((p1->x*p2->y - p1->y*p2->x)*(p3->x - p4->x) - (p1->x - p2->x)*(p3->x*p4->y - p3->y*p4->x))/( (p1->x - p2->x)*(p3->y - p4->y) - (p1->y - p2->y)*(p3->x - p4->x) );

    ret.y = ((p1->x*p2->y - p1->y*p2->x)*(p3->y - p4->y) - (p1->y - p2->y)*(p3->x*p4->y - p3->y*p4->x)) / ( (p1->x - p2->x)*(p3->y - p4->y) - (p1->y - p2->y)*(p3->x - p4->x) );

    return ret;
}

// 다각형의 넓이를 구한다.
// find the area of a polygon
double GetPolygonArea(int num_points, Point* points){
        double ret = 0;
        int i, j;
        i = num_points - 1;
        for(j = 0; j < num_points; ++j){
            ret += points[i].x * points[j].y - points[j].x * points[i].y;
            i = j;
        }
        ret = ret < 0 ? -ret : ret;
        ret /= 2;

        return ret;
}


Point intersection_point[10];
double GetIntersection(int num1, Point* points1, int num2, Point* points2){
    
    double ret;
    Line l1, l2;

    int interection_num;
    

    // points1과 points2 각각을 반시계방향으로 정렬한다.
    // sort by counter clockwise point1 and points2.
    p = points1[0];
    QuickSort(points1, 1, num1-1);

    p = points2[0];
    QuickSort(points2, 1, num2-1);

    // 차례대로 점들을 이었을 때, 다각형이 만들어질 수 있도록 시작점을 마지막에 추가한다.
    // Add the starting point to the last in order to make polygon when connect points in order.
    points1[num1] = points1[0];
    points2[num2] = points2[0];

    // points1의 다각형 선분들과 points2의 다각형 선분들의 교차점을 구한다.
    // Find the intersection of the polygon segments of points1 and the polygon segments of points2.
    for(int i = 0; i < num1; ++i){
        l1.p1 = points1[i];
        l1.p2 = points1[i+1];
        for(int j = 0; j < num2; ++j){
            l2.p1 = points2[j];
            l2.p2 = points2[j+1];

            // 선분 l1과 l2가 교차한다면 교차점을 intersection_point에 저장한다.
            // If line segments l1 and l2 intersect, store the intersection at intersection_point.
            if(LineIntersection(l1, l2)){
                intersection_point[interection_num++] = 
                IntersectionPoint(&l1.p1, &l1.p2, &l2.p1, &l2.p2);
            }
        }
    }

    for(int i = 0; i < num1; ++i){
        if(PolygonInOut(points1[i], num2, points2)){
            intersection_point[interection_num++] = points1[i];
        }
    }
    for(int i = 0; i < num2; ++i){
        if(PolygonInOut(points2[i], num1, points1)){
            intersection_point[interection_num++] = points2[i];
        }
    }
    
    // restore
    points1[num1].x = 0;    points1[num1].y = 0;
    points2[num2].x = 0;    points2[num2].y = 0;

    p = intersection_point[0];
    QuickSort(intersection_point, 1, interection_num-1);

    // std::cout << " intersection_point : " << intersection_point << std::endl;
    // std::cout << " interection_num : " << interection_num << std::endl;

    ret = GetPolygonArea(interection_num, intersection_point); 
    return ret;
}

double GetIoU(int num1, Point* points1, int num2, Point* points2){
    
    double ret;
    double A, B; //, union_area, intersection_area;
    // int interection_num = 0;
    // memset(intersection_point, 0, sizeof(intersection_point));

    

    auto intersection_area_new = GetIntersection(num1, points1, num2, points2);
    A = GetPolygonArea(num1, points1);
    B = GetPolygonArea(num2, points2);
    auto union_area_new = A + B - intersection_area_new;

    // std::cout << "intersection : " << intersection_area_new << std::endl;
    // std::cout << "A : " << A << std::endl;
    // std::cout << "B : " << B << std::endl;
    // std::cout << "union : " << union_area_new << std::endl;

    ret = intersection_area_new / union_area_new;    

    intersection_area_new = 0;   
    union_area_new = 0;

    return ret;
}


float Tracker::CalculateIou(const My_RotatedRect& det, const Track& track) {
    My_RotatedRect bbox = track.GetStateAsBbox();

    Eigen::MatrixXd *trk_corners = new Eigen::MatrixXd;
    *trk_corners = Eigen::MatrixXd(4,2);
    (*trk_corners)(0,0) = bbox.center_x - (bbox.width -1) / 2  - bbox.center_x;
    (*trk_corners)(0,1) = bbox.center_y - (bbox.height -1) / 2 - bbox.center_y;
    (*trk_corners)(1,0) = bbox.center_x - (bbox.width -1) / 2  - bbox.center_x;
    (*trk_corners)(1,1) = bbox.center_y - (bbox.height -1) / 2 + bbox.height -1 - bbox.center_y;
    (*trk_corners)(2,0) = bbox.center_x - (bbox.width -1) / 2  + bbox.width -1  - bbox.center_x;
    (*trk_corners)(2,1) = bbox.center_y - (bbox.height -1) / 2 + bbox.height -1 - bbox.center_y;
    (*trk_corners)(3,0) = bbox.center_x - (bbox.width -1) / 2  + bbox.width -1  - bbox.center_x;
    (*trk_corners)(3,1) = bbox.center_y - (bbox.height -1) / 2 - bbox.center_y;

    Eigen::MatrixXd *trk_R = new Eigen::MatrixXd;
    *trk_R = Eigen::MatrixXd(2,2);
    (*trk_R)(0,0) =  cos(bbox.angle);
    (*trk_R)(0,1) = -sin(bbox.angle);
    (*trk_R)(1,0) =  sin(bbox.angle);
    (*trk_R)(1,1) =  cos(bbox.angle);

    Eigen::MatrixXd *trk_result = new Eigen::MatrixXd;
    *trk_result = (*trk_R) * (*trk_corners).transpose();
    (*trk_result)(0,0) += bbox.center_x;
    (*trk_result)(0,1) += bbox.center_x;
    (*trk_result)(0,2) += bbox.center_x;
    (*trk_result)(0,3) += bbox.center_x;
    (*trk_result)(1,0) += bbox.center_y;
    (*trk_result)(1,1) += bbox.center_y;
    (*trk_result)(1,2) += bbox.center_y;
    (*trk_result)(1,3) += bbox.center_y;

    Eigen::MatrixXd *corners = new Eigen::MatrixXd;
    *corners = Eigen::MatrixXd(4,2);
    (*corners)(0,0) = det.center_x - (det.width -1) / 2  - det.center_x;
    (*corners)(0,1) = det.center_y - (det.height -1) / 2 - det.center_y;
    (*corners)(1,0) = det.center_x - (det.width -1) / 2  - det.center_x;
    (*corners)(1,1) = det.center_y - (det.height -1) / 2 + det.height -1 - det.center_y;
    (*corners)(2,0) = det.center_x - (det.width -1) / 2  + det.width -1  - det.center_x;
    (*corners)(2,1) = det.center_y - (det.height -1) / 2 + det.height -1 - det.center_y;
    (*corners)(3,0) = det.center_x - (det.width -1) / 2  + det.width -1  - det.center_x;
    (*corners)(3,1) = det.center_y - (det.height -1) / 2 - det.center_y;

    Eigen::MatrixXd *R = new Eigen::MatrixXd;
    *R = Eigen::MatrixXd(2,2);
    (*R)(0,0) =  cos(det.angle);
    (*R)(0,1) = -sin(det.angle);
    (*R)(1,0) =  sin(det.angle);
    (*R)(1,1) =  cos(det.angle);

    Eigen::MatrixXd *result = new Eigen::MatrixXd; 
    *result = (*R) * (*corners).transpose();
    (*result)(0,0) += det.center_x;
    (*result)(0,1) += det.center_x;
    (*result)(0,2) += det.center_x;
    (*result)(0,3) += det.center_x;
    (*result)(1,0) += det.center_y;
    (*result)(1,1) += det.center_y;
    (*result)(1,2) += det.center_y;
    (*result)(1,3) += det.center_y;


    //! 음수 보정작업 //
    double *result_x_array = nullptr;
    result_x_array = new double[8];
    result_x_array[0] = (*result)(0,0);
    result_x_array[1] = (*result)(0,1);
    result_x_array[2] = (*result)(0,2);
    result_x_array[3] = (*result)(0,3);
    result_x_array[4] = (*trk_result)(0,0);
    result_x_array[5] = (*trk_result)(0,1);
    result_x_array[6] = (*trk_result)(0,2);
    result_x_array[7] = (*trk_result)(0,3);

    double *min_result_x = new double;
    *min_result_x = *std::min_element(result_x_array, result_x_array+8);
    // double result_x_array[8] = { (*result)(0,0), (*result)(0,1), (*result)(0,2), (*result)(0,3), (*trk_result)(0,0), (*trk_result)(0,1), (*trk_result)(0,2), (*trk_result)(0,3) };
    // double min_result_x = *std::min_element(result_x_array, result_x_array+8);
    if (*min_result_x < 0) {
        (*result)(0,0) -= *min_result_x;
        (*result)(0,1) -= *min_result_x;
        (*result)(0,2) -= *min_result_x;
        (*result)(0,3) -= *min_result_x;
        (*trk_result)(0,0) -= *min_result_x;
        (*trk_result)(0,1) -= *min_result_x;
        (*trk_result)(0,2) -= *min_result_x;
        (*trk_result)(0,3) -= *min_result_x;
    }

    double *result_y_array = nullptr;
    result_y_array = new double[8];
    result_y_array[0] = (*result)(1,0);
    result_y_array[1] = (*result)(1,1);
    result_y_array[2] = (*result)(1,2);
    result_y_array[3] = (*result)(1,3);
    result_y_array[4] = (*trk_result)(1,0);
    result_y_array[5] = (*trk_result)(1,1);
    result_y_array[6] = (*trk_result)(1,2);
    result_y_array[7] = (*trk_result)(1,3);

    double *min_result_y = new double;
    *min_result_y = *std::min_element(result_y_array, result_y_array+8);

    // double result_y_array[8] = { (*result)(1,0), (*result)(1,1), (*result)(1,2), (*result)(1,3), (*trk_result)(1,0), (*trk_result)(1,1), (*trk_result)(1,2), (*trk_result)(1,3) };
    // double min_result_y = *std::min_element(result_y_array, result_y_array+8);
    if (*min_result_y < 0) {
        (*result)(1,0) -= *min_result_y;
        (*result)(1,1) -= *min_result_y;
        (*result)(1,2) -= *min_result_y;
        (*result)(1,3) -= *min_result_y;
        (*trk_result)(1,0) -= *min_result_y;
        (*trk_result)(1,1) -= *min_result_y;
        (*trk_result)(1,2) -= *min_result_y;
        (*trk_result)(1,3) -= *min_result_y;
    }


    Point *points1 = nullptr;
    points1 = new Point[8];
    Point *points2 = nullptr;
    points2 = new Point[8];

    points1[0].x = int((*result)(0,0));   points1[0].y = int((*result)(1,0));
    points1[1].x = int((*result)(0,1));   points1[1].y = int((*result)(1,1));
    points1[2].x = int((*result)(0,2));   points1[2].y = int((*result)(1,2));
    points1[3].x = int((*result)(0,3));   points1[3].y = int((*result)(1,3));

    points2[0].x = int((*trk_result)(0,0));   points2[0].y = int((*trk_result)(1,0));
    points2[1].x = int((*trk_result)(0,1));   points2[1].y = int((*trk_result)(1,1));
    points2[2].x = int((*trk_result)(0,2));   points2[2].y = int((*trk_result)(1,2));
    points2[3].x = int((*trk_result)(0,3));   points2[3].y = int((*trk_result)(1,3));

    // std::cout << " x0 : " << points1[0].x << " x1 : " << points1[1].x << " x2 : " << points1[2].x << " x3 : " << points1[3].x << std::endl;
    // std::cout << " y0 : " << points1[0].y << " y1 : " << points1[1].y << " y2 : " << points1[2].y << " y3 : " << points1[3].y << std::endl;
    // std::cout << " tx0 : " << points2[0].x << " tx1 : " << points2[1].x << " tx2 : " << points2[2].x << " tx3 : " << points2[3].x << std::endl;
    // std::cout << " ty0 : " << points2[0].y << " ty1 : " << points2[1].y << " ty2 : " << points2[2].y << " ty3 : " << points2[3].y << std::endl;


    // GetIoU(4, points1, 4, points2);

    // printf("IoU : %lf\n", GetIoU(4, points1, 4, points2));
    // // new_iou = GetIoU(4, points1, 4, points2);
    // // if (new_iou > 1) {
    // //     new_iou = 0;
    // // }

    delete[] points1;
    delete[] points2;

    // // Create points to represent a 5x5 closed polygon.
    // std::vector<point_xy> *points1 = new std::vector<point_xy>;
    // (*points1) +=
    //   point_xy((*result)(0,0),(*result)(1,0)),
    //   point_xy((*result)(0,1),(*result)(1,1)),
    //   point_xy((*result)(0,2),(*result)(1,2)),
    //   point_xy((*result)(0,3),(*result)(1,3)),
    //   point_xy((*result)(0,0),(*result)(1,0))
    //   ;

    // std::vector<point_xy> *points2 = new std::vector<point_xy>;
    // (*points2) +=
    //   point_xy((*trk_result)(0,0),(*trk_result)(1,0)),
    //   point_xy((*trk_result)(0,1),(*trk_result)(1,1)),
    //   point_xy((*trk_result)(0,2),(*trk_result)(1,2)),
    //   point_xy((*trk_result)(0,3),(*trk_result)(1,3)),
    //   point_xy((*trk_result)(0,0),(*trk_result)(1,0))
    //   ;

    // boost::geometry::model::polygon<point_xy> *polygon1 = new boost::geometry::model::polygon<point_xy>; 
    // boost::geometry::model::polygon<point_xy> *polygon2 = new boost::geometry::model::polygon<point_xy>; ; 
    // boost::geometry::assign_points(*polygon1, *points1);
    // boost::geometry::assign_points(*polygon2, *points2);

    
    // std::vector<polygon> *union_output = new std::vector<polygon>;
    // boost::geometry::union_(*polygon1, *polygon2, *union_output);

    // std::vector<polygon> *intersection_output = new std::vector<polygon>;
    // boost::geometry::intersection(*polygon1, *polygon2, *intersection_output);

    // // double union_area = boost::geometry::area(union_output[0]);
    // // double intersection_area = boost::geometry::area(intersection_output[0]);
    // // auto iou = intersection_area / union_area;

    // float *union_area = new float;
    // float *intersection_area = new float;

    // BOOST_FOREACH(polygon const& p, *union_output)
    // {
    //     *union_area = boost::geometry::area(p);
    //     // std::cout << ": " << boost::geometry::area(p) << std::endl;
    // }

    // BOOST_FOREACH(polygon const& q, *intersection_output)
    // {
    //     *intersection_area = boost::geometry::area(q);
    //     // std::cout << ": " << boost::geometry::area(q) << std::endl;
    // }

    // float *iou = new float;
    // *iou = (*intersection_area) / (*union_area);
    // // std::cout << "  IOU : " << iou << "  intersect : " << intersection_area << "  union : " << union_area << std::endl;
    // if (*iou > 1) {
    //     *iou = 0;
    // }

    // //! 변수 초기화 //
    // // *intersection_area = 0;
    // // *union_area = 0;
    delete trk_corners;
    delete trk_R;
    delete trk_result;
    delete corners;
    delete R;
    delete result;
    delete[] result_x_array;
    delete min_result_x;
    delete[] result_y_array;
    delete min_result_y;
    // delete points1;
    // delete points2;
    // delete polygon1;
    // delete polygon2;
    // delete union_output;
    // delete intersection_output;
    // delete union_area;
    // delete intersection_area;

    // return 0.5;
    return GetIoU(4, points1, 4, points2);
}


void Tracker::HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
                                size_t nrows, size_t ncols,
                                std::vector<std::vector<float>>& association) {
    Matrix<float> matrix(nrows, ncols);
    // Initialize matrix with IOU values
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            // Multiply by -1 to find max cost
            if (iou_matrix[i][j] != 0) {
                matrix(i, j) = -iou_matrix[i][j];
            }
            else {
                // TODO: figure out why we have to assign value to get correct result
                matrix(i, j) = 1.0f;
            }
            // std::cout << "i : " << i << " j : " << j << " value : " << matrix(i, j) << std::endl;
        }
    }
    // std::cout << "======================================================================================================" << std::endl;
    // Apply Kuhn-Munkres algorithm to matrix.
    Munkres<float> m;
    m.solve(matrix);

    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
            // std::cout << "i : " << i << " j : " << j << " value : " << matrix(i, j) << std::endl;
        }
    }
}


void Tracker::AssociateDetectionsToTrackers(const std::vector<My_RotatedRect>& detection,
                                            std::map<int, Track>& tracks,
                                            std::map<int, My_RotatedRect>& matched,
                                            std::vector<My_RotatedRect>& unmatched_det,
                                            float iou_threshold) {

    // Set all detection as unmatched if no tracks existing
    if (tracks.empty()) {
        for (const auto& det : detection) {
            unmatched_det.push_back(det);
        }
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    // resize IOU matrix based on number of detection and tracks
    iou_matrix.resize(detection.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    // resize association matrix based on number of detection and tracks
    association.resize(detection.size(), std::vector<float>(tracks.size()));


    // row - detection, column - tracks
    for (size_t i = 0; i < detection.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {
            iou_matrix[i][j] = CalculateIou(detection[i], trk.second);
            j++;
        }
    }

    // Find association
    HungarianMatching(iou_matrix, detection.size(), tracks.size(), association);

    for (size_t i = 0; i < detection.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            if (0 == association[i][j]) {
                // Filter out matched with low IOU
                if (iou_matrix[i][j] >= iou_threshold) {
                    matched[trk.first] = detection[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            j++;
        }
        // if detection cannot match with any tracks
        if (!matched_flag) {
            unmatched_det.push_back(detection[i]);
        }
    }
}


void Tracker::Run(const std::vector<My_RotatedRect>& detections) {

    /*** Predict internal tracks from previous frame ***/
    for (auto &track : tracks_) {
        track.second.Predict();
    }

    // Hash-map between track ID and associated detection bounding box
    std::map<int, My_RotatedRect> matched;
    // vector of unassociated detections
    std::vector<My_RotatedRect> unmatched_det;

    // return values - matched, unmatched_det
    if (!detections.empty()) {
        AssociateDetectionsToTrackers(detections, tracks_, matched, unmatched_det);
    }

    /*** Update tracks with associated bbox ***/
    for (const auto &match : matched) {
        const auto &ID = match.first;
        tracks_[ID].Update(match.second);
    }

    /*** Create new tracks for unmatched detections ***/
    for (const auto &det : unmatched_det) {
        Track tracker;
        tracker.Init(det);
        // Create new track and generate new ID
        if (id_ <= 3) {
            tracks_[id_++] = tracker;
        }
        
    }

    // /*** Delete lose tracked tracks ***/
    // for (auto it = tracks_.begin(); it != tracks_.end();) {
    //     if (it->second.coast_cycles_ > kMaxCoastCycles) {
    //         it = tracks_.erase(it);
    //     } else {
    //         it++;
    //     }
    // }
}


std::map<int, Track> Tracker::GetTracks() {
    return tracks_;
}
