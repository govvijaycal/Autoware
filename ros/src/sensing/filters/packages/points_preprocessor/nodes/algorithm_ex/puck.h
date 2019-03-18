#ifndef PUCK_H
#define PUCK_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/GridCells.h"
#include "nav_msgs/OccupancyGrid.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include "autoware_msgs/CloudCluster.h"
#include <pcl/point_types.h>
#include <opencv/cv.h>

#include "autoware_msgs/ConfigRingGroundFilter.h"

enum Label
{
    GROUND,
    VERTICAL,
    UNKNOWN //Initial state, not classified
};

namespace velodyne_pointcloud
{
// shorter names for point cloud types in this namespace
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

class PuckAlgorithm
{
public:

    PuckAlgorithm(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~PuckAlgorithm() {}

private:

    //void convertPoints(const VPointCloud::ConstPtr &inMsg);
    void preprocessing16(const VPointCloud::ConstPtr &inMsg);
    void preprocessing32(const VPointCloud::ConstPtr &inMsg);
    void process_cell(const VPointCloud::ConstPtr &inMsg);
    void curbdetection(const VPointCloud::ConstPtr &inMsg);
//    void objectdetection(const jsk_recognition_msgs::BoundingBox &inboundingbox); //, autoware_msgs::CloudCluster &inclusters);

    // Ring ground filter
    //void groundsubstraction(const VPointCloud::ConstPtr &inMsg);

    ros::Subscriber input_2;
    ros::Subscriber input_16;
    ros::Subscriber input_32;
    ros::Subscriber input_;
    ros::Subscriber input_1;
    ros::Subscriber input_lane_l;
    ros::Subscriber input_lane_r;
    ros::Subscriber bounding_box;
    ros::Subscriber clusters_message;

    ros::Publisher output_1;
    ros::Publisher output_16;
    ros::Publisher output_32;
    ros::Publisher output_curb;

    // ros::NodeHandle node_handle_;
    ros::Subscriber config_node_sub_;
    ros::Subscriber points_node_sub_;
    ros::Subscriber config_node_sub_16;
    ros::Subscriber points_node_sub_16;
    ros::Publisher groundless_points_pub_16;
    ros::Publisher groundless_points_pub_32;
    ros::Publisher ground_points_pub_16;
    ros::Publisher ground_points_pub_32;
    ros::Publisher groundless_points_pub_;
    ros::Publisher ground_points_pub_;

    std::string point_topic_;
    std::string point_topic_16;
    int 		sensor_model_;
    double 		sensor_height_;
    double 		max_slope_;
    int 		min_point_;
    double 		clipping_thres_;
    double 		gap_thres_;
    double		point_distance_;
    bool		floor_removal_;
    int 		vertical_res_;
    int 		horizontal_res_;

    double 		limiting_ratio_;
    cv::Mat 	index_map_;
    Label 		class_label_[64];

    int 		sensor_model_16;
    double 		sensor_height_16;
    int         horizontal_res_16;
    double 		max_slope_16;
    int 		min_point_16;
    double 		clipping_thres_16;
    double 		gap_thres_16;
    double		point_distance_16;
    bool		floor_removal_16;

    int 		vertical_res_16;
    double 		limiting_ratio_16;
    cv::Mat 	index_map_16;
    Label 		class_label_16[64];

    cv::Mat 	index_map_vel;
    cv::Mat 	index_map_curb;

    int horizontal_res_curb;
    int vertical_res_curb;

//    //VLP_16
//    sensor_model_16= 16;
//    sensor_height_16= 0.65;
//    floor_removal_16= true;
//    max_slope_16= 20.0;
//    point_distance_16= 0.05;
//    min_point_16= 3;
//    clipping_thres_16 = 3.0;
//    // gap_thres : lower parameter can detect curb : 0.15 ~ 0.2 is enough detecting curb
//    gap_thres_16= 0.15;
//    //HDL 32

//    sensor_model_= 32;
//    sensor_height_= 1.3;
//    floor_removal_= true;
//    max_slope_= 20.0;
//    point_distance_= 0.05;
//    min_point_= 3;
//    clipping_thres_ = 3.0;
//    // gap_thres : lower parameter can detect curb : 0.15 ~ 0.2 is enough detecting curb
//    gap_thres_= 0.15;



    boost::chrono::high_resolution_clock::time_point t1_;
    boost::chrono::high_resolution_clock::time_point t2_;
    boost::chrono::nanoseconds elap_time_;

    const int 	DEFAULT_HOR_RES = 2000;

    void SetHorizontalRes(const int sensor_model, int &horizontal_res);
    void InitLabelArray(int in_model);
    void InitLabelArray_16(int in_model);
    void InitDepthMap(int in_width);
    void InitDepthMap_16(int in_width);
    void PublishPointCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
                           int in_indices[], int &in_out_index_size,
                           pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &in_cloud);
    void ConfigCallback(const autoware_msgs::ConfigRingGroundFilterConstPtr &config);
    void ConfigCallback16(const autoware_msgs::ConfigRingGroundFilterConstPtr &config);
    void VelodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg);
    void VelodyneCallback_16(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg);
    void FilterGround(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
                      pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_groundless_points,
                      pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_ground_points);
    void FilterGround_16(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
                         pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_groundless_points,
                         pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_ground_points);

    //    void ground_filter16(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg);
};

}

#define				PI						3.14159265358979
#define SIZE_GRID 0.2 //cell ũ��
#define X_METER 50.53 //80.9 //82.25 //�Ÿ� ���� (�ִ�)
#define Y_METER 20
#define Y_METER_LATERAL 40
#define X_SIZE 165 //161 //107
#define Y_SIZE 201 //Y_METER*2/SIZE_GRID //y���� �ε��� ��
// cell
#define CELL_STEP 0.01 //0.02 //0.05
#define CELL_MIN_SIZE 0.2

#define NUM_LAYER_VLP 16
#define NUM_LAYER 32

//Ground substraction from table //���� 0.1�϶� 15�� �˻�, ���Ʒ� 1.5�� �˻�
#define SLOPE_DEGREE_QUANTIZE 4 //0.25
#define SLOPE_DEGREE_RANGE 7
#define SEARCH_DEGREE 6 //�˻� ��
#define Z_TH 0.1
#define D_TH 3

#define NUM_ONE_ROTATE 2000

#define PUCK_T_X 0
#define PUCK_T_Y 0
#define PUCK_T_Z 0   // the height of puck
#define PUCK_R 0
#define NUM_ONE_ROTATE 2000

//#define PUCK_LEFT_T_X -4.02
//#define PUCK_LEFT_T_Y 1.06+0.238
//#define PUCK_LEFT_T_Z 0.947
//#define PUCK_LEFT_R -89.3
//#define PUCK_LEFT_TILT_DEGREE1 0.0
//#define PUCK_LEFT_TILT_DIRECTION1 117.9
//#define PUCK_LEFT_TILT_DEGREE2 0.0
//#define PUCK_LEFT_TILT_DIRECTION2 207.9

typedef struct _Puck_layer
{
    int pInfo;

}Puck_layer;

typedef struct _PUCK_DATA
{
    int m_chN;
    float m_calibX;
    float m_calibY;
    int m_sampleNum;
    int m_objectNumber;
    int m_landmarkNumber;
    int m_lineFeatureNumber;
    int m_targetObjectNumber;
    float m_roadDist;
    int m_lateralPointNumber;

    int m_valuablePointNumber;
    int m_valuablePointNumber_update;

    float lane_slope_l;
    float lane_intercept_l;
    float lane_slope_r;
    float lane_intercept_r;
    float lane_slope_ll;
    float lane_intercept_ll;
    float lane_slope_rr;
    float lane_intercept_rr;

    float prelane_intercept_l;
    float prelane_intercept_r;

    float roadside_slope_lateral;
    float roadside_intercept_lateral;
    float preRoadside_slope_lateral;
    float preRoadside_intercept_lateral;

    float longline_slope_lateral;
    float longline_intercept_lateral;

    float wall_slope_lateral;
    float wall_intercept_lateral;

    float curb_slope_lateral;
    float curb_intercept_lateral;

    float m_velo[NUM_LAYER*NUM_ONE_ROTATE*3];
    float intensity[NUM_LAYER*NUM_ONE_ROTATE];
    float rho[NUM_LAYER*NUM_ONE_ROTATE];
    float theta_rad[NUM_LAYER*NUM_ONE_ROTATE];
    //  float point_lane_l[PUCK_LINE_POINT*4];
    //  float point_lane_r[PUCK_LINE_POINT*4];
    //  float point_lane_ll[PUCK_LINE_POINT*4];
    //  float point_lane_rr[PUCK_LINE_POINT*4];

    //  float m_roadBoundary[NUM_LATERAL_POINTS*3];

    unsigned char m_sample_checker[NUM_ONE_ROTATE];

    unsigned char m_groundPt[NUM_LAYER*NUM_ONE_ROTATE];
    unsigned char m_latLocPt[NUM_LAYER*NUM_ONE_ROTATE];
    unsigned char m_curbPt[NUM_LAYER*NUM_ONE_ROTATE];
    unsigned char m_endPt[NUM_LAYER*NUM_ONE_ROTATE];
    unsigned char m_wallPt[NUM_LAYER*NUM_ONE_ROTATE];
    unsigned char m_columnPt[NUM_LAYER*NUM_ONE_ROTATE];

    float m_grid_min[X_SIZE*Y_SIZE];
    int m_grid_landmark[X_SIZE*Y_SIZE]; //
    unsigned char m_object[X_SIZE*Y_SIZE];
    unsigned char m_landmark[X_SIZE*Y_SIZE]; //
    float m_cellDist[X_SIZE*Y_SIZE];
    float m_cellAngle[X_SIZE*Y_SIZE];

    int m_cellToSegment[X_SIZE*Y_SIZE];
    int m_cellToLandmark[X_SIZE*Y_SIZE];

    int m_pointToCell[NUM_LAYER*NUM_ONE_ROTATE];
    int m_pointToSegment[NUM_LAYER*NUM_ONE_ROTATE];
    int m_pointToLandmark[NUM_LAYER*NUM_ONE_ROTATE]; //

    //  int m_landmarkToPoint[MAX_LANDMARK_N*MAX_POINT_PER_LANDMARK]; //

    //  int m_numPointPerLandmark[MAX_LANDMARK_N]; //

    double m_g_p_c[X_SIZE*Y_SIZE];

    int m_numColumn;
    int m_numBank;

    double m_column_X[100];
    double m_column_Y[100];
    double m_bank_X[100];
    double m_bank_Y[100];

    //  POLY_2ND roadside_2nd_param;

    //  ObjectShowing m_objectShowing[MAX_OBJ_N];
    //  ObjectTracking m_objectTracking[MAX_OBJ_N];
    //  ObjectTracking m_TrackingResult[MAX_OBJ_N];

    //  LongitudinalLine m_longitudinalLine;

    //  POLY_2ND boundParam_2D;
}PUCK_DATA;

//PUCK_DATA		puck_data1, puck_data2;

#endif // PUCK_H


//#include "typedef.h"

