#ifndef TYPEDEF_H
#define TYPEDEF_H

#endif // TYPEDEF_H

#define NUM_LAYER 16
#define NUM_ONE_ROTATE 2000


#define HDL_Grabber_toRadians(x) ((x) * PI / 180.0)
#define VLP_MAX_NUM_LASERS 16
#define VLP_DUAL_MODE 0x39
#define PUCK_LINE_POINT 12

#define HDL_NUM_ROT_ANGLES 36001
#define HDL_LASER_PER_FIRING 32
#define HDL_MAX_NUM_LASERS 64
#define HDL_FIRING_PER_PKT 12
#define HDL_PACKET_SIZE 1206
#define HDL_ROTATE_RATE 80 // msec
////****////

#define SIZE_GRID 0.2 //cell ũ��
#define X_METER 50.53 //80.9 //82.25 //�Ÿ� ���� (�ִ�)
#define Y_METER 20
#define Y_METER_LATERAL 40
#define X_SIZE 165 //161 //107
#define Y_SIZE 201 //Y_METER*2/SIZE_GRID //y���� �ε��� ��
//���� cell
#define CELL_STEP 0.01 //0.02 //0.05
#define CELL_MIN_SIZE 0.2

#define MAX_OBJ_N 400 //��ü �ִ� ��
#define NEIGHBOR_CELL_DIST 2
#define MAX_POINT_PER_SEG 5000 //�� segment�� point �� �ִ밪
#define MAX_CELL_PER_SEG 500 //�� segment�� cell �� �ִ밪
#define INFO_HEIGHT 3

#define LANDMARK_HEIGHT_HIGH 10
#define LANDMARK_HEIGHT_LOW 4

#define MAX_LANDMARK_N 100
#define MAX_POINT_PER_LANDMARK 2000

//���� ��ü ����
#define MIN_POINT_PER_SEG 4
#define TRACK_OBJ_N 32

//table ���� ground ���� //���� 0.1�϶� 15�� �˻�, ���Ʒ� 1.5�� �˻�
#define SLOPE_DEGREE_QUANTIZE 4 //0.25
#define SLOPE_DEGREE_RANGE 7
#define SEARCH_DEGREE 6 //�˻� ��
#define Z_TH 0.1
#define D_TH 3

//���� �� ����
#define ROAD_WIDTH_Q_SIZE 0.2
#define CURB_HEIGHT_LOW 0.15 //���� �ּ� ����
#define CURB_HEIGHT_HIGH 0.5 //���� �ִ� ����
#define NUM_LATERAL_POINTS 3000 //Ⱦ���� Ư¡�� �ִ� ��
#define SEARCH_RANGE_ROAD_BOUNDARY 1.0 //���� ���踦 ã�� ���� ���� ������ ���� ��ġ ���� 3.0

//������ ���� ����
#define NUM_RANDOMNUM_GEN 100 //random �Լ� ���� Ƚ��
#define TH_DIST_FOR_INLIER 0.1 //�ζ��̾� ���ϱ� ���� �Ӱ谪
#define TH_NUM_FOR_LINE 8 //���� ������ ���� �ּ� ����Ʈ ����
#define MAX_LINE_N 400

//��/Ⱦ���� Box ũ�� ����
#define MIN_TARGET_PTNUM_LATKEEPBOX 100
#define MIN_TARGET_PTNUM_LNGKEEPBOX 200
#define LNG_OBJ_ANGLE_TH_LATKEEPBOX 5
#define LAT_OBJ_WIDTH_LATKEEPBOX 1.5
#define LNG_OBJ_MIN_WIDTH_SEDAN_LATKEEPBOX 1.5
#define LNG_OBJ_MAX_WIDTH_SEDAN_LATKEEPBOX 5
#define LNG_OBJ_MAX_WIDTH_TRUCK_LATKEEPBOX 20
#define SEDAN_LAT_WIDTH 1.8
#define SEDAN_LNG_WIDTH 4.5
#define TRUCK_LAT_WIDTH 2.5
#define TRUCK_LNG_WIDTH 10

// ------------------- Multiple Traget Tracking ----------------------- //
#define STATE_NUM_PUCK       6       // ���� ���� ���� (x��ġ, x�ӵ�, x���ӵ�, y��ġ, y�ӵ�, y���ӵ�)
#define NOISE_POS_CV         0.08    // Constant Velocity ���� �ý��� ��ġ ���� [m]
#define NOISE_VEL_CV         0.3     // Constant Velocity ���� �ý��� �ӵ� ���� [m/s]
#define NOISE_ACC_CV         0.02    // Constant Velocity ���� �ý��� ��ġ ���� [m/s^2]
#define NOISE_POS_CA         0.08    // Constant Acceleration ���� �ý��� ��ġ ���� [m]
#define NOISE_VEL_CA         0.3     // Constant Acceleration ���� �ý��� �ӵ� ���� [m/s]
#define NOISE_ACC_CA         0.02    // Constant Acceleration ���� �ý��� ���ӵ� ���� [m/s^2]
#define MEAS_NUM_PUCK        2       // ���� ���� ���� (x��ġ, y��ġ)
#define NOISE_POS_MEAS       0.05    // ���� ������ ��ġ ���� [m]
#define TRACK_WINDOW_SIZE    10      // �� ���� ������ ������ ���ΰ� (10 == 1 sec, 100ms x 10)
#define EUCLIDEAN_DIST       2.0     // Validation Gate size (1.0m)
#define STATE_NUM_FUSION     6       // IMM �˰��������� �����Ǵ� ���� ���� ������ ����
#define IMM_MODE_NUM         2       // IMM �˰������� �����ϴ� �� ���� ���� (CV & CA)
#define MAX_TRACK_NUM        32      // ������ �ִ� ���� ����
#define SAMPLE_TIME_PUCK	 0.08	 // PUCK ������Ʈ �ֱ� (sec)

/*
// ------------------- Multiple Traget Tracking ----------------------- //

typedef struct _ObjectShowing
{
        float Zmax;
        float Zmin;
        float Xpoint1;
        float Ypoint1;
        float Xpoint2;
        float Ypoint2;
        float Xpoint3;
        float Ypoint3;
        float Xpoint4;
        float Ypoint4;
        int objInfo;
} ObjectShowing;

typedef struct _LandmarkShowing
{
        float Xmax;
        float Xmin;
        float Ymax;
        float Ymin;
        float Zmax;
        float Zmin;
} LandmarkShowing;

typedef struct _LineParam
{
        float X1;
        float X2;
        float Y1;
        float Y2;
} LineParam;

typedef struct _LongitudinalLine
{
        int lineNum;
        LineParam linePt[MAX_LINE_N];
} LongitudinalLine;

typedef struct _ObjectTracking
{
        unsigned int Id;
        unsigned int age;
        float x_pos;
        float y_pos;
        float lat_minP_x;
        float lat_minP_y;
        float lat_maxP_x;
        float lat_maxP_y;
        float pos_xb[4];
        float pos_yb[4];
        float min_x;
        float min_y;
        float max_x;
        float max_y;
        float width;
        float height;
        float rel_vx;
        float rel_vy;
        unsigned char stopflag; // 0:stop, 1:moving
} ObjectTracking;


*/
typedef struct _PUCK_DATA
{
        int m_chN;
        float m_calibX;
        float m_calibY;
        int m_sampleNum;
        int m_objectNumber;
        int m_landmarkNumber; //
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
        float point_lane_l[PUCK_LINE_POINT*4];
        float point_lane_r[PUCK_LINE_POINT*4];
        float point_lane_ll[PUCK_LINE_POINT*4];
        float point_lane_rr[PUCK_LINE_POINT*4];

        float m_roadBoundary[NUM_LATERAL_POINTS*3];

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

        int m_landmarkToPoint[MAX_LANDMARK_N*MAX_POINT_PER_LANDMARK]; //

        int m_numPointPerLandmark[MAX_LANDMARK_N]; //

        double m_g_p_c[X_SIZE*Y_SIZE];

        int m_numColumn;
        int m_numBank;

        double m_column_X[100];
        double m_column_Y[100];
        double m_bank_X[100];
        double m_bank_Y[100];

//        POLY_2ND roadside_2nd_param;

//        ObjectShowing m_objectShowing[MAX_OBJ_N];
//        ObjectTracking m_objectTracking[MAX_OBJ_N];
//        ObjectTracking m_TrackingResult[MAX_OBJ_N];

//        LongitudinalLine m_longitudinalLine;

//        POLY_2ND boundParam_2D;
}PUCK_DATA;

PUCK_DATA		puck_data1, puck_data2;
//POINT_LiDAR		LidarPuck1, LidarPuck2;


/*
// ------------------- Multiple Traget Tracking ----------------------- //
#pragma pack(push,1)
typedef struct _IMM_FUSION
{
    unsigned int id;                // Track ID
    unsigned int age;               // Track age
    unsigned int gnnID;             // Associated measurement ID
    unsigned int maxAge;            // Maximum previous age
    float modeProb[IMM_MODE_NUM];   // probability of each mode ( 0 - 1 )

//     state variables definition
//    * - X[0]: position on x-axis [m]
//    * - X[1]: velocity on x-axis [m]
//        * - X[2]: acceleration on x-axis [m]
//    * - X[3]: position on y-axis [m]
//    * - X[4]: velocity on y-axis [m]
//        * - X[5]: acceleration on y-axis [m]

    float x;
    float vx;
        float ax;
        float y;
        float vy;
    float ay;

    tMATRIX *P;
        tMATRIX *S;
}IMM_FUSION;

typedef struct _IMM_CV
{
    unsigned int id;        // Track ID
    unsigned int age;		// Track age
    unsigned int gnnID;		// Associated measurement ID
    unsigned int maxAge;	// Maximum previous age
    float modeProb;         // Mode probability ( 0 - 1 )

//     state variables definition
//    * - X[0]: position on x-axis [m]
//    * - X[1]: velocity on x-axis [m]
//        * - X[2]: acceleration on x-axis [m]
//    * - X[3]: position on y-axis [m]
//    * - X[4]: velocity on y-axis [m]
//        * - X[5]: acceleration on y-axis [m]

    float x;
    float vx;
        float ax;
        float y;
        float vy;
    float ay;

    tMATRIX *P;
    tMATRIX *S;
}IMM_CV;

typedef struct _IMM_CA
{
    unsigned int id;        // Track ID
    unsigned int age;		// Track age
    unsigned int gnnID;		// Associated measurement ID
    unsigned int maxAge;	// Maximum previous age
    float modeProb;         // Mode probability ( 0 - 1 )

//     state variables definition
//    * - X[0]: position on x-axis [m]
//    * - X[1]: velocity on x-axis [m]
//    * - X[2]: acceleration on x-axis [m]
//    * - X[3]: position on y-axis [m]
//    * - X[4]: velocity on y-axis [m]
//    * - X[5]: acceleration on y-axis [m]

    float x;
    float vx;
        float ax;
        float y;
        float vy;
    float ay;

    tMATRIX *P;
    tMATRIX *S;
}IMM_CA;

typedef struct _GNN_TRACK
{
    unsigned int    id;
    unsigned int    matched;
    unsigned int    trackAge;
    unsigned int    trackMaxAge;
    float           trackScore;
    float           pos_x;
    float           pos_y;
}GNN_TRACK;

typedef struct _GNN
{
    unsigned int	ageIMM;
    unsigned int	numTrack;           // �ش� scan�� �������� ��Ī�� ���� ����
    unsigned int	numTrack_prev;
    unsigned int	numNewTrack;        // �ش� scan�� �ű� ������ ���� ����
    unsigned int	numObservation;     // �ش� scan�� ȹ���� ���� ������ ����
    unsigned int    numTotalTrack;      // �ش� scan�� ������ ��ü ���� ����

        GNN_TRACK	gnnTrack[MAX_TRACK_NUM], gnnTrack_prev[MAX_TRACK_NUM];
        IMM_FUSION  immFusion[MAX_TRACK_NUM], immFusion_prev[MAX_TRACK_NUM];
        IMM_CV      immCV[MAX_TRACK_NUM], immCV_prev[MAX_TRACK_NUM];
        IMM_CA      immCA[MAX_TRACK_NUM], immCA_prev[MAX_TRACK_NUM];
}GNN;

typedef struct _LINESEG_PUCK
{
        double intercept;
        double angle;

        double pre_intercept;
        double pre_angle;
}LINESEG_PUCK;

#pragma pack(pop)

void PuckPosFeatureAlgorithm(POINT_LiDAR *LidarPuck, PUCK_DATA *puck_data, int dataLength, int chN);
void PuckObjectAlgorithm(PUCK_DATA *puck_data);
void SetCalibLidar(float *cellDist, float *cellAngle, int chN);
void PreProcessing(PUCK_DATA *puck_data);
void FreeSpaceDetection(PUCK_DATA *puck_data);
void GroundSubtraction(PUCK_DATA *puck_data);
void ObjectSegmentation(PUCK_DATA *puck_data);
void CurbDetection(PUCK_DATA *puck_data);
void ColumnDetection(PUCK_DATA *puck_data);
int LateralRoadFeatureExtraction(PUCK_DATA *puck_data);
void ObjectClustering(PUCK_DATA *puck_data);
int LandmarkClustering(PUCK_DATA *puck_data);
void LongitudinalLineExtration(PUCK_DATA *puck_data);
void MakingBox(PUCK_DATA *puck_data);

int	FittingCurve2D(int no_point, _2D_POSITION *point_vector, POLY_2ND *poly_curve);
int	FittingCurve3D(int no_point, _2D_POSITION *point_vector, POLY_3RD *poly_curve);
// ------------------- Multiple Traget Tracking ----------------------- //
extern GNN trackState1,trackState2;		// IMM_GNN_State
extern int initializeMTT1,initializeMTT2;	// Multiple Target Track initialization flag

*/

//// Constant Velocity mode
//tMATRIX  *F_CV; /**< the state-transition matrix */
//tMATRIX  *G_CV; /**< the process input noise */
//tMATRIX  *Q_CV; /**< the covariance of the process noise */
//tMATRIX  *H_CV; /**< the observation model */
//tMATRIX  *R_CV; /**< the covariance of the observation noise */

//// Constant Acceleration mode
//tMATRIX  *F_CA; /**< the state-transition matrix */
//tMATRIX  *G_CA; /**< the process input noise */
//tMATRIX  *Q_CA; /**< the covariance of the process noise */
//tMATRIX  *H_CA; /**< the observation model */
//tMATRIX  *R_CA; /**< the covariance of the observation noise */

//tMATRIX *MP;
//tMATRIX *X_CV_hat;
//tMATRIX *X_CA_hat;
//tMATRIX *P_CV_hat;
//tMATRIX *P_CA_hat;
//tMATRIX *Z; // measurement vector
//tMATRIX *V; // residual(=innovation) vector
//tMATRIX *S; // residual covariance
//tMATRIX *K_CV; // Kalman Gain of CV mode
//tMATRIX *K_CA; // Kalman Gain of CA mode
//tMATRIX *I_CV; // Identity matrix of CV mode
//tMATRIX *I_CA; // Identity matrix of CA mode
//tMATRIX *P_CV_temp;
//tMATRIX *P_CA_temp;
//tMATRIX *P_FUSION;

//tMATRIX *X_temp;
//tMATRIX *add_temp; // temporal matrix for add operation
//tMATRIX *diff_temp; // temporal matrix for difference operation
//tMATRIX *trans_temp; // temporal matrix for transpose operation
//tMATRIX *mul_temp; // temporal matrix for matrix multiple operation
//tMATRIX *multiple_temp; // temporal matrix for multiple operation





//void InitializeMultiTargetTracking();	// ���� �ʱ�ȭ
//void ReleaseMultiTargetTracking();		// ���� ����
//void GlobalNearestNeighbor(PUCK_DATA *data, GNN *state, int *flag);	// Data association
//void IMMTracker(PUCK_DATA *data, GNN *state, int *flag);				// Interacting Multiple Model Tracker
//void InitCovarianceMatrix(tMATRIX* m);
//void InitResidualCovarianceMatrix(tMATRIX* m);
//float GaussRandom (float mu, float sigma);


//// ---------------------  Puck Lane Detection  ----------------------- //
//void PuckLaneDetection_right(PUCK_DATA *puck_data);
//void PuckLaneDetection_left(PUCK_DATA *puck_data);



//typedef struct TARGET_INFO
//{
//        int						target_id;

//        _2D_POSITION		target_pos;
//        _2D_POSITION		target_vel;
//        _2D_POSITION		target_acc;
//        double				target_width;

//        eClass				target_class;

//        int						large_width;


//        unsigned int		id;				// Object ID
//        unsigned int		age;			// Object Age
//        unsigned int		status;			//
//        double			rssi;			// ���� �Ŀ�
//        unsigned int radar_flag;
//        double	CIPV_flag;
//        int obstacle_valid;

//        double max_x;
//        double min_x;
//        double max_y;
//        double min_y;
//        double max_z;
//        double min_z;
//        double width;
//        double height;

//        //OnComingFlag	oncoming;
//        //MovingFlag		MFlag;
//        double TTC;
//        int cutIn;
//}_TARGET_INFO;
