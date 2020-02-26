#include "puck.h"

namespace velodyne_pointcloud
{
//typedef pcl::PointXYZI Point;
//typedef pcl::PointCloud<Point> PointCloud;

PuckAlgorithm::PuckAlgorithm(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    output_16 = node.advertise<sensor_msgs::PointCloud2>("velodyne_preprocess", 2);
    output_32 = node.advertise<sensor_msgs::PointCloud2>("velodyne_preprocess32", 2);
    // output_1 = node.advertise<nav_msgs::OccupancyGrid>("velodyne_cells", 10);
    output_1 = node.advertise<nav_msgs::GridCells>("velodyne_cells", 2);
    // output_1 = node.advertise<sensor_msgs::PointCloud2>("velodyne_rings", 10);
    output_curb = node.advertise<sensor_msgs::PointCloud2>("velodyne_curb", 2);

    // subscribe to VelodyneScan packets
    input_16 = node.subscribe("/rear_lidar/points_raw", 10, &PuckAlgorithm::preprocessing16, this,
                              ros::TransportHints().tcpNoDelay(true));

    input_32 = node.subscribe("/center_lidar/points_raw", 10, &PuckAlgorithm::preprocessing32, this,
                              ros::TransportHints().tcpNoDelay(true));

    input_2 = node.subscribe("/points_no_ground_32", 10, &PuckAlgorithm::process_cell, this,
                             ros::TransportHints().tcpNoDelay(true));

    input_1 = node.subscribe("/points_ground_32", 10, &PuckAlgorithm::curbdetection, this,
                             ros::TransportHints().tcpNoDelay(true));

//    input_lane_l = node.subscribe("/lane_left", 10, &PuckAlgorithm::objectdetection, this,
//                             ros::TransportHints().tcpNoDelay(true));

//    input_lane_r = node.subscribe("/lane_right", 10, &PuckAlgorithm::objectdetection, this,
//                             ros::TransportHints().tcpNoDelay(true));

//    bounding_box = node.subscribe("/bounding_boxes", 10, &PuckAlgorithm::objectdetection, this,
//                             ros::TransportHints().tcpNoDelay(true));
//    clusters_message = node.subscribe("/cloud_clusters", 10, &PuckAlgorithm::objectdetection, this,
//                                      ros::TransportHints().tcpNoDelay(true));

//    input_lane_l = node.subscribe("/lane_left", 2, &PuckAlgorithm::objectdetection, this);
//    input_lane_r = node.subscribe("/lane_right", 2, &PuckAlgorithm::objectdetection, this);
//    bounding_box = node.subscribe("/bounding_boxes", 2, &PuckAlgorithm::objectdetection, this);
//    clusters_message = node.subscribe("/cloud_clusters", 2, &PuckAlgorithm::objectdetection, this);

    //bounding_box = node.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes",1);
   // clusters_message = h.advertise<autoware_msgs::CloudClusterArray>("/cloud_clusters",1);


    //VLP_16
    sensor_model_16= 16;
    sensor_height_16= 0.65;
    floor_removal_16= true;
    max_slope_16= 20.0;
    point_distance_16= 0.05;
    min_point_16= 3;
    clipping_thres_16 = 3.0;
    // gap_thres : lower parameter can detect curb : 0.15 ~ 0.2 is enough detecting curb
    gap_thres_16= 0.05;


    int default_horizontal_res_16;
    SetHorizontalRes(sensor_model_16, default_horizontal_res_16);
    horizontal_res_16 = default_horizontal_res_16;

    points_node_sub_16 = node.subscribe("/velodyne_preprocess", 2, &PuckAlgorithm::VelodyneCallback_16, this);
    groundless_points_pub_16 = node.advertise<sensor_msgs::PointCloud2>("/points_no_ground_16", 2);
    ground_points_pub_16 = node.advertise<sensor_msgs::PointCloud2>("/points_ground_16", 2);

    vertical_res_16 = sensor_model_16;
    InitLabelArray_16(sensor_model_16);
    limiting_ratio_16 = tan(max_slope_16*M_PI/180);



    //HDL 32

    sensor_model_= 32;
    sensor_height_= 1.3;
    floor_removal_= true;
    max_slope_= 20.0;
    point_distance_= 0.3;
    min_point_= 3;
    clipping_thres_ = 1.0;
    // gap_thres : lower parameter can detect curb : 0.15 ~ 0.2 is enough detecting curb
    gap_thres_= 0.05;

    int default_horizontal_res_;
    SetHorizontalRes(sensor_model_, default_horizontal_res_);
    horizontal_res_= default_horizontal_res_;

    points_node_sub_ = node.subscribe("/velodyne_preprocess32", 2, &PuckAlgorithm::VelodyneCallback, this);
    groundless_points_pub_ = node.advertise<sensor_msgs::PointCloud2>("/points_no_ground_32", 2);
    ground_points_pub_ = node.advertise<sensor_msgs::PointCloud2>("/points_ground_32", 2);

    vertical_res_ = sensor_model_;
    InitLabelArray(sensor_model_);
    limiting_ratio_ = tan(max_slope_*M_PI/180);

}


void PuckAlgorithm::preprocessing16(const VPointCloud::ConstPtr &inMsg)
{
    if (output_16.getNumSubscribers() == 0)         // no one listening?
        return;                                     // do nothing
    VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = inMsg->header.stamp;
    //  outMsg->header.frame_id = inMsg->header.frame_id;
    outMsg->header.frame_id = "rear_lidar";
    outMsg->height = 1;

    for (size_t i = 0; i < inMsg->points.size(); ++i)
    {
        VPoint p;

        if(inMsg->points[i].x > -4.5 && inMsg->points[i].x < 0.2 && inMsg->points[i].y > -1.0  && inMsg->points[i].y < 1.0)
        {
            continue;
        }
        else
        {
            p.x = inMsg->points[i].x;
            p.y = inMsg->points[i].y;
            p.z = inMsg->points[i].z;
            p.ring = inMsg->points[i].ring;
            p.intensity = inMsg->points[i].intensity;
            outMsg->points.push_back(p);
            ++outMsg->width;
        }

    }
    //        int point_size = inMsg->points.size();
    //        ROS_INFO_STREAM("data size" << point_size );
    //        int point_size2 = outMsg->width;
    //        ROS_INFO_STREAM("out data size" << point_size2 );
    output_16.publish(outMsg);
}


void PuckAlgorithm::preprocessing32(const VPointCloud::ConstPtr &inMsg)
{
    if (output_32.getNumSubscribers() == 0)
        return;

    VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = inMsg->header.stamp;
    //  outMsg->header.frame_id = inMsg->header.frame_id;
    outMsg->header.frame_id = "center_lidar";
    outMsg->height = 1;

    for (size_t i = 0; i < inMsg->points.size(); ++i)
    {
        VPoint p;

        if(inMsg->points[i].x > -1.0 && inMsg->points[i].x < 1.0 && inMsg->points[i].y > -2.5  && inMsg->points[i].y < 2.5 && inMsg->points[i].z < 0 && inMsg->points[i].z > - 1.3)
        {
            continue;
        }
        else if(inMsg->points[i].x > 3.0 && inMsg->points[i].x < 5.0 && inMsg->points[i].y > 0.0  && inMsg->points[i].y < 1.5 && inMsg->points[i].z < -2.0)
        {
            continue;
        }
        else if(inMsg->points[i].x > -5.0 && inMsg->points[i].x < -3.0 && inMsg->points[i].y > 0.0  && inMsg->points[i].y < 1.5 && inMsg->points[i].z < -2.0)
        {
            continue;
        }

        else if(inMsg->points[i].z > 1.0)
        {
            continue;
        }
        //        else if(inMsg->points[i].x > 5.0)
        //        {
        //            continue;
        //        }


        else
        {
            //            p.x = -inMsg->points[i].y;
            //            p.y = inMsg->points[i].x;
            p.x = inMsg->points[i].x;
            p.y = inMsg->points[i].y;
            p.z = inMsg->points[i].z;
            p.ring = inMsg->points[i].ring;
            p.intensity = inMsg->points[i].intensity;
            outMsg->points.push_back(p);
            ++outMsg->width;
        }

    }
    output_32.publish(outMsg);
}

void PuckAlgorithm::process_cell(const VPointCloud::ConstPtr &inMsg)
{
    if (output_1.getNumSubscribers() == 0)         // no one listening?
        return;                                     // do nothing
    {
        nav_msgs::GridCells gridmap;
        //nav_msgs::OccupancyGrid gridmap;
        gridmap.cell_width = CELL_MIN_SIZE;
        gridmap.cell_height = CELL_MIN_SIZE;
        gridmap.header.stamp = ros::Time::now();
        gridmap.cells.clear();
        gridmap.header.frame_id = "center_lidar";


        // Projection on the occupancy grid map
        for (size_t s_idx = 0; s_idx < inMsg->points.size(); s_idx++)
        {
            // ROS_INFO_STREAM("s_idx =" << s_idx );
            if(inMsg->points[s_idx].y > Y_METER)
            {
                continue;
            }

            if(inMsg->points[s_idx].y < -Y_METER)
            {
                continue;
            }

            if(inMsg->points[s_idx].x > X_METER)
            {
                continue;
            }

            if(inMsg->points[s_idx].x < -X_METER)
            {
                continue;
            }

            if(inMsg->points[s_idx].x > -1.5 && inMsg->points[s_idx].x < 1.5 && inMsg->points[s_idx].y > -1.0  && inMsg->points[s_idx].y < 1.0)
            {
                continue;
            }

            geometry_msgs::Point point;
            point.x = floor(inMsg->points[s_idx].x/SIZE_GRID)*SIZE_GRID;  //(float)xIdx*0.2-Y_METER;
            point.y = floor(inMsg->points[s_idx].y/SIZE_GRID)*SIZE_GRID; // (float)yIdx*0.2-Y_METER;
            point.z = 0;
            gridmap.cells.push_back(point);

        }
        output_1.publish(gridmap);

    }
}


void PuckAlgorithm::curbdetection(const VPointCloud::ConstPtr &inMsg)
{
    int s_idx, l_idx, i;
    float dist,dist1;
    int countPt;
    int ptIdx1, ptIdx2,ptIdx3, ptIdx, ptIdxNext, ptIdxBefore;
    double intercept1,intercept2, slope, slope1, slope2, circle_slope1;
    double signVal, signVal1, signVal2;
    int flagForFail;
    int startIdx, endIdx;
    double thInlierCurbLaneDist = 0.01;  //0.03 base
    double heightSign1,heightSign2;
    int remainCount;
    double slopeDiff1, slopeDiff2;

    VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = inMsg->header.stamp;
    outMsg->header.frame_id = "center_lidar";
    outMsg->height = 1;
    //velodyne_pointcloud::PointXYZIR point;
    //velodyne_pointcloud::PointXYZIR point_curb;
    VPoint point_curb;

    horizontal_res_curb = 2250;
    vertical_res_curb = 32;
    //        horizontal_res_curb = 1800;
    //        vertical_res_curb = 16;

    //This line is not necessary
    //horizontal_res_ = int(in_cloud_msg->points.size() / vertical_res_);
    //InitDepthMap(horizontal_res_);

    const int mOne = -1;
    const int mtwo = 0;

    index_map_vel = cv::Mat_<int>(vertical_res_curb, horizontal_res_curb, mOne);
    // index_map_curb = cv::Mat_<int>(vertical_res_curb, horizontal_res_curb, mtwo);

    for (size_t i = 0; i < inMsg->points.size(); i++)
    {
        double u = atan2(inMsg->points[i].y,inMsg->points[i].x) * 180/M_PI;
        if (u < 0) { u = 360 + u; }
        //                int column = horizontal_res_ - (int)((double)horizontal_res_ * u / 360.0) - 1;
        //                int row = vertical_res_ - 1 - inMsg->points[i].ring;
        int column = (int)((double)horizontal_res_curb * u / 360.0);
        int row = inMsg->points[i].ring;
        index_map_vel.at<int>(row, column) = i;
        //        int point_size =row;
        //        ROS_INFO_STREAM("row " << point_size );
        //        int point_size2 =column;
        //        ROS_INFO_STREAM("column " << point_size2 );
    }


    for (int l_idx = 5 ; l_idx < vertical_res_curb-10; l_idx++)
    {
        // 1. two points selection
        for (int s_idx = 0; s_idx < horizontal_res_curb; s_idx++)
        {
            flagForFail = 0; startIdx = 0; endIdx = 0;

            ptIdx1 = index_map_vel.at<int>(l_idx,s_idx);
            if (s_idx < horizontal_res_curb - 10)
                ptIdx2 = index_map_vel.at<int>(l_idx,s_idx+ 10);
            else
                ptIdx2 = index_map_vel.at<int>(l_idx,s_idx-horizontal_res_curb+10);

//            if(s_idx > 9)
//                ptIdx3 = index_map_vel.at<int>(l_idx,s_idx-10);
//            else
//                ptIdx3 = index_map_vel.at<int>(l_idx,horizontal_res_curb+s_idx-10);

            if(ptIdx1 < 0 || ptIdx2 < 0) //0 || ptIdx3 < 0 )
            {
                flagForFail = 1;
                continue;
            }

            heightSign1 = inMsg->points[ptIdx1].z - inMsg->points[ptIdx2].z;
//            heightSign2 = inMsg->points[ptIdx3].z - inMsg->points[ptIdx1].z;

            if(heightSign1==0)
            {
                 flagForFail = 1;
                 continue;
            }
//            if(heightSign2==0)
//                continue;

            slope1 = (inMsg->points[ptIdx2].y-inMsg->points[ptIdx1].y) / (inMsg->points[ptIdx2].x-inMsg->points[ptIdx1].x);
//            slope2 = (inMsg->points[ptIdx1].y-inMsg->points[ptIdx3].y) / (inMsg->points[ptIdx1].x-inMsg->points[ptIdx3].x);

            intercept1 = inMsg->points[ptIdx1].y-slope1*inMsg->points[ptIdx1].x;
//            intercept2 = inMsg->points[ptIdx1].y-slope2*inMsg->points[ptIdx1].x;

            remainCount = 0;

            //  dist = 0;
            // 2. Check inlier btw. 2 points based on straight line, height same direction reduction
            for(int i = s_idx+1 ; i < s_idx+10 ; i++)
            {
                if(index_map_vel.at<int>(l_idx, i)  < 0)
                {
                    remainCount++;
                    continue;
                }

                if(index_map_vel.at<int>(l_idx, i+1)  < 0)
                {
                    remainCount++;
                    continue;
                }

                ptIdx = index_map_vel.at<int>(l_idx,i);
                ptIdxNext = index_map_vel.at<int>(l_idx,i+1);

                if((inMsg->points[ptIdx].z-inMsg->points[ptIdxNext].z)*heightSign1 < 0)
                {
                    flagForFail = 1;
                    break;
                }

                 circle_slope1 = (-inMsg->points[ptIdx].x /inMsg->points[ptIdx].y);
                 dist1 = fabs(slope1*inMsg->points[ptIdx].x-inMsg->points[ptIdx].y + intercept1)/sqrt(slope1*slope1+(-1.0)*(-1.0));

//                 if(fabs(slope1-circle_slope1) > 0.2)
                if(dist1 > thInlierCurbLaneDist)
                 {
                     remainCount++;
                     continue;
                 }

                 if(remainCount > 2)
                 {
                     flagForFail = 1;
                     break;
                 }
//                 dist1 = fabs(slope1*inMsg->points[ptIdx].x-inMsg->points[ptIdx].y + intercept1)/sqrt(slope1*slope1+(-1.0)*(-1.0));
            }

            if(fabs(inMsg->points[ptIdx1].z - inMsg->points[ptIdx2].z) < 0.035)
            {
              flagForFail = 1;
            }



            if(flagForFail == 1)
                continue;

            remainCount = 0;
            startIdx = s_idx;



            /*
              if(s_idx > 9)
            {
                for(i = s_idx ; i > 0 ; i--)
                {
                    if(index_map_vel.at<int>(l_idx, i) < 0)
                    {
                        remainCount++;
                        continue;
                    }
                    if(index_map_vel.at<int>(l_idx, i-1) < 0)
                    {
                        remainCount++;
                        continue;
                    }

                    ptIdx =index_map_vel.at<int>(l_idx,i);
                    ptIdxBefore = index_map_vel.at<int>(l_idx,i-1);

                    if((inMsg->points[ptIdxBefore].z - inMsg->points[ptIdx].z)*heightSign2< 0)
                    {
                        flagForFail = 1;
                        break;
                    }
                    else
                    {
                        if(dist>thInlierCurbLaneDist)
                        {
                            remainCount++;
                            if(remainCount == 5)
                            {
                                break;
                            }
                        }
                        else
                        {
                            startIdx = i;
                        }

                    }

                }
            }

               remainCount = 0;
               endIdx = s_idx+10;

              for(i=s_idx+10+1 ; i< horizontal_res_curb-1 ; i++)
              {
                  if(index_map_vel.at<int>(l_idx, i)  < 0)
                  {
                      remainCount++;
                      continue;
                  }

                  ptIdx =index_map_vel.at<int>(l_idx,i);
                  ptIdxNext = index_map_vel.at<int>(l_idx,i+1);

                  circle_slope1 = (-inMsg->points[ptIdx].x /inMsg->points[ptIdx].y);

                  if((inMsg->points[ptIdx].z - inMsg->points[ptIdxNext].z)*heightSign1<0)
                  {
                      break;
                  }

                  else
                  {
                      if(fabs(slope1-circle_slope1) > 0.2)
                      {
                          remainCount++;
                          if(remainCount==5)
                          {
                              break;
                          }
                       }
                      else
                      {
                          endIdx = i;
                      }
                  }
              }


     if(fabs(inMsg->points[ index_map_vel.at<int>(l_idx,endIdx)].z - inMsg->points[ index_map_vel.at<int>(l_idx,startIdx)].z) <  0.1)
       continue;
/*


            dist1 = fabs(slope1*inMsg->points[ptIdx].x-inMsg->points[ptIdx].y + intercept1)/sqrt(slope1*slope1+(-1.0)*(-1.0));
            //    dist += dist1;

            if(dist > thInlierCurbLaneDist)
            {
                remainCount++;
                //                    continue;
                if(remainCount == 4 )
                {
                    flagForFail = 1;
                    break;
                }
            }
            if((inMsg->points[ptIdxNext].z-inMsg->points[ptIdx].z)*heightSign1 < 0)
            {
                flagForFail = 1;
                break;
            }
        }


        if(remainCount > 8)
            continue;

        if(flagForFail == 1)
            continue;


        remainCount = 0;
        startIdx = s_idx;



        //3-1. inlier extension (up)
        for(i = s_idx-1 ; i > 0 ; i--)
        {
            if(index_map_vel.at<int>(l_idx, i) < 0)
            {
                continue;
            }

            ptIdx =index_map_vel.at<int>(l_idx,i);
            ptIdxBefore = index_map_vel.at<int>(l_idx,i-1);

            dist = fabs(slope* inMsg->points[ptIdx].x - inMsg->points[ptIdx].y + intercept) / sqrt(slope*slope+(-1.0)*(-1.0));
            //    dist = fabs(slope*puck_data->m_velo[ptIdx*3+0]-puck_data->m_velo[ptIdx*3+1]+intercept)/sqrt(slope*slope+(-1.0)*(-1.0));

            if((inMsg->points[ptIdx].z - inMsg->points[ptIdxBefore].z)*heightSign < 0)
            {
                break;
            }

            else
            {
                if(dist>thInlierCurbLaneDist)
                {
                    remainCount++;
                    if(remainCount == 8)
                    {
                        break;
                    }
                }
                else
                {
                    startIdx = i;
                }

            }
        }

        remainCount = 0;
        endIdx = s_idx+15;



        //3-2. inlier extension (down)

        for(i=s_idx+15+1 ; i< horizontal_res_curb-1 ; i++)
        {
            if(index_map_vel.at<int>(l_idx, i)  < 0)
            {
                continue;
            }

            ptIdx =index_map_vel.at<int>(l_idx,i);
            ptIdxNext = index_map_vel.at<int>(l_idx,i+1);

            dist = fabs(slope* inMsg->points[ptIdx].x - inMsg->points[ptIdx].y + intercept) / sqrt(slope*slope+(-1.0)*(-1.0));

            if((inMsg->points[ptIdxNext].z - inMsg->points[ptIdx].z)*heightSign<0)
            {
                break;
            }

            else
            {
                if(dist>thInlierCurbLaneDist)
                {
                    remainCount++;
                    if(remainCount == 7)
                    {
                        break;
                    }
                }
                else
                {
                    endIdx = i;
                }
            }
        }

        // Check the validity about the start, end point of line (remove points by camparison the xy slope )
        if(fabs(inMsg->points[ index_map_vel.at<int>(l_idx,endIdx)].z - inMsg->points[ index_map_vel.at<int>(l_idx,startIdx)].z) <  0.1)
            continue;

        for(i=10 ; i<20 ; i++)
        {
            if(startIdx-i<0)
            {
                i=20;
                break;
            }
            if(index_map_vel.at<int>(l_idx, startIdx-i)  < 0)
            {
                continue;
            }
            else
            {
                break;
            }
        }

        //            if(i == 20)
        //                slope1 = slope;
        //            else
        //                slope1 = (inMsg->points[ index_map_vel.at<int>(l_idx,startIdx-i)].y - inMsg->points[ index_map_vel.at<int>(l_idx,startIdx)].y / inMsg->points[ index_map_vel.at<int>(l_idx,startIdx-i)].x - inMsg->points[ index_map_vel.at<int>(l_idx,startIdx)].x );

        //            for(i=10 ; i<20 ; i++)
        //            {
        //                if(endIdx+i>horizontal_res_curb-1)
        //                {
        //                    i=20;
        //                    break;
        //                }

        //                if(index_map_vel.at<int>(l_idx, endIdx+i) < 0)
        //                {
        //                    continue;
        //                }
        //                else
        //                {
        //                    break;
        //                }
        //            }

        //            if(i == 20)
        //                slope2 = slope;
        //            else
        //                slope2 = (inMsg->points[ index_map_vel.at<int>(l_idx,endIdx+i)].y - inMsg->points[ index_map_vel.at<int>(l_idx,endIdx)].y / inMsg->points[ index_map_vel.at<int>(l_idx,endIdx+i)].x - inMsg->points[ index_map_vel.at<int>(l_idx,endIdx)].x );

        //            // I have to set up slope difference basis (criteria) - degree difference
        //            slopeDiff1 = fabs((atan(slope1)*180/PI)-(atan(slope)*180/PI));
        //            slopeDiff2 = fabs((atan(slope2)*180/PI)-(atan(slope)*180/PI));

        //            if((atan(slope)*180/PI)>15.0)
        //                continue;

        //            if((atan(slope)*180/PI)<-15.0)
        //                continue;

        //            if(slopeDiff1<10.0)
        //                continue;

        //            if(slopeDiff2<10.0)
        //                continue;


        signVal1 = 0.0;



        //4-1. sign check (up)
        countPt = 0;
        for(i=(startIdx)-1 ; i>=0 ; i--)
        {
            if(index_map_vel.at<int>(l_idx, i) < 0)
            {
                continue;
            }

            countPt++;

            ptIdx = index_map_vel.at<int>(l_idx,i);

            signVal = slope*inMsg->points[ptIdx].x -inMsg->points[ptIdx].y + intercept;
            if(signVal>0)
                signVal = 1;
            else
                signVal = -1;

            signVal1 = signVal1+signVal;

            if(heightSign>0)
            {
                if(inMsg->points[ ptIdx].z < inMsg->points[ index_map_vel.at<int>(l_idx,s_idx)].z)
                {
                    flagForFail = 1;
                    break;
                }
            }
            else
            {
                if(inMsg->points[ ptIdx].z > inMsg->points[ index_map_vel.at<int>(l_idx,s_idx)].z)
                {
                    flagForFail = 1;
                    break;
                }
            }

            if(countPt==10)
                break;
        }

        if(countPt<10)
            continue;

        if(flagForFail==1)
            continue;



        signVal2 = 0.0;
        //4-2. sign check (down)
        countPt = 0;
        for(i=(endIdx)+1 ; i<horizontal_res_curb ; i++)
        {
            if(index_map_vel.at<int>(l_idx, i) < 0)
            {
                continue;
            }

            countPt++;

            ptIdx = index_map_vel.at<int>(l_idx,i);

            signVal = slope*inMsg->points[ptIdx].x -inMsg->points[ptIdx].y + intercept;

            if(signVal>0)
                signVal = 1;
            else
                signVal = -1;

            signVal2 = signVal2+signVal;

            if(heightSign>0)
            {
                if(inMsg->points[ ptIdx].z > inMsg->points[ index_map_vel.at<int>(l_idx,s_idx+10)].z)
                {
                    flagForFail = 1;
                    break;
                }
            }
            else
            {
                if(inMsg->points[ ptIdx].z < inMsg->points[ index_map_vel.at<int>(l_idx,s_idx+10)].z)
                {
                    flagForFail = 1;
                    break;
                }
            }

            if(countPt==10)
                break;
        }

        if(countPt<10)
            continue;

        if(flagForFail==1)
            continue;

        //if(signVal1*signVal2>-80)
        //	continue;
*/

        //5. final decision to inlier

        //            startIdx  = 500;
        //            endIdx = 700;

//            startIdx = s_idx;
//            endIdx = s_idx + 10;


//      if(flagForFail == 0)
//      {
//        for(i=startIdx ; i<=endIdx ; i++)
//        {
//            // ptIdx = i*NUM_LAYER+l_idx;
//            if(index_map_vel.at<int>(l_idx, i) < 0)
//            {
//                continue;

//            }
//            ptIdx = index_map_vel.at<int>(l_idx,i);

//            //               index_map_vel.at<int>(l_idx,i) = 1;
//            // puck_data->m_curbPt[ptIdx] = 1;
//            point_curb.x = inMsg->points[ptIdx].x;
//            point_curb.y = inMsg->points[ptIdx].y;
//            point_curb.z = inMsg->points[ptIdx].z;
//            point_curb.intensity = inMsg->points[ptIdx].intensity;
//            point_curb.ring = inMsg->points[ptIdx].ring;
//            outMsg->points.push_back(point_curb);
//       //     ++outMsg->width;
//            //                ROS_WARN("44444444444444444444444444");
//        }
//      }
//        //                         output_curb.publish(outMsg);
//        //             ROS_INFO_STREAM("startIdx" << startIdx);
//        //             ROS_INFO_STREAM("endIdx" << endIdx);

//        //            ROS_WARN("11111111111111111111111111111");
//    }
//    //        ROS_WARN("22222222222222222222222222222222");
//}
//output_curb.publish(outMsg);


//    ROS_WARN("333333333333333333333333333333333");
//    for (size_t i = 0; i < inMsg->points.size(); ++i)
//    {
//          VPoint p;
//        if(index_map_vel.at<int>(l_idx,i) == 1)
//        {
//            point_curb.x = inMsg->points[index_map_vel.at<int>(l_idx,i)].x;
//            point_curb.y = inMsg->points[index_map_vel.at<int>(l_idx,i)].y;
//            point_curb.z = inMsg->points[index_map_vel.at<int>(l_idx,i)].z;
//            point_curb.intensity = inMsg->points[index_map_vel.at<int>(l_idx,i)].intensity;
//            point_curb.ring = inMsg->points[index_map_vel.at<int>(l_idx,i)].ring;
//            outMsg->points.push_back(point_curb);
//          //  ++outMsg->width;
//        }
//    }

//      output_curb.publish(outMsg);


                if(index_map_vel.at<int>(l_idx, s_idx) < 0)
                {
                    continue;
                }

                if(s_idx > 0 && s_idx < horizontal_res_curb && flagForFail == 0)
                {
                    ptIdx = index_map_vel.at<int>(l_idx,s_idx);
                    point_curb.x = inMsg->points[ptIdx].x;
                    point_curb.y = inMsg->points[ptIdx].y;
                    point_curb.z = inMsg->points[ptIdx].z;
                    point_curb.intensity = inMsg->points[ptIdx].intensity;
                    point_curb.ring = inMsg->points[ptIdx].ring;
                    outMsg->points.push_back(point_curb);
                    ++outMsg->width;
                }
            }
        }
        output_curb.publish(outMsg);
}





void PuckAlgorithm::SetHorizontalRes(const int sensor_model, int &horizontal_res)
{
    switch(sensor_model)
    {
    case 64:
        horizontal_res = 2083;
        break;
    case 32:
        horizontal_res = 2250;
        break;
    case 16:
        horizontal_res = 1800;
        break;
    default:
        horizontal_res = DEFAULT_HOR_RES;
        break;
    }
}

void PuckAlgorithm::InitLabelArray(int in_model)
{
    for(int a = 0; a < vertical_res_; a++)
    {
        class_label_[a] = UNKNOWN;
    }
}

void PuckAlgorithm::InitLabelArray_16(int in_model)
{
    for(int a = 0; a < vertical_res_16; a++)
    {
        class_label_16[a] = UNKNOWN;
    }
}

void PuckAlgorithm::InitDepthMap(int in_width)
{
    const int mOne = -1;
    index_map_ = cv::Mat_<int>(vertical_res_, in_width, mOne);
}

void PuckAlgorithm::InitDepthMap_16(int in_width)
{
    const int mOne = -1;
    index_map_16 = cv::Mat_<int>(vertical_res_16, in_width, mOne);
}



void PuckAlgorithm::PublishPointCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
                                      int in_indices[], int &in_out_index_size,
                                      pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &in_cloud)
{
    velodyne_pointcloud::PointXYZIR point;
    for (int i = 0; i < in_out_index_size; i++)
    {
        point.x = in_cloud_msg->points[in_indices[i]].x;
        point.y = in_cloud_msg->points[in_indices[i]].y;
        point.z = in_cloud_msg->points[in_indices[i]].z;
        point.intensity = in_cloud_msg->points[in_indices[i]].intensity;
        point.ring = in_cloud_msg->points[in_indices[i]].ring;
        in_cloud.push_back(point);
    }
    in_out_index_size = 0;
}


void PuckAlgorithm::FilterGround(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
                                 pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_groundless_points,
                                 pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_ground_points)
{

    velodyne_pointcloud::PointXYZIR point;

    //This line is not necessary
    //horizontal_res_ = int(in_cloud_msg->points.size() / vertical_res_);
    InitDepthMap(horizontal_res_);

    for (size_t i = 0; i < in_cloud_msg->points.size(); i++)
    {
        double u = atan2(in_cloud_msg->points[i].y,in_cloud_msg->points[i].x) * 180/M_PI;
        if (u < 0) { u = 360 + u; }
        int column = horizontal_res_ - (int)((double)horizontal_res_ * u / 360.0) - 1;
        int row = vertical_res_ - 1 - in_cloud_msg->points[i].ring;
        index_map_.at<int>(row, column) = i;
    }

    for (int i = 0; i < horizontal_res_; i++)
    {
        Label point_class[vertical_res_];
        int unknown_index[vertical_res_];
        int point_index[vertical_res_];
        int unknown_index_size = 0;
        int point_index_size = 0;
        double z_ref = 0;
        double r_ref = 0;
        std::copy(class_label_, class_label_ + vertical_res_, point_class);

        for (int j = vertical_res_ - 1; j >= 0; j--)
        {
            if (index_map_.at<int>(j,i) > -1 && point_class[j] == UNKNOWN)
            {
                double x0 = in_cloud_msg->points[index_map_.at<int>(j, i)].x;
                double y0 = in_cloud_msg->points[index_map_.at<int>(j, i)].y;
                double z0 = in_cloud_msg->points[index_map_.at<int>(j, i)].z;
                double r0 = sqrt(x0*x0 + y0*y0);
                double r_diff = r0 - r_ref;
                double z_diff = fabs(z0 - z_ref);
                double pair_angle;

                if (r_diff != 0.)
                {
                    pair_angle = z_diff/r_diff;
                }
                else
                {//this should never execute due to Sensor specs
                    ROS_WARN("RingGroundFilter: Division by Zero avoided on pair_angle");
                    pair_angle = 0;
                }
                if (
                        (	(pair_angle > 0 && pair_angle < limiting_ratio_)
                            && z_diff < gap_thres_
                            && z0 < clipping_thres_
                            )
                        || point_index_size == 0
                        )
                {
                    r_ref = r0;
                    z_ref = z0;
                    point_index[point_index_size] = j;
                    point_index_size++;
                }
                else
                {
                    if (point_index_size > min_point_)
                    {
                        for (int m = 0; m < point_index_size; m++)
                        {
                            int index = index_map_.at<int>(point_index[m],i);
                            point.x = in_cloud_msg->points[index].x;
                            point.y = in_cloud_msg->points[index].y;
                            point.z = in_cloud_msg->points[index].z;
                            point.intensity = in_cloud_msg->points[index].intensity;
                            point.ring = in_cloud_msg->points[index].ring;
                            out_ground_points.push_back(point);
                            point_class[point_index[m]] = GROUND;
                        }
                        point_index_size = 0;
                    }
                    else
                    {
                        for (int m = 0; m < point_index_size; m++)
                        {
                            int index = index_map_.at<int>(point_index[m],i);
                            point.z = in_cloud_msg->points[index].z;
                            if (point.z > clipping_thres_ - sensor_height_)
                            {
                                point.x = in_cloud_msg->points[index].x;
                                point.y = in_cloud_msg->points[index].y;
                                point.intensity = in_cloud_msg->points[index].intensity;
                                point.ring = in_cloud_msg->points[index].ring;
                                out_groundless_points.push_back(point);
                                point_class[point_index[m]] = VERTICAL;
                            }
                            else
                            {
                                unknown_index[unknown_index_size] = index;
                                unknown_index_size++;
                            }
                        }
                        point_index_size = 0;
                    }
                    //These line were missing
                    r_ref = r0;
                    z_ref = z0;
                    point_index[point_index_size] = j;
                    point_index_size++;
                }
            }
            if (j == 0)
            {
                if (point_index_size != 0)
                {
                    if (point_index_size > min_point_)  // min_points
                    {
                        for (int m = 0; m < point_index_size; m++)
                        {
                            int index = index_map_.at<int>(point_index[m],i);
                            point.x = in_cloud_msg->points[index].x;
                            point.y = in_cloud_msg->points[index].y;
                            point.z = in_cloud_msg->points[index].z;
                            point.intensity = in_cloud_msg->points[index].intensity;
                            point.ring = in_cloud_msg->points[index].ring;
                            out_ground_points.push_back(point);
                            point_class[point_index[m]] = GROUND;
                        }
                        point_index_size = 0;
                    }
                    else
                    {
                        for (int m = 0; m < point_index_size; m++)
                        {
                            int index = index_map_.at<int>(point_index[m],i);
                            point.z = in_cloud_msg->points[index].z;
                            if (point.z > clipping_thres_ - sensor_height_)
                            {
                                point.x = in_cloud_msg->points[index].x;
                                point.y = in_cloud_msg->points[index].y;
                                point.intensity = in_cloud_msg->points[index].intensity;
                                point.ring = in_cloud_msg->points[index].ring;
                                out_groundless_points.push_back(point);
                                point_class[point_index[m]] = VERTICAL;
                            }
                            else
                            {
                                unknown_index[unknown_index_size] = index;
                                unknown_index_size++;
                            }
                        }
                        point_index_size = 0;
                    }//end else
                }//end if (point_index_size != 0)

                double centroid = 0;
                int cluster_index[vertical_res_];
                int cluster_index_size = 0;
                for (int m = unknown_index_size - 1; m >= 0; m--)
                {
                    double x0 = in_cloud_msg->points[unknown_index[m]].x;
                    double y0 = in_cloud_msg->points[unknown_index[m]].y;
                    double r0 = sqrt(x0*x0 + y0*y0);
                    double r_diff = fabs(r0 - centroid);
                    if ((r_diff < point_distance_) || cluster_index_size == 0)
                    {
                        cluster_index[cluster_index_size] = unknown_index[m];
                        cluster_index_size++;
                        centroid = r0;
                        if (m == 0)
                        {
                            if(cluster_index_size > 1)
                            {
                                PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_groundless_points);
                            }
                            else
                            {
                                PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_ground_points);
                            }
                        }
                    }
                    else
                    {
                        if(cluster_index_size > 1)
                        {
                            PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_groundless_points);
                        }
                        else
                        {
                            PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_ground_points);
                        }
                        cluster_index[cluster_index_size] = unknown_index[m];
                        cluster_index_size++;
                        centroid = r0;
                    }
                    if (m == 0)
                    {
                        if(cluster_index_size > 1)
                        {
                            PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_groundless_points);
                        }
                        else
                        {
                            PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_ground_points);
                        }
                    }
                }//end for (int m = unknown_index_size - 1; m >= 0; m--)
            }//end if (j == 0)
        }
    }

}

void PuckAlgorithm::FilterGround_16(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
                                    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_groundless_points,
                                    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_ground_points)
{

    velodyne_pointcloud::PointXYZIR point;

    //This line is not necessary
    //horizontal_res_ = int(in_cloud_msg->points.size() / vertical_res_);
    InitDepthMap(horizontal_res_16);

    for (size_t i = 0; i < in_cloud_msg->points.size(); i++)
    {
        double u = atan2(in_cloud_msg->points[i].y,in_cloud_msg->points[i].x) * 180/M_PI;
        if (u < 0) { u = 360 + u; }
        int column = horizontal_res_16 - (int)((double)horizontal_res_16 * u / 360.0) - 1;
        int row = vertical_res_16 - 1 - in_cloud_msg->points[i].ring;
        index_map_16.at<int>(row, column) = i;
    }

    for (int i = 0; i < horizontal_res_16; i++)
    {
        Label point_class[vertical_res_16];
        int unknown_index[vertical_res_16];
        int point_index[vertical_res_16];
        int unknown_index_size = 0;
        int point_index_size = 0;
        double z_ref = 0;
        double r_ref = 0;
        std::copy(class_label_16, class_label_16 + vertical_res_16, point_class);

        for (int j = vertical_res_16 - 1; j >= 0; j--)
        {
            if (index_map_16.at<int>(j,i) > -1 && point_class[j] == UNKNOWN)
            {
                double x0 = in_cloud_msg->points[index_map_16.at<int>(j, i)].x;
                double y0 = in_cloud_msg->points[index_map_16.at<int>(j, i)].y;
                double z0 = in_cloud_msg->points[index_map_16.at<int>(j, i)].z;
                double r0 = sqrt(x0*x0 + y0*y0);
                double r_diff = r0 - r_ref;
                double z_diff = fabs(z0 - z_ref);
                double pair_angle;

                if (r_diff != 0.)
                {
                    pair_angle = z_diff/r_diff;
                }
                else
                {//this should never execute due to Sensor specs
                    ROS_WARN("RingGroundFilter: Division by Zero avoided on pair_angle");
                    pair_angle = 0;
                }
                if (
                        (	(pair_angle > 0 && pair_angle < limiting_ratio_16)
                            && z_diff < gap_thres_16
                            && z0 < clipping_thres_16
                            )
                        || point_index_size == 0
                        )
                {
                    r_ref = r0;
                    z_ref = z0;
                    point_index[point_index_size] = j;
                    point_index_size++;
                }
                else
                {
                    if (point_index_size > min_point_)
                    {
                        for (int m = 0; m < point_index_size; m++)
                        {
                            int index = index_map_16.at<int>(point_index[m],i);
                            point.x = in_cloud_msg->points[index].x;
                            point.y = in_cloud_msg->points[index].y;
                            point.z = in_cloud_msg->points[index].z;
                            point.intensity = in_cloud_msg->points[index].intensity;
                            point.ring = in_cloud_msg->points[index].ring;
                            out_ground_points.push_back(point);
                            point_class[point_index[m]] = GROUND;
                        }
                        point_index_size = 0;
                    }
                    else
                    {
                        for (int m = 0; m < point_index_size; m++)
                        {
                            int index = index_map_16.at<int>(point_index[m],i);
                            point.z = in_cloud_msg->points[index].z;
                            if (point.z > clipping_thres_16 - sensor_height_16)
                            {
                                point.x = in_cloud_msg->points[index].x;
                                point.y = in_cloud_msg->points[index].y;
                                point.intensity = in_cloud_msg->points[index].intensity;
                                point.ring = in_cloud_msg->points[index].ring;
                                out_groundless_points.push_back(point);
                                point_class[point_index[m]] = VERTICAL;
                            }
                            else
                            {
                                unknown_index[unknown_index_size] = index;
                                unknown_index_size++;
                            }
                        }
                        point_index_size = 0;
                    }
                    //These line were missing
                    r_ref = r0;
                    z_ref = z0;
                    point_index[point_index_size] = j;
                    point_index_size++;
                }
            }
            if (j == 0)
            {
                if (point_index_size != 0)
                {
                    if (point_index_size > min_point_)
                    {
                        for (int m = 0; m < point_index_size; m++)
                        {
                            int index = index_map_16.at<int>(point_index[m],i);
                            point.x = in_cloud_msg->points[index].x;
                            point.y = in_cloud_msg->points[index].y;
                            point.z = in_cloud_msg->points[index].z;
                            point.intensity = in_cloud_msg->points[index].intensity;
                            point.ring = in_cloud_msg->points[index].ring;
                            out_ground_points.push_back(point);
                            point_class[point_index[m]] = GROUND;
                        }
                        point_index_size = 0;
                    }
                    else
                    {
                        for (int m = 0; m < point_index_size; m++)
                        {
                            int index = index_map_16.at<int>(point_index[m],i);
                            point.z = in_cloud_msg->points[index].z;
                            if (point.z > clipping_thres_16 - sensor_height_16)
                            {
                                point.x = in_cloud_msg->points[index].x;
                                point.y = in_cloud_msg->points[index].y;
                                point.intensity = in_cloud_msg->points[index].intensity;
                                point.ring = in_cloud_msg->points[index].ring;
                                out_groundless_points.push_back(point);
                                point_class[point_index[m]] = VERTICAL;
                            }
                            else
                            {
                                unknown_index[unknown_index_size] = index;
                                unknown_index_size++;
                            }
                        }
                        point_index_size = 0;
                    }//end else
                }//end if (point_index_size != 0)

                double centroid = 0;
                int cluster_index[vertical_res_16];
                int cluster_index_size = 0;
                for (int m = unknown_index_size - 1; m >= 0; m--)
                {
                    double x0 = in_cloud_msg->points[unknown_index[m]].x;
                    double y0 = in_cloud_msg->points[unknown_index[m]].y;
                    double r0 = sqrt(x0*x0 + y0*y0);
                    double r_diff = fabs(r0 - centroid);
                    if ((r_diff < point_distance_16) || cluster_index_size == 0)
                    {
                        cluster_index[cluster_index_size] = unknown_index[m];
                        cluster_index_size++;
                        centroid = r0;
                        if (m == 0)
                        {
                            if(cluster_index_size > 1)
                            {
                                PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_groundless_points);
                            }
                            else
                            {
                                PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_ground_points);
                            }
                        }
                    }
                    else
                    {
                        if(cluster_index_size > 1)
                        {
                            PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_groundless_points);
                        }
                        else
                        {
                            PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_ground_points);
                        }
                        cluster_index[cluster_index_size] = unknown_index[m];
                        cluster_index_size++;
                        centroid = r0;
                    }
                    if (m == 0)
                    {
                        if(cluster_index_size > 1)
                        {
                            PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_groundless_points);
                        }
                        else
                        {
                            PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_ground_points);
                        }
                    }
                }//end for (int m = unknown_index_size - 1; m >= 0; m--)
            }//end if (j == 0)
        }
    }

}



void PuckAlgorithm::ConfigCallback(const autoware_config_msgs::ConfigRingGroundFilterConstPtr &config)
{
    sensor_model_ = std::stoi(config->sensor_model);
    sensor_height_ = config->sensor_height;
    max_slope_ = config->max_slope;
    point_distance_ = config->point_distance;
    min_point_ = config->min_point;
    clipping_thres_ = config->clipping_thres;
    gap_thres_ = config->gap_thres;
    SetHorizontalRes(sensor_model_, horizontal_res_);
}

void PuckAlgorithm::ConfigCallback16(const autoware_config_msgs::ConfigRingGroundFilterConstPtr &config)
{
    sensor_model_16 = std::stoi(config->sensor_model);
    sensor_height_16 = config->sensor_height;
    max_slope_ = config->max_slope;
    point_distance_ = config->point_distance;
    min_point_ = config->min_point;
    clipping_thres_ = config->clipping_thres;
    gap_thres_ = config->gap_thres;
    SetHorizontalRes(sensor_model_16, horizontal_res_16);
}


void PuckAlgorithm::VelodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg)
{

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> vertical_points;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ground_points;
    vertical_points.header = in_cloud_msg->header;
    ground_points.header = in_cloud_msg->header;
    //    vertical_points.header.frame_id = "rear_lidar";
    //    ground_points.header.frame_id = "rear_lidar";
    vertical_points.clear();
    ground_points.clear();

    FilterGround(in_cloud_msg, vertical_points, ground_points);

    if (!floor_removal_)
    {
        vertical_points = *in_cloud_msg;
    }

    groundless_points_pub_.publish(vertical_points);
    ground_points_pub_.publish(ground_points);

}

void PuckAlgorithm::VelodyneCallback_16(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg)
{

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> vertical_points_16;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ground_points_16;
    vertical_points_16.header = in_cloud_msg->header;
    ground_points_16.header = in_cloud_msg->header;
    //    vertical_points_16.header.frame_id = "rear_lidar";
    //    ground_points_16.header.frame_id = "rear_lidar";
    vertical_points_16.clear();
    ground_points_16.clear();

    FilterGround(in_cloud_msg, vertical_points_16, ground_points_16);

    if (!floor_removal_16)
    {
        vertical_points_16 = *in_cloud_msg;
    }

    groundless_points_pub_16.publish(vertical_points_16);
    ground_points_pub_16.publish(ground_points_16);

}

 //void PuckAlgorithm::objectdetection(const jsk_recognition_msgs::BoundingBoxArray &inboundingbox)
//  void objectdetection(const jsk_recognition_msgs::BoundingBox &inboundingbox) // , autoware_msgs::CloudCluster &inclusters)
// {
////      autoware_msgs::CloudCluster cluster;
////      cluster.header.stamp = inclusters.header.stamp;
////      cluster.header.frame_id = "center_lidar";
//      jsk_recognition_msgs::BoundingBox boundingbox;
//      boundingbox.header.stamp = inboundingbox.header.stamp;
//      boundingbox.header.frame_id = "center_lidar";

//        for (auto i=inclusters.clusters.begin(); i!= inclusters.clusters.end(); i++)
//      {

//      }

//        for (auto i=inboundingbox.boxes.begin(); i!= inboundingbox.boxes.end(); i++)
//        {


//        }



// }

}
