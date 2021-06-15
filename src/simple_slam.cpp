#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>

struct Point2 {
public:

  Point2(int x_coord, int y_coord):
    x{x_coord}, y{y_coord} {}
  int x, y; ///< Coordinates of point
};


class calcLine {
  using Point = Point2;
public: // methods

  calcLine(const double startx,const double starty,const double endx,const double endy) {
    calcLinefunc(startx, starty, endx, endy);
  }
  /// Returns the line's component points
  const std::vector<Point>& points() const { return _points; }
private:

  void calcLinefunc(double x1, double y1, double x2, double y2) {
    // TODO: copypasted from
    //   http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
    //   review and simplification are required
  	x1 = int(x1*10);
  	y1 = int(y1*10);
  	x2 = int(x2*10);
  	y2 = int(y2*10);
    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) * 2;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) * 2;

    _points.push_back(Point(x1, y1));

    if (delta_x >= delta_y) {
      // error may go below zero
      int error(delta_y - (delta_x >> 1));
      while (x1 != x2) {
      	//std::cout<<"x1 "<<x1<<"x2 "<<x2<<std::endl;
        if ((0 <= error) && (error || (0 < ix))) {
          error -= delta_x;
          y1 += iy;
        }
        // else do nothing
        error += delta_y;
        x1 += ix;
        _points.push_back(Point(x1, y1));
      }
    }
    else {
      // error may go below zero
      int error(delta_x - (delta_y >> 1));

      while (y1 != y2) {
        if ((0 <= error) && (error || (0 < iy))) {
          error -= delta_y;
          x1 += ix;
        }
        // else do nothing
        error += delta_x;
        y1 += iy;
        _points.push_back(Point(x1, y1));
      }
    }
  }
private: // fields
  std::vector<Point> _points;
};


class subscribe
{
public:
  // the size of the map
  int array[200][200] = {10};

  subscribe()
  {
    pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/gridMap", 10);
    sub_ = n_.subscribe("/scan", 1000, &subscribe::chatterCallback,this);
  }
  
  void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    tf::StampedTransform transform;
    std::string msg_frame_id =ros::message_traits::FrameId<sensor_msgs::LaserScan>::value(*msg);
    ros::Time msg_time =ros::message_traits::TimeStamp<sensor_msgs::LaserScan>::value(*msg);
    std::string target_frame = "/odom";

    try{
      tf_lsnr.lookupTransform(target_frame,msg_frame_id,ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Transform failed. %s",ex.what());
      return;
    }

    using Point = Point2;
    double pos_x = transform.getOrigin().getX();
    double pos_y = transform.getOrigin().getY();
    double pos_angle = tf::getYaw(transform.getRotation());

    tf::Transform trans;
    trans.setOrigin(tf::Vector3(pos_x, pos_y, 0.0));
    tf::Quaternion qua;
    qua.setRPY(0, 0, pos_angle);
    trans.setRotation(qua);
    std::string frame_odom = "odom";
    std::string frame_robot_pose = "robot_pose";
    tf_brcst.sendTransform(tf::StampedTransform(trans, ros::Time::now(),frame_odom, frame_robot_pose));
    nav_msgs::OccupancyGrid map;
    map.header.frame_id="map";
    map.info.resolution = 0.1;         // float32
    map.info.width      = 200;           // uint32
    map.info.height     = 200;           // uint32
    map.info.origin.position.x = -10;
    map.info.origin.position.y = -10;
    map.header.stamp = ros::Time::now();
    

    for(int j=0;j<360;j++) //increment by one degree
    {
      double point1 = pos_x+msg->ranges[j]*std::cos(j*3.14/180 + pos_angle );
      double point2 = pos_y+msg->ranges[j]*std::sin(j*3.14/180 + pos_angle );
      
      if(point1>10 || point1<-10 || point2 >10 || point2<-10) continue;
      std::vector<Point> pts = calcLine(pos_x,pos_y, point1,point2).points();
      //std::cout<<pts.size()<<std::endl;
      array[int(point1*10)+100][int(point2*10)+100] = 90;
      pts.pop_back();
      for (const auto &pt : pts) {
        array[pt.x+100][pt.y+100] = 10;
      }
    }

    for(int j=0;j<200;j++){     
      for(int i=0;i<200;i++){
        map.data.push_back(array[i][j]);
      }
    }
    pub_.publish(map);
  }
  

private:
  tf::TransformBroadcaster tf_brcst;
  tf::TransformListener tf_lsnr;
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "simple_slam");
  subscribe start;
  ros::spin();
  return 0;
}

