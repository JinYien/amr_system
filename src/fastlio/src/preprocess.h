#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <livox_driver/msg/custom_msg.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};
enum Surround
{
  Prev,
  Next
};
enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

class Preprocess
{
public:
  Preprocess();
  ~Preprocess();

  void process(const livox_driver::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);

  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128];
  vector<orgtype> typess[128];
  int point_filter_num, N_SCANS;
  double blind;
  bool feature_enabled;

private:
  void livox_handler(const livox_driver::msg::CustomMsg::UniquePtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
