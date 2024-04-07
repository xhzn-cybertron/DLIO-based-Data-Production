#include "dlio/map.h"

/**
  * @brief 地图生成服务
  *
  * @param map_name  地图名
  * @param map_dir   地图保存目录
  * @param map         输入三维点云
  * @param min_height  滤掉最低点云
  * @param max_height  滤掉最高点云
 //  * @param traj        轨迹点云 , pcl::PointCloud<PointType>::Ptr traj
  */
void dlio::MapNode::saveJpgYamlPcd(pcl::PointCloud<PointType>::Ptr pc_map,
                                   double req_resolution) {
  std::cout << __func__ << __LINE__ << std::endl;
  // float min_height = mapping_min_height;
  // float max_height = mapping_max_height;
  pcl::PointCloud<PointType>::Ptr m =
      pcl::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map);
  std::string file_pure_path_ = std::getenv("HOME") + std::string("/map");
  // 1. 得到地面点和刚体点, 以及xy的取值范围
  pcl::PointCloud<PointType>::Ptr pc_all(new pcl::PointCloud<PointType>());
  // pcl::PointCloud<PointType>::Ptr pc_ground(new PclScan());
  float max_x = -100, min_x = 100, max_y = -100, min_y = 100;
  for (int i = 0; i < pc_map->points.size(); i++) {
    PointType pt = pc_map->points[i];
    if (pt.z > 0.2 || pt.z < -1.5) {
      continue;
    }
    // TODO 获取地面点点云聚类，建图更准确，便于全局路径规划
    //   if (pc_map->points[i].type == PointTypeGround) {
    //     pc_ground->points.push_back(pt);
    //   } else if (pc_map->points[i].type == PointTypeRigid) {
    //     pc_rigid->points.push_back(pt);
    //   }
    pc_all->points.push_back(pt);

    max_x = std::max(pt.x, max_x);
    max_y = std::max(pt.y, max_y);
    min_x = std::min(pt.x, min_x);
    min_y = std::min(pt.y, min_y);
  }

  // 2. 得到分辨率
  int border_size = 0;
  // int height = 6120 - 2 * border_size;
  // int width = 6120 - 2 * border_size;
  // double resolution_x = (max_x - min_x) / height;
  // double resolution_y = (max_y - min_y) / width;
  // double resolution =
  //     resolution_x > resolution_y ? resolution_x : resolution_y;
  double resolution_x = req_resolution;
  double resolution_y = req_resolution;
  double resolution = req_resolution;
  int height = (max_x - min_x) / resolution_x;
  int width = (max_y - min_y) / resolution_y;
  double center_x = (max_x + min_x) / 2.0;
  double center_y = (max_y + min_y) / 2.0;
  double center_corr_x = (max_x + min_x) / 2.0;
  double center_corr_y = (max_y + min_y) / 2.0;

  int jpg_size = height > width ? height : width;
  height = jpg_size;
  width = jpg_size;

  center_corr_x = (0.5 * height - center_x / resolution) * resolution;
  center_corr_y = (0.5 * width - center_y / resolution) * resolution;

  std::cout << "max_x = " << max_x << "min_x =" << min_x
            << "max_x - min_x= " << max_x - min_x << "center_x = " << center_x
            << std::endl;
  std::cout << "max_y = " << max_y << "min_y =" << min_y
            << "max_y - min_y= " << max_y - min_y << "center_y = " << center_y
            << std::endl;
  // 3. 构造MAT
  cv::Mat cv_map(height, width, CV_8UC1, cv::Scalar::all(0));
  cv::Mat mask = cv::Mat::zeros(cv_map.size(), CV_8UC1);

  // 定义像素与世界坐标系点的映射关系
  // - 像素中心点的世界坐标为(center_x,center_y)
  // - 像素的col递增方向为世界坐标的x方向
  // - 像素的row递减方向为世界坐标的y方向
  // double center_x = (max_x + min_x) / 2.0;
  // double center_y = (max_y + min_y) / 2.0;

  // o 需要首先涂上rigid的颜色,然后将ground覆盖上一层白色
  // ? lio_sam地面点云保存在哪里？？？
  float min_intensity = 1000;
  float max_intensity = -1000;
  for (int i = 0; i < pc_all->points.size(); i++) {
    PointType pt = pc_all->points[i];
    int col = width / 2 + (pt.x - center_x) / resolution;
    int row = height / 2 - (pt.y - center_y) / resolution;
    // 将这个像素的周围的5*5区域全部涂黑
    for (int r_index = row - 2; r_index <= row + 2; r_index++) {
      for (int c_index = col - 2; c_index <= col + 2; c_index++) {
        if (r_index >= 0 && r_index <= height - 1 && c_index >= 0 &&
            c_index <= width - 1) {
          cv_map.at<uchar>(r_index, c_index) = int(pt.intensity);
          mask.at<uchar>(r_index, c_index) = 255;
          min_intensity = std::min(min_intensity, pt.intensity);
          max_intensity = std::max(max_intensity, pt.intensity);
        }
      }
    }
  }
  std::cout << "intensity range is " << min_intensity << " --> "
            << max_intensity << std::endl;
  // o 转换色系, 得到渲染后的图像
  cv::Mat im_color(cv_map.size(), CV_8UC3, cv::Scalar::all(255));
  cv::applyColorMap(cv_map, im_color, cv::COLORMAP_HSV);
  // o 定义roi为整体的一部分(加上border就是整体了)
  cv::Mat im_total(height + 2 * border_size, width + 2 * border_size, CV_8UC3,
                   cv::Scalar::all(255));
  cv::Mat im_color_roi =
      im_total(cv::Rect(border_size, border_size, cv_map.cols, cv_map.rows));
  im_color.copyTo(im_color_roi, mask);

  // o 将轨迹范围内表示为白色
  // if (0) {
  //   for (int i = 0; i < traj->points.size(); i++) {
  //     PointType pt = traj->points[i];
  //     int col = width / 2 + (pt.x - center_x) / resolution;
  //     int row = height / 2 - (pt.y - center_y) / resolution;
  //     // 定义点x+-0.5m, y+-0.5为空闲区域
  //     int range = mapping_traj_tollerance / resolution;
  //     for (int r_index = row - range; r_index <= row + range; r_index++) {
  //       for (int c_index = col - range; c_index <= col + range; c_index++) {
  //         if (r_index >= 0 && r_index <= height - 1 && c_index >= 0 &&
  //             c_index <= width - 1) {
  //           im_color_roi.at<cv::Vec3b>(r_index, c_index)[0] = 254;
  //           im_color_roi.at<cv::Vec3b>(r_index, c_index)[1] = 254;
  //           im_color_roi.at<cv::Vec3b>(r_index, c_index)[2] = 254;
  //         }
  //       }
  //     }
  //   }
  // }
  time_t curr_time;
  tm *curr_tm;
  char date_string[100];
  time(&curr_time);
  curr_tm = localtime(&curr_time);

  strftime(date_string, 50, "%Y_%m_%d", curr_tm);
  // 添加地图信息
  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2, 8);
  // string msg_info = "name: map.jpg";
  // cv::putText(im_total, msg_info, cvPoint(10, 30), cv::FONT_HERSHEY_PLAIN,
  // 2,
  //             cvScalar(0, 0, 255), 2);
  // msg_info = string("date: ") + date_string;
  // cv::putText(im_total, msg_info, cvPoint(10, 60), cv::FONT_HERSHEY_PLAIN,
  // 2,
  //             cvScalar(0, 0, 255), 2);

  // 4. 制作png文件
  std::string map2d_png_file = file_pure_path_ + ".jpg";
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  cv::imwrite(map2d_png_file, im_total, compression_params);

  // 5. 保存yaml文件
  std::string map_yaml_file = file_pure_path_ + ".yaml";
  FILE *yaml = fopen(map_yaml_file.c_str(), "w");
  // std::ofstream fout(map_yaml_file);
  fprintf(yaml,
          "image: map.jpg\nresolution: %f\norigin: [%f, %f, 0]\nnegate: "
          "0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          map2d_png_file, resolution, -center_corr_x, -center_corr_y);
  // fout << yaml_node_doc;
  fclose(yaml);
  std::cout << "save jpg and yaml end!!!!" << std::endl;
  // return true;
}
