# livox_data_loader
load livox dataset and publish by sensor msg on ros2
工作流程：
  1、下载livox数据集，配置数据集路径；
  2、编译该工程，运行该工程。
工程功能描述：
  将大疆数据集中的点云文件PCD和图像文件JPG对齐，并回放到ros2中；
  图像数据：6路摄像头，数据类型为：sensor_msgs::msg::Image；
  激光数据：6路激光雷达，数据类型为：sensor_msgs::msg::PointCloud2；
