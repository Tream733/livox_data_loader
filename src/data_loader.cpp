#include "data_loader.hpp"

using namespace std::chrono_literals;
using namespace std::filesystem;

DataLoader::DataLoader()
: Node("publisher_node"), file_index_(0)
{
  // publisher pointcloud
  publisher_point_cloud_front_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/point_cloud_front", 10);
  publisher_point_cloud_left_front_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/point_cloud_left_front", 10);
  publisher_point_cloud_left_rear_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/point_cloud_left_rear", 10);
  publisher_point_cloud_right_rear_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/point_cloud_right_rear", 10);
  publisher_point_cloud_right_front_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/point_cloud_right_front", 10);
  publisher_point_cloud_tele_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/point_cloud_tele", 10);
  // publisher image
  publisher_image_front_ = this->create_publisher<sensor_msgs::msg::Image>("livox/image_front", 10);
  publisher_image_left_front_ = this->create_publisher<sensor_msgs::msg::Image>("livox/image_left_front", 10);
  publisher_image_left_rear_ = this->create_publisher<sensor_msgs::msg::Image>("livox/image_left_rear", 10);
  publisher_image_right_rear_ = this->create_publisher<sensor_msgs::msg::Image>("livox/image_right_rear", 10);
  publisher_image_right_front_ = this->create_publisher<sensor_msgs::msg::Image>("livox/image_right_front", 10);
  publisher_image_tele_ = this->create_publisher<sensor_msgs::msg::Image>("livox/image_tele", 10);
  this->declare_parameter<std::string>("yaml_path", "config.yml");
  // this->get_parameter("yaml_path",yaml_path_);
  initFilePath("/home/txy/workspace/param/config.yml");
  // initFilePath(yaml_path_);
  createPublishersDataFileNames();
  timer_ = this->create_wall_timer(
    100ms, std::bind(&DataLoader::onTimerCallback, this));
}

DataLoader:: ~DataLoader()
{
  
}


void DataLoader::onTimerCallback()
{
  // 01- POINTCLOUD  front left_front left_rear right_rear right_front tele //
  // point_cloud_front
  auto point_cloud_front = std::make_unique<sensor_msgs::msg::PointCloud2>();
  std::string path_point_cloud_front = path_point_cloud_front_ + file_names_point_cloud_front_[file_index_];
  convertPclToPointcloud2(*point_cloud_front,path_point_cloud_front);
  // point_cloud_left_front
  auto point_cloud_left_front = std::make_unique<sensor_msgs::msg::PointCloud2>();
  std::string path_point_cloud_left_front = path_point_cloud_left_front_ + file_names_point_cloud_left_front_[file_index_];
  convertPclToPointcloud2(*point_cloud_left_front,path_point_cloud_left_front);
  // point_point_cloud_left_rear
  auto point_point_cloud_left_rear = std::make_unique<sensor_msgs::msg::PointCloud2>();
  std::string path_point_point_cloud_left_rear = path_point_point_cloud_left_rear_ + file_names_point_cloud_left_rear_[file_index_];
  convertPclToPointcloud2(*point_point_cloud_left_rear,path_point_point_cloud_left_rear);
  // point_cloud_right_rear
  auto point_cloud_right_rear = std::make_unique<sensor_msgs::msg::PointCloud2>();
  std::string path_point_cloud_right_rear = path_point_cloud_right_rear_ + file_names_point_cloud_right_rear_[file_index_];
  convertPclToPointcloud2(*point_cloud_right_rear,path_point_cloud_right_rear);
  // point_cloud_right_front
  auto point_cloud_right_front = std::make_unique<sensor_msgs::msg::PointCloud2>();
  std::string path_point_cloud_right_front = path_point_cloud_right_front_ + file_names_point_cloud_right_front_[file_index_];
  convertPclToPointcloud2(*point_cloud_right_front,path_point_cloud_right_front);
  // point_cloud_tele
  auto point_cloud_tele = std::make_unique<sensor_msgs::msg::PointCloud2>();
  std::string path_point_cloud_tele = path_point_cloud_tele_ + file_names_point_cloud_tele_[file_index_];
  convertPclToPointcloud2(*point_cloud_tele,path_point_cloud_front);

  // 02- IMAGE  front left_front left_rear right_rear right_front tele   //
  // image_front
  int index_ns = file_names_image_front_[file_index_].find_last_of(".");
  int index_s =  file_names_image_front_[file_index_].find_first_of("_");
  std::string time_str = file_names_image_front_[file_index_].substr(0,index_ns);
  std::string time_s = time_str.substr(0,index_s);
  std::string time_ns = time_str.substr(index_s+1,-1);
  std::string time = time_s + time_ns;
  // signed long int tt = strtol(time.c_str(), nullptr,10);
  stamp_ = rclcpp::Time(std::stoi(time_s),std::stoi(time_ns));
  // stamp_ = rclcpp::Time(tt);
  auto image_front = std::make_unique<sensor_msgs::msg::Image>();
  std::string path_image_front = path_image_front_ + file_names_image_front_[file_index_];
  convertImageToMsg(*image_front, path_image_front);
  // image_left_front
  auto image_left_front = std::make_unique<sensor_msgs::msg::Image>();
  std::string path_image_left_front = path_image_left_front_ + file_names_image_left_front_[file_index_];
  convertImageToMsg(*image_left_front, path_image_left_front);
  // image_left_rear
  auto image_left_rear = std::make_unique<sensor_msgs::msg::Image>();
  std::string path_image_left_rear = path_image_left_rear_ + file_names_image_left_rear_[file_index_];
  convertImageToMsg(*image_left_rear, path_image_left_rear);
  // path_image_right_rear_
  auto image_right_rear = std::make_unique<sensor_msgs::msg::Image>();
  std::string path_image_right_rear = path_image_right_rear_ + file_names_image_right_rear_[file_index_];
  convertImageToMsg(*image_right_rear, path_image_right_rear);
  // image_right_front
  auto image_right_front = std::make_unique<sensor_msgs::msg::Image>();
  std::string path_image_right_front = path_image_right_front_ + file_names_image_right_front_[file_index_];
  convertImageToMsg(*image_right_front, path_image_right_front);
  // image_tele
  auto image_tele = std::make_unique<sensor_msgs::msg::Image>();
  std::string path_image_tele = path_image_tele_ + file_names_image_tele_[file_index_];
  convertImageToMsg(*image_tele, path_image_tele);
    
  // 3 publisher
  // pointcloud
  publisher_point_cloud_front_->publish(std::move(point_cloud_front));
  publisher_point_cloud_left_front_->publish(std::move(point_cloud_left_front));
  publisher_point_cloud_left_rear_->publish(std::move(point_point_cloud_left_rear));
  publisher_point_cloud_right_rear_->publish(std::move(point_cloud_right_rear));
  publisher_point_cloud_right_front_->publish(std::move(point_cloud_right_front));
  publisher_point_cloud_tele_->publish(std::move(point_cloud_tele));

  // image
  publisher_image_front_->publish(std::move(image_front));
  publisher_image_left_front_->publish(std::move(image_left_front));
  publisher_image_left_rear_->publish(std::move(image_left_rear));
  publisher_image_right_rear_->publish(std::move(image_right_rear));
  publisher_image_right_front_->publish(std::move(image_right_front));
  publisher_image_tele_->publish(std::move(image_tele));
  
  file_index_++;
  std::cout<<"  index    "<< file_index_<<std::endl;
}

void DataLoader::convertPclToPointcloud2(sensor_msgs::msg::PointCloud2 & msg,const std::string path)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  // if (pcl::io::loadPCDFile<pcl::PointXYZI> (path, cloud) == -1)
  // {
  //   //* load the file
  //   RCLCPP_INFO(this->get_logger(), "Could not read Velodyne's point cloud. Check your file path!");
  //   exit(EXIT_FAILURE);
  // }
  std::ifstream in;
  in.open(path);
  int cunt = 0;
  std::string line;
  while(!in.eof()){
        std::getline(in,line);
        cunt += 1;
        if (cunt > 10)
            break;

    }
  // float x , y ,z, i;
  while(!in.eof())
  {   pcl::PointXYZI point;
        in >> point.x >> point.y >> point.z >> point.intensity;
        getline(in, line);
        if (in.fail())
                    break;
                //out << x << " " << y << " " << z << endl;
        //  point.x = x;
        //  point.y = y;
        //  point.z = z;
        //  point.intensity = i;
         cloud.push_back(point);
  }
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "livox";
  msg.header.stamp = stamp_;
}


void DataLoader::loadAscPts(char* filename,pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  std::ifstream in;
	in.open(filename);
  if(!in)
  {
    std::cerr<< "File"<<filename<<" cloud not be opened"<<std::endl;
  }
  std::string line;

}


void DataLoader::initFilePath(std::string data_path)
{
  YAML::Node config;
  try
  {
        config = YAML::LoadFile(data_path);
  }
  catch(YAML::BadFile e)
  {
        RCLCPP_ERROR(this->get_logger(),"load yaml error.");
        return;
  }
  if(config["data_path"]){
    // base path
    data_path_ = config["data_path"].as<std::string>();
  }


  
  // pointcloud path
  path_point_cloud_front_ = data_path_ + "lidar/1/";
  path_point_cloud_left_front_ = data_path_ + "lidar/2/";
  path_point_point_cloud_left_rear_ = data_path_ + "lidar/3/";
  path_point_cloud_right_rear_  =  data_path_ + "lidar/4/"; 
  path_point_cloud_right_front_ = data_path_ + "lidar/5/";
  path_point_cloud_tele_ = data_path_ + "lidar/6/";
  // image path
  path_image_front_ = data_path_ + "image/1/";
  path_image_left_front_ = data_path_ + "image/2/";
  path_image_left_rear_ = data_path_ + "image/3/";
  path_image_right_rear_ = data_path_ + "image/4/";
  path_image_right_front_ = data_path_ + "image/5/";
  path_image_tele_ = data_path_ + "image/6/";
}

std::string DataLoader::getPath(DataLoader::PublisherType publisher_type)
{
  RCLCPP_INFO(this->get_logger(), "get_path: '%i'", publisher_type);
  std::string path;
  if (publisher_type == DataLoader::PublisherType::POINT_CLOUD_FRONT){
    path = path_point_cloud_front_;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_LEFT_FRONT){
    path = path_point_cloud_left_front_;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_LEFT_REAR){
    path = path_point_point_cloud_left_rear_;
  } else if(publisher_type ==DataLoader::PublisherType::POINT_CLOUD_RIGHT_REAR) {
    path = path_point_cloud_right_rear_;
  } else if(publisher_type ==DataLoader::PublisherType::POINT_CLOUD_RIGHT_FRONT) {
    path = path_point_cloud_right_front_;
  } else if(publisher_type ==DataLoader::PublisherType::POINT_CLOUD_TELE) {
    path = path_point_cloud_tele_;
  } else if(publisher_type ==DataLoader::PublisherType::IMAGE_FRONT) {
    path = path_image_front_;
  } else if(publisher_type ==DataLoader::PublisherType::IMAGE_LEFT_FRONT) {
    path = path_image_left_front_;
  } else if(publisher_type ==DataLoader::PublisherType::IMAGE_LEFT_REAR) {
    path = path_image_left_rear_;
  } else if(publisher_type ==DataLoader::PublisherType::IMAGE_RIGHT_REAR) {
    path = path_image_right_rear_;
  } else if(publisher_type ==DataLoader::PublisherType::IMAGE_RIGHT_FRONT) {
    path = path_image_right_front_;
  } else if(publisher_type ==DataLoader::PublisherType::IMAGE_TELE) {
    path = path_image_tele_;
  } 
  return path;
}

std::vector<std::string> DataLoader::getFilenames(PublisherType publisher_type)
{
  if (publisher_type == DataLoader::PublisherType::POINT_CLOUD_FRONT){
     return file_names_point_cloud_front_;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_LEFT_FRONT){
     return file_names_point_cloud_left_front_;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_LEFT_REAR){
     return file_names_point_cloud_left_rear_;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_RIGHT_REAR){
     return file_names_point_cloud_right_rear_;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_RIGHT_FRONT){
     return file_names_point_cloud_right_front_;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_TELE){
     return file_names_point_cloud_tele_;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_FRONT){
     return file_names_image_front_;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_LEFT_FRONT){
     return file_names_image_left_front_;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_LEFT_REAR){
     return file_names_image_left_rear_;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_RIGHT_REAR){
     return file_names_image_right_rear_;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_RIGHT_FRONT){
     return file_names_image_right_front_;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_TELE){
     return file_names_image_tele_;
  }
}

void DataLoader::setFilenames(PublisherType publisher_type, std::vector<std::string> file_names)
{
  if (publisher_type == DataLoader::PublisherType::POINT_CLOUD_FRONT){
      file_names_point_cloud_front_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_LEFT_FRONT){
      file_names_point_cloud_left_front_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_LEFT_REAR){
      file_names_point_cloud_left_rear_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_RIGHT_REAR){
      file_names_point_cloud_right_rear_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_RIGHT_FRONT){
      file_names_point_cloud_right_front_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::POINT_CLOUD_TELE){
      file_names_point_cloud_tele_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_FRONT){
      file_names_image_front_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_LEFT_FRONT){
      file_names_image_left_front_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_LEFT_REAR){
      file_names_image_left_rear_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_RIGHT_REAR){
      file_names_image_right_rear_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_RIGHT_FRONT){
      file_names_image_right_front_= file_names;
  }else if(publisher_type == DataLoader::PublisherType::IMAGE_TELE){
      file_names_image_tele_= file_names;
  }
}

void DataLoader::createPublishersDataFileNames()
{
  for ( int type_index = 0; type_index != 12; type_index++ )
  {
    DataLoader::PublisherType type = static_cast<DataLoader::PublisherType>(type_index);
    std::vector<std::string> file_names = getFilenames(type);

   try
   {
      for (const auto & entry : directory_iterator(getPath(type))){
        if (entry.is_regular_file()) {
            file_names.push_back(entry.path().filename());
        }
      }

      //Order lidar file names
      std::sort(file_names.begin(), file_names.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs  < rhs ;
            });
      setFilenames(type, file_names);
    }catch (const filesystem_error& e)
    {
        RCLCPP_ERROR(this->get_logger(), "File path not found.");
    }
  }
}

void DataLoader::convertImageToMsg(sensor_msgs::msg::Image & msg, const std::string path  )
{
  cv::Mat frame;
  frame = cv::imread(path);
  // std::cout<<frame.size()<<std::endl;
  if (frame.empty())                      // Check for invalid input
  {
    // std::cout<< path<<std::endl;
    RCLCPP_ERROR(this->get_logger(), "Image does not exist. Check your files path!");
    rclcpp::shutdown();
  }

  msg.height = frame.rows;
  msg.width = frame.cols;
  std::string type = matType2encoding(frame.type());
  msg.encoding = type;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = "livox";
  msg.header.stamp = stamp_;
}

std::string DataLoader::matType2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}
