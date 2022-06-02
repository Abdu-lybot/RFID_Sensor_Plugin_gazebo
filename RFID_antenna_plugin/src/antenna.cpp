//
// Created by lybot on 25/3/22.
//
#include "ros/ros.h"
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/SpawnModel.h>
#include "tf/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <iostream>
#include <std_msgs/String.h>

#define MAX_TAGS 1000
#define MAX_DETECTION_RANGE 10
#define MAX_PLUGINS 20
#define PI 3.1415
using namespace std;

static string detected_tag_name;
static string detected_tag_colour;
static int antenna_id;
static float beam_width_h ;
static float beam_width_v ;
static float r_zero ;
static string path_yellow_tag;
static string path_blue_tag;
static string path_red_tag;
static string path_purple_tag;
static string name_spawned_obj;
static string target_frame;
static string source_frame;
static string world_frame = "odom_sim";


enum Tag_colors{
    white,
    yellow,
    blue,
    red,
    purple
};

class Tag{
public:
    string name;
    int id;
    float x_world, y_world, z_world;
    float x_antenna, y_antenna, z_antenna;
    Tag_colors color;
    bool spawned {false};
    bool exists {false};

    void build_tag_array(){
        //array of vector of tags?
    }
    bool get_tag_spawn_status(Tag *tag){
        return tag->spawned = true;
    }
    void set_spawned(Tag *tag, bool condition){
        tag->spawned = condition;
    }
    void set_exist(Tag *tag, bool condition){
        tag->exists = condition;
    }
    void init_tag(Tag *tag, string tag_name, float x_world, float y_world, float z_world, float x_antenna, float y_antenna, float z_antenna){
        tag->name = tag_name;
        tag->x_world = x_world;
        tag->y_world = y_world;
        tag->z_world = z_world;
        tag->x_antenna = x_antenna;
        tag->y_antenna = y_antenna;
        tag->z_antenna = z_antenna;
    }
    float calculate_distance(float x, float y, float z){
        float d = sqrt(x*x + y*y);
        return d;
    }
    bool in_range(float x, float y, float z){
        float distance_between_antenna_tag = calculate_distance(x, y, z);
        if (distance_between_antenna_tag < MAX_DETECTION_RANGE){
            return true;
            }else{
            return false;
        }
    }
};
enum antenna_orientation{
    Front,
    Right,
    Left,
    Back
};

class Antenna{
public:
    antenna_orientation antenna_id;
    int start_delay {10};

    string get_color_path_from_sdf(Tag_colors colour) {
        string path;
        switch (colour) {
            case yellow:
                path = path_yellow_tag;
                return path;
            case blue:
                path = path_blue_tag;
                return path;
            case red:
                path = path_red_tag;
                return path;
            case purple:
                path = path_purple_tag;
                return path;
            default:
                cout << "Error choosing color " << endl;
        }
    }
    string get_color_string_from_antenna_id(antenna_orientation orientation) {
        string c_name;
        Tag tag;
        switch (orientation) {
            case Front:
                c_name = "Yellow";
                return c_name;
            case Right:
                c_name = "blue";
                return c_name;
            case Left:
                c_name = "red";
                return c_name;
            case Back:
                c_name = "purple";
                return c_name;
            default:
                cout << "Error choosing color " << endl;
        }
    }
    double normalized_antenna_pattern(float call_elevation_h, float call_elevation_v){
        double d =  pow(cos((PI/2)*(call_elevation_h/beam_width_h)),2.0) * pow(cos((PI/2)*(call_elevation_v/beam_width_v)),2.0);
        return d;
    }
    double x_argument(float tag_ant_dis, float call_elavation_h, float call_elevation_v){
        double d0 = normalized_antenna_pattern(0,0);
        double _x_arg = (pow((d0 * tag_ant_dis),2.0)/pow((normalized_antenna_pattern(call_elavation_h,call_elevation_v))*r_zero,2.0));
        return _x_arg;
    }
    double pdf(float tag_ant_dis, float call_elavation_h, float call_elevation_v){
        if(abs(call_elavation_h) > PI/2){
            return 0;
        }else{
            double _pdf = 2/(1+ pow(3.0, sqrt(x_argument(tag_ant_dis,call_elavation_h,call_elevation_v))));
            return _pdf;
        }
    }
    double pdf_threshold(double pdf){
        float random_v = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float reduced_pdf = pow(1-(1-pdf),1.0); //in python its 1/self.freq*self.standard_time, which values are 1
        if(random_v < reduced_pdf){
            return true;
        }else{
            return false;
        }
    }
    double get_call_elevation_h(float x, float y, float z){
        double call_elevation_h = acos(x/ sqrt(pow(x,2.0)+pow(y,2.0)));
        return call_elevation_h;
    }
    float get_call_elevation_v(float x, float y, float z){
        double call_elevation_v = acos(x/ sqrt(pow(x,2.0)+pow(z,2.0)));
        return call_elevation_v;
    }
};
 class Spawner{
 public:
     int total_detected_tags;
     int yellow_tags;
     int blue_tags;
     int red_tags;
     int purple_tags;

     void load_config_data(){
         string package_path = ros::package::getPath("gazebo_to_ros_tf");
         string yaml_path = package_path + "/config/data.yaml";
         cout << "the package path is "<< yaml_path << endl;
         //USING YAML FILE TO LOAD PARAMETERS
         /*YAML::Node data;
         data = YAML::LoadFile(yaml_path);
         path_white_tag = data["white_tag_sdf"].as<string>();
         path_yellow_tag = data["yellow_tag_sdf"].as<string>();
         path_red_tag = data["red_tag_sdf"].as<string>();
         path_blue_tag = data["blue_tag_sdf"].as<string>();
         path_purple_tag = data["purple_tag_sdf"].as<string>();
         target_frame = data["antenna_frame"].as<string>();
         source_frame = data["tag_frame"].as<string>();
         name_spawned_obj = data["spawned_tag_frame"].as<string>();
         cout << "heeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeere "<< target_frame << " " << source_frame <<endl;
         //YAML::Dump(data);*/

     }

     void spawn_obj(gazebo_msgs::SpawnModel & my_model){
        ros::NodeHandle n;
        ros::service::waitForService("gazebo/spawn_sdf_model");
        ros::ServiceClient spawn_model_client = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        if(spawn_model_client.call(my_model)){
            auto responce = my_model.response;
            ROS_INFO("%s",responce.status_message.c_str());
        }else{
            ROS_ERROR("FAILED TO CALL SERVICE abdu");
        }
    }
     geometry_msgs::Pose getpose(float x ,float y, float z){
         geometry_msgs::Pose obj_pose;
         obj_pose.position.x = x;
         obj_pose.position.y = y;
         obj_pose.position.z = z;
         obj_pose.orientation.x = 0;
         obj_pose.orientation.y = 0;
         obj_pose.orientation.z = 0;
         obj_pose.orientation.w = 1;

         return obj_pose;
     }
     gazebo_msgs::SpawnModel model_structure(string name, float x, float y, float z){
         gazebo_msgs::SpawnModel spawn_model;
         spawn_model.request.initial_pose = getpose(x, y, z);
         spawn_model.request.model_name = name;
         spawn_model.request.reference_frame = "world";
         //cout << "the pose of the object "<< name << "is " << getpose(x, y, z) << endl;

         return spawn_model;
     }
     void file_manager(gazebo_msgs::SpawnModel & my_model, const string& path){
         string line;
         ifstream file(path);
         while (!file.eof()){
             getline(file, line);
             //cout << line<< endl;
             my_model.request.model_xml += line;
         }
         file.close();
     }
     string name_construct(int idx, antenna_orientation ori){
         Antenna antenna;
         string name = detected_tag_name + detected_tag_colour + to_string(idx);
         return name;
     }
     void spawn_tag(Tag *t, Spawner *spawn, Tag_colors paint_color){
         Antenna antenna{};
         cout << "tag with coulor " << paint_color <<endl;
         string path {antenna.get_color_path_from_sdf(paint_color)};
         gazebo_msgs::SpawnModel detected_tag = spawn->model_structure(t->name, t->x_world, t->y_world, t->z_world);
         spawn->file_manager(detected_tag, path);
         spawn->spawn_obj(detected_tag);
         t->set_spawned(t,true);
     }
     //not working?? becuase of buffer?
     geometry_msgs::TransformStamped get_transform(string target, string source, int idx, ros::Time time){
         ros::NodeHandle node;
         tf2_ros::Buffer tfBuffer;
         tf2_ros::TransformListener listen(tfBuffer);
         geometry_msgs::TransformStamped tf;
         tf = tfBuffer.lookupTransform(target, source + to_string(idx+1),time);
         return tf;
     }
     Tag_colors get_colour_from_sdf(){
         if(detected_tag_colour == "blue"){
             return blue;
         }
         if(detected_tag_colour == "yellow"){
             return yellow;
         }
         if(detected_tag_colour == "red"){
             return red;
         }
         if(detected_tag_colour == "purple"){
             return purple;
         }
     }
     void publish_detected_tags (Tag *tag, ros::Publisher detected_tag_pub){
         geometry_msgs::PoseStamped tag_info;
         tag_info.header.frame_id = tag->name;
         tag_info.header.stamp = ros::Time::now();
         tag_info.pose.position.x = tag->x_world;
         tag_info.pose.position.y = tag->y_world;
         tag_info.pose.position.z = tag->z_world;
         detected_tag_pub.publish(tag_info);
     }
     void antenna_start(){
         Spawner spawn;
         load_config_data();
         Tag tag[MAX_TAGS];
         Antenna antenna{};
         sleep(antenna.start_delay);
         //antenna.antenna_id = Back;
         ros::NodeHandle node;
         ros::Publisher detected_tag_pub = node.advertise<geometry_msgs::PoseStamped>("detected_tags",1000);

         tf2_ros::Buffer tfBuffer, tfBuffer2;
         tf2_ros::TransformListener listen(tfBuffer),listen2(tfBuffer2);
         geometry_msgs::TransformStamped tf_tag_antenna, tf_tag_world;
         ros::Rate rate(20.0);
         while(node.ok()){
             try{
                 for(int idx = 0; idx < MAX_TAGS ;idx++){
                     tf_tag_antenna = tfBuffer.lookupTransform(target_frame + to_string(antenna_id), source_frame+ to_string(idx+1),ros::Time(0));
                     tf_tag_world = tfBuffer2.lookupTransform(world_frame, source_frame+ to_string(idx+1),ros::Time(0));
                     if(tf_tag_antenna.transform.translation.x){
                         tag->set_exist(&tag[idx], true);
                         tag->init_tag(&tag[idx], name_construct(idx, static_cast<antenna_orientation>(antenna_id)), tf_tag_world.transform.translation.x, tf_tag_world.transform.translation.y, tf_tag_world.transform.translation.z, tf_tag_antenna.transform.translation.x, tf_tag_antenna.transform.translation.y, tf_tag_antenna.transform.translation.z);
                         if(tag->in_range(tag[idx].x_antenna,tag[idx].y_antenna, tag[idx].z_antenna)){
                             double call_elevation_h = antenna.get_call_elevation_h(tag[idx].x_antenna,tag[idx].y_antenna, tag[idx].z_antenna);
                             double call_elevation_v = antenna.get_call_elevation_v(tag[idx].x_antenna,tag[idx].y_antenna, tag[idx].z_antenna);
                             float dist = tag->calculate_distance(tag[idx].x_antenna,tag[idx].y_antenna, tag[idx].z_antenna);
                             double pdf = antenna.pdf(dist, call_elevation_h, call_elevation_v);
                             if (antenna.pdf_threshold(pdf) && !tag[idx].spawned){
                                 printf("Tag %d at has a probability of %lf",idx,pdf);
                                 printf("Horizontal call_elevation angle: %lf, Vertical call_elevation angle %lf", call_elevation_h, call_elevation_v);
                                 spawn_tag(&tag[idx], &spawn, get_colour_from_sdf());
                                 publish_detected_tags(&tag[idx], detected_tag_pub);
                             }

                         }else{
                             //printf("Tag %d is not in range in distance %f \n", idx+1, tag->calculate_distance(tf.transform.translation.x,tf.transform.translation.y, tf.transform.translation.z));
                         }
                         //printf("RFID tag num %d exists\n", idx+1);
                     }else{
                         tag->set_spawned(&tag[idx], false);
                         //printf("RFID tag num %d does not exist\n", idx+1);
                     }
                     rate.sleep();
                 }
             }
             catch(tf2::TransformException ex){
                 ROS_ERROR("%s", ex.what());
                 ros::Duration(1.0).sleep();
                 continue;
             }
         }
     }
 };

namespace gazebo{
    class RFID_Antenna:public SensorPlugin{
    public:
        RFID_Antenna():SensorPlugin(){
            printf("############################################################################################################################################\n");
            printf("############################################################################################################################################\n");
            printf("Antenna Plugin has started!\n");
        }
        public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
            {
            if (_sdf->HasElement("antenna_name")){
                target_frame = _sdf->Get<string>("antenna_name");
                cout << "ANTENNA NAME: "<<target_frame<<endl;
            }
            if (_sdf->HasElement("tag_name")){
                source_frame = _sdf->Get<string>("tag_name");
                cout << "TAG NAME AND FRAME TO BE DETECTED: "<<source_frame<<endl;
            }
            if (_sdf->HasElement("detected_tag_name")){
                detected_tag_name = _sdf->Get<string>("detected_tag_name");
                cout << "DETECTED TAG NAME: "<<detected_tag_name<<endl;
            }
            if (_sdf->HasElement("detected_tag_colour")){
                detected_tag_colour = _sdf->Get<string>("detected_tag_colour");
                cout << "COLOUR: "<<detected_tag_colour<<endl;
            }
            if (_sdf->HasElement("antenna_id")){
                antenna_id = _sdf->Get<int>("antenna_id");
                cout << "ANTENNA ID: "<<antenna_id<<endl;
            }
            if (_sdf->HasElement("azimuth_beamwidth")){
                beam_width_h = _sdf->Get<float>("azimuth_beamwidth");
                cout << "azimuth_beamwidth: "<<beam_width_h<<endl;
            }
            if (_sdf->HasElement("elevation_beamwidth")){
                beam_width_v = _sdf->Get<float>("elevation_beamwidth");
                cout << "elevation_beamwidth: "<<beam_width_v<<endl;
            }
            if (_sdf->HasElement("rzero")){
                r_zero = _sdf->Get<float>("rzero");
                cout << "R0: "<<r_zero<<endl;
            }
            if (_sdf->HasElement("tag_yellow_sdf_path")){
                path_yellow_tag = _sdf->Get<string>("tag_yellow_sdf_path");
                cout << "YELLOW TAG SDF PATH: "<<path_yellow_tag<<endl;
            }
            if (_sdf->HasElement("tag_blue_sdf_path")){
                path_blue_tag = _sdf->Get<string>("tag_blue_sdf_path");
                cout << "BLUE TAG SDF PATH: "<<path_blue_tag<<endl;
            }
            if (_sdf->HasElement("tag_red_sdf_path")){
                path_red_tag = _sdf->Get<string>("tag_red_sdf_path");
                cout << "RED TAG SDF PATH: "<<path_red_tag<<endl;
            }
            if (_sdf->HasElement("tag_purple_sdf_path")){
                path_purple_tag = _sdf->Get<string>("tag_purple_sdf_path");
                cout << "PURPLE TAG SDF PATH: "<<path_purple_tag<<endl;
            }
                printf("############################################################################################################################################\n");
                printf("############################################################################################################################################\n");
            int idx = rand() % 100 +1;
            Spawner *go = new Spawner[MAX_PLUGINS]();
//            thread th(&Spawner::antenna_start,go,go);
            Spawner spawn2{};
            thread th(&Spawner::antenna_start,spawn2);
            th.detach();
            //delete go;
            }
    };
    GZ_REGISTER_SENSOR_PLUGIN(RFID_Antenna)
};

// int main(int argc, char ** argv){
//     cout << "The Antenna plugin has started"<<endl;
//     ros::init(argc,argv,"Antenna_plugin_node");
//     Tag tag[MAX_TAGS];
//     Antenna antenna;
//     antenna.antenna_id = Back;
//     Spawner spawn{};
//     spawn.antenna_start(tag, spawn, antenna.antenna_id);
//     cout << tag[1].spawned<<endl;
// };;
