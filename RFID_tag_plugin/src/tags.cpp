//
// Created by lybot on 22/3/22.
//
#include "ros/ros.h"
#include "ros/package.h"
#include <iostream>
#include <gazebo/gazebo.hh>
#include "gazebo/sensors/sensors.hh"
#include "gazebo_msgs/SpawnModel.h"
#include "tf2_ros/transform_broadcaster.h"
#include <thread>
#include <cstdio>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <iostream>


#define MAX_TAGS 1000
#define FIXTURES_INMAP 100
#define MAX_LETTERS 100

using namespace std;

static string path_fix;
static string path_white_tag;
static int tags_for_fix;
static int number_fixtures;
static string tag_frame;
static string odom_frame;
//static int tag_count;
class Tag{
public:
    string name;
    int id;
    float x, y, z;
};

class Fixture{
public:
    string name;
    int id;
    float x, y, z;

};
class Spawner{
public:
    int tag_quantity;
    //Tag tags[MAX_TAGS];

    void load_config_data(){
         string package_config_path = ros::package::getPath("gazebo_to_ros_tf");
         string package_map_path = ros::package::getPath("RFID_tag_plugin");
         string yaml_path = package_config_path + "/config/data1.yaml";
         string _map = package_map_path + "/map_layouts/map.txt";
         static const char* path_map = _map.c_str();
         cout << "the package path is "<< path_map << endl;
            //USING YAML LOADING FILE
         //YAML::Node config;
         //config = YAML::LoadFile(yaml_path);
         //path_white_tag = config["white_tag_sdf"].as<string>();
         //path_fix = config["fixture_sdf"].as<string>();
         //tag_frame = config["tag_frame"].as<string>();
         //odom_frame = config["odom_frame_name"].as<string>();
         //cout << "woooooooooooooooooooo "<< path_white_tag << path_fix <<endl;
         //cout << "maaaaaaaaaap " << path_map <<endl;
         //sleep(0.1);
         //YAML::Dump(config);

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
        Spawner start;
        gazebo_msgs::SpawnModel spawn_model;
        spawn_model.request.initial_pose = start.getpose(x, y, z);
        spawn_model.request.model_name = name;
        spawn_model.request.reference_frame = "world";
        //cout << "the pose of the object "<< name << "is " << start.getpose(x, y, z) << endl;

        return spawn_model;
    }

    void spawn_transform(string name, float x, float y, float z){
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = odom_frame;
        msg.child_frame_id = name;
        msg.transform.translation.x = x;
        msg.transform.translation.y = y;
        msg.transform.translation.z = z;
        msg.transform.rotation.x = 0;
        msg.transform.rotation.y = 0;
        msg.transform.rotation.z = 0;
        msg.transform.rotation.w = 1;
        br.sendTransform(msg);
        //cout << "successfully sent transform " << x << " "<< y <<" "<< z <<endl;
    }

    Fixture* read_map_from_file(){
        cout << "start reading file" <<endl;
        string package_map_path = ros::package::getPath("RFID_tag_plugin");
        string _map = package_map_path + "/map_layouts/map.txt";
        const char* const map  = _map.c_str();
        auto *map_square = new Fixture[FIXTURES_INMAP];
        FILE *f = fopen(map, "r");
        int idx = 0;
        while(!feof(f) && idx < FIXTURES_INMAP){
            char line[MAX_LETTERS];
            char name[MAX_LETTERS];
            fgets(line, MAX_LETTERS,f);
            int matched = sscanf(line,"%s%*c%d%*c%f%*c%f%*c%f%*c",name,&map_square[idx].id,&map_square[idx].x,&map_square[idx].y,&map_square[idx].z);
            map_square[idx].name = name;
            if (matched == 5){
                //cout << "successfully read line with values " <<endl;
                //cout << map_square[idx].name << map_square[idx].x << map_square[idx].y << map_square[idx].z << endl;
                number_fixtures++;
            }else{
                ROS_ERROR("ERROR READING LINE FROM FILE");
            }
            idx++;
        }
        return map_square;
    }

    void spawn_fixtures(Spawner *spawn){
        Tag tags[100];
        Fixture *fixtures = spawn->read_map_from_file();
        int idx = 1;
        int tag_counter= 0;
        while( idx < number_fixtures){
            gazebo_msgs::SpawnModel fix = spawn->model_structure(fixtures[idx].name , fixtures[idx].x, fixtures[idx].y, fixtures[idx].z);
            spawn->file_manager(fix, path_fix);
            spawn->spawn_obj(fix);
            spawn->spawn_tags(fixtures[idx].x, fixtures[idx].y, fixtures[idx].z, spawn, &tag_counter, tags);
            cout << "fixture " << fixtures[idx].name << " is spawning " <<endl;
            idx++;
            sleep(0.1);
        }
        cout << "tag quantity is "<< spawn->tag_quantity << endl;
        spawn->spawn_frames(spawn->tag_quantity, tags);
    }

    void spawn_frames(int n, Tag *tags){
        Spawner tag_spawner;
        ros::NodeHandle node;
        ros::Rate rate(20.0);
        while(node.ok()){
            int idx=0;
            while(idx<n){
                //cout << "check array of tags "<< tags[idx].name <<endl;
                tag_spawner.spawn_transform(tags[idx].name , tags[idx].x, tags[idx].y, tags[idx].z);
                idx++;
                rate.sleep();
            }
        }
    }

    void spawn_tags(float x, float y, float z, Spawner *tag_spawner, int* tag_count, Tag *tags){
        int idx=1;
        while (idx <= tags_for_fix){
            //srand( (unsigned)time( NULL ) );  //for creating psuedorandom var.
            tags[*tag_count].name = tag_frame + to_string(*tag_count+1);
            tags[*tag_count].x = x + (2*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX))-1)/2;
            tags[*tag_count].y = y + (2*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX))-1)/2;
            tags[*tag_count].z = z + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))/2;
            gazebo_msgs::SpawnModel tag = tag_spawner->model_structure(tags[*tag_count].name , tags[*tag_count].x, tags[*tag_count].y, tags[*tag_count].z);
            tag_spawner->file_manager(tag, path_white_tag);
            tag_spawner->spawn_obj(tag);
            cout << "tag " << tags[*tag_count].name << " is spawning " <<endl;
            ROS_INFO("at x y z %f %f %f " ,tags[*tag_count].x, tags[*tag_count].y, tags[*tag_count].z);
            idx++;
            (*tag_count)++; //why does this line skip +1 count?
        }
        tag_spawner->tag_quantity = *tag_count;
    }
};
namespace gazebo{
    class RFID_TAG:public SensorPlugin{
    public:
        RFID_TAG():SensorPlugin(){
            printf("############################################################################################################################################\n");
            printf("############################################################################################################################################\n");
            cout << "Tag Plugin has started" << endl;
            ROS_DEBUG("URDF FILE PATH: ");
        }
        public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
            {
                if (_sdf->HasElement("tag_name")){
                    tag_frame = _sdf->Get<string>("tag_name");
                    cout << "TAG NAME: "<<tag_frame<<endl;
                }
                if (_sdf->HasElement("parent_frame")){
                    odom_frame = _sdf->Get<string>("parent_frame");
                    cout << "PARENT_FRAME NAME: "<<odom_frame<<endl;
                }
                if (_sdf->HasElement("tags_per_fix")){
                    tags_for_fix = _sdf->Get<int>("tags_per_fix");
                    cout << "The NUM OF TAGS/FIX: "<<tags_for_fix<<endl;
                }
                if (_sdf->HasElement("fix_sdf_path")){
                    path_fix = _sdf->Get<string>("fix_sdf_path");
                    cout << "FIX SDF PATH: "<<path_fix<<endl;
                }
                if (_sdf->HasElement("tag_sdf_path")){
                    path_white_tag = _sdf->Get<string>("tag_sdf_path");
                    cout << "TAG SDF PATH: "<<path_white_tag<<endl;
                }
                printf("############################################################################################################################################\n");
                printf("############################################################################################################################################\n");
            auto *go = new Spawner();
            thread th(&Spawner::spawn_fixtures,go,go);
            th.detach();
            delete go;
            }
    };
    GZ_REGISTER_SENSOR_PLUGIN(RFID_TAG)
}

//int main(int argc, char **argv) {
//    cout << "Tag Plugin has started" << endl;
//    ros::init(argc, argv, "Tag_Plugin_node");
//    Spawner start;
//    start.spawn_fixtures(&start);
//    //start.read_map_from_file();
//    ROS_DEBUG("URDF FILE PATH: ");
//
//
//    ros::spin();
//    return 0;
//}
