//
// Created by fusy on 07/04/22.
//

#ifndef DATA_GENERATOR_MACRO_H
#define DATA_GENERATOR_MACRO_H


std::string recipepath = "/home/fusy/Documents/bin2pcd/ros_ws/playback_recipe.txt";
//std::string recipepath = "/home/fusy/Documents/bin2pcd/ros_ws/src/data_generator/playback_recipe.txt";

uint32_t getRGB[] = {
        0x000000 , // 0 : noise
        0x4682b4 , // 1 : animal
        0x0000e6 , // 2 : human.pedestrian.adult, blue
        0x87ceeb , // 3 : human.pedestrian.child Skyblue,
        0x6495ed , // 4 : human.pedestrian.construction_worker  Cornflowerblue
        0xdb7093 , // 5 : human.pedestrian.personal_mobility  Palevioletred
        0x000080 , // 6 : human.pedestrian.police_officer  Navy,
        0xf08080 , // 7 : human.pedestrian.stroller Lightcoral
        0x8a2be2 , // 8 : human.pedestrian.wheelchair Blueviolet
        0x708090 , // 9 : movable_object.barrier Slategrey
        0xd2691e , // 10: movable_object.debris Chocolate
        0x696969 , // 11: movable_object.pushable_pullable Dimgrey
        0x2f4f4f , // 12: movable_object.trafficcone Darkslategrey
        0xbc8f8f , // 13: static_object.bicycle_rack Rosybrown
        0xdc143c , // 14: vehicle.bicycle Crimson
        0xff7f50 , // 15: vehicle.bus.bendy Coral
        0xff4500 , // 16: vehicle.bus.rigid Orangered
        0xff9e00 , // 17: vehicle.car Orange
        0xe99646 , // 18: vehicle.construction Darksalmon
        0xff5300 , // 19: vehicle.emergency.ambulance
        0xffd700 , // 20: vehicle.emergency.police Gold
        0xff3d63 , // 21: vehicle.motorcycle Red
        0xff8c00 , // 22: vehicle.trailer Darkorange
        0xff6347 , // 23: vehicle.truck Tomato
        0x00cfbf , // 24: flat.driveable_surface nuTonomy green
        0xaf004b , // 25: flat.other
        0x4b004b , // 26: flat.sidewalk
        0x70b43c , // 27: flat.terrain
        0xdeb887 , // 28: static.manmade Burlywood
        0xffe4c4 , // 29: static.other Bisque
        0x00af00 , // 20: static.vegetation Green
        0xfff0f5 // 31: vehicle.ego
};

class Point {
public:
    double x, y, z;
    Point(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    }
    Point() {};
};

// Convert float32 to 4 bytes
union
{
    float value;
    uint8_t byte[4];
} floatToBytes;
union
{
    uint32_t value;
    uint8_t byte[4];
} uint32_tToBytes;

void build_msg_Fields(sensor_msgs::PointCloud2 &msg) {
    sensor_msgs::PointField field;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 0;
    field.count = 1;
    field.name = std::string("x");
    msg.fields.push_back(field);

    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 4;
    field.count = 1;
    field.name = std::string("y");
    msg.fields.push_back(field);

    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 8;
    field.count = 1;
    field.name = std::string("z");
    msg.fields.push_back(field);

    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 12;
    field.count = 1;
    field.name = std::string("intensity");
    msg.fields.push_back(field);

    field.datatype = sensor_msgs::PointField::UINT32;
    field.offset = 16;
    field.count = 1;
    field.name = std::string("rgb");
    msg.fields.push_back(field);
//
    msg.is_dense = true;
    msg.header.frame_id = std::string("velodyne");;
    msg.height = 1;
    msg.is_bigendian = false;
    msg.point_step = sizeof(float) * 5;
//    msg.point_step = sizeof(float) * 4;
}

//tf::Matrix3x3 camera_intrinsics(1266.417203046554, 0.0, 816.2670197447984, 0.0, 1266.417203046554, 491.50706579294757, 0.0, 0.0, 1.0);

#endif //DATA_GENERATOR_MACRO_H
