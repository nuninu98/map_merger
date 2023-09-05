#include <map_merger/algorithm/map_parser.h>

using namespace std;

MapParser::MapParser(){

}

MapParser::~MapParser(){

}

bool MapParser::generateLandmark(const string& file_path, landmark& landmark){
    ifstream bin(file_path, ios::in | ios::binary);
    if(!bin.good()){
        cout<<"No Such file: "<<file_path<<endl;
        return false;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    while(bin){
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double intensity = 0.0;
        pcl::PointXYZI point;
        bin.read((char*)&point.x, sizeof(double));
        bin.read((char*)&point.y, sizeof(double));
        bin.read((char*)&point.z, sizeof(double));
        bin.read((char*)&point.intensity, sizeof(double));
    }
    bin.close();
    return true;
}