#ifndef __MORIN_MAP_MERGER_MAP_PARSER_H__
#define __MORIN_MAP_MERGER_MAP_PARSER_H__
#include <iostream>
#include <string>
#include <fstream>
#include <experimental/filesystem>
#include <map_merger/data_types/landmark.h>
using namespace std;

class MapParser{
    private:
        
    public:
        MapParser();

        ~MapParser(); 

        bool generateLandmark(const string& map_folder, vector<landmark>& map_output);

};

#endif