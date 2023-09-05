#ifndef __MORIN_MAP_MERGER_MAP_PARSER_H__
#define __MORIN_MAP_MERGER_MAP_PARSER_H__
#include <iostream>
#include <string>
#include <fstream>
#include <map_merger/data_types/landmark.h>
using namespace std;

class MapParser{
    private:
        bool generateLandmark(const string& file_path, landmark& landmark);
    public:
        MapParser();

        ~MapParser(); 
};

#endif