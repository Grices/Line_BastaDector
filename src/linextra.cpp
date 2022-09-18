#include <iostream>
#include <fstream>
#include "include/resbyfit.h"
#include "include/resbyvec.h"
#include "include/dataread.h"

class Extrali
{
    public:
        attribute::Points* target;
    public:
        Extrali(std::vector<attribute::Points>& scanpoints)
        {
            this->target = getGoal(getVect(scanpoints));
        }
}; 

int main()
{
    std::string filepath = "/home/penghua/BasestaDect/src/lds_pointdz.bin";
    if(filepath.empty()) 
    {
        return 0;
    }
    else
    {
        ReadtoPoints input_points(filepath);
        std::cout << "input_fps size: " << input_points.mf_points.size() << std::endl;
        for(auto i = input_points.mf_points.begin(); i != input_points.mf_points.end(); i++)
        {
            std::cout << "point size: " << i->size() << std::endl;
            Extrali ptg(*i);
            if(ptg.target != NULL)
            {
                std::cout << "GOAL: " << ptg.target->x << ", " << ptg.target->y << std::endl;
            }
            else
            {
                std::cout << "There is no target" << std::endl;
            }
        }
    }

    return 0;
}