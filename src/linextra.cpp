#include <iostream>
#include "include/release.h"
#include "include/dataread.h"

class Extrali
{
    public:
        attribute::Points* target;
    public:
        Extrali(std::vector<attribute::Points>& scanpoints)
        {
            this->target = goalPoint(getTarget(scanpoints));
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
        ReadtoPoints test_points(filepath);
        std::cout << "test_points size: " << test_points.mf_points.size() << std::endl;
        for(auto i = test_points.mf_points.begin(); i != test_points.mf_points.end()-60; i++)
        {
            Extrali ptg(*i);
            if(ptg.target != NULL)
            {
                std::cout << ptg.target->x << ", " << ptg.target->y << std::endl;
            }
            else
            {
                std::cout << "There is no target" << std::endl;
            }
        }
    }

    return 0;
}