#include <iostream>
#include <fstream>
#include <string>
#include <vector>



class Readdata
{
    public:
        struct ldpoints
        {
            float x, y, dis, theta, power;
            ldpoints() {}
            ldpoints(float _x, float _y, float _dis, float _the, float _pow) : x(_x), y(_y), dis(_dis), theta(_the), power(_pow) {}
        };
    public:
        std::vector<ldpoints> mread_point;
    public:
        void operator()(void)
        {}
};