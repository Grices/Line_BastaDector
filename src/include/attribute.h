#ifndef _ATTRIBUTE_H_
#define _ATTRIBUTE_H_

#include <vector>
#include <array>
#include <list>

namespace attribute
{
class Line;

class Points
{
    public:
        float x, y;
        float dis, theta, power;

    public:
        Points() {}
        Points(float _x, float _y) : x(_x), y(_y) {}
        Points(float _x, float _y, float _dis, float _the, float _pow) : x(_x), y(_y), dis(_dis), theta(_the), power(_pow) {}
        Points(const Points& _poi) : x(_poi.x), y(_poi.y) {}
        ~Points() {}
    public:
        float euclid_Dis(const Points& p) const;
        bool chebyshev_Dis(const Points& p, float thres);
        float linepoi_Dis(const Line& line);
};

class Line
{
    public:
        float A, B, C;
        float slope, sigma; 
        std::vector<Points> combine;

    public:
        Line() : A(0), B(0), C(0), slope(0), sigma(0) {}
        Line(float _A, float _B, float _C) : A(_A), B(_B), C(_C) {}
        ~Line() {}
    public:
        std::array<float, 2> vect(void) const;
        float length(void) const;
        Points midpoi(void);
};

class Scancluster
{
    public:
        std::list<std::vector<Points>> cluster;

    public:
        Scancluster(const std::vector<Points>& poi, const float euci_thres);
        ~Scancluster() {}

    public:
        void delSmalldis(const unsigned int min_num);
};

std::vector<Points> preSmooth(std::vector<Points>& prepoi);

}

#endif