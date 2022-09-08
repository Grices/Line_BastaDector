#ifndef _ATTRIBUTE_H_
#define _ATTRIBUTE_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <list>

namespace attribute
{
class Points
{
    public:
        float x, y, dis, theta, power;
    public:
        Points() {}
        Points(float _x, float _y, float _dis, float _the, float _pow) : x(_x), y(_y), dis(_dis), theta(_the), power(_pow) {}
        // Points(float _x, float _y) : x(_x), y(_y) {}
        // Points(const Points& _poi) : x(_poi.x), y(_poi.y) {} 
        ~Points() {}

    public:
        static float euclid_Dis(const Points& p, const Points& q)
        {
            return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2));
        }
        static bool chebyshev_Dis(const Points& p, const Points& q, float thres)
        {
            return (fabs(p.x - q.x) - fabs(p.y - q.y)) < thres ? true : false;
        }        
};

class Line
{
    public:
        float A, B, C;
        float slope, sigma; 
        std::vector<Points> combine;

    public:
        // Line() : A(0), B(0), C(0), slope(0), sigma(0) {}
        Line(float _A, float _B, float _C) : A(_A), B(_B), C(_C) {}
        ~Line() {}

    public:

        static float length(const Line& line)
        {
            return Points::euclid_Dis(line.combine.back(), line.combine.front());
        }
        static std::array<float, 2> vect(const Line& line)
        {
            return {(line.combine.back().x - line.combine.front().x), (line.combine.back().y - line.combine.front().y)};
        }
        static Points midpoi(const Line& line)
        {
            return line.combine.at(line.combine.size() / 2);
        }
};

class Scancluster
{
    public:
        std::list<std::vector<Points>> cluster;

    public:
        Scancluster(const std::vector<Points>& poi, const float euci_thres)
        {
            std::vector<attribute::Points> tmp;
            tmp.reserve(poi.size());

            for(auto i = poi.begin() + 1; i != poi.end(); i++)
            {
                if((*(i - 1)).dis > 2000 || (*i).dis > 2000) { continue; }
                else
                {
                    tmp.push_back(*(i - 1));
                    tmp.push_back(*i);

                    if((i == poi.end() - 1) && (!tmp.empty()))
                    {
                        cluster.push_back(tmp);
                        break;
                    }
                    else if(Points::euclid_Dis(*i, *(i - 1)) > euci_thres)
                    {
                        tmp.pop_back();
                        cluster.push_back(tmp);
                        tmp.clear();
                        continue;
                    }
                    else
                    { continue; }
                }
            }
        }

        ~Scancluster() {}
    public:
        void delSmalldis(const unsigned int min_num)
        {
            std::list<std::vector<attribute::Points>>::iterator clu_it = this->cluster.begin();
            while (clu_it != (this->cluster).end())
            {
                if(clu_it->size() < min_num)
                { cluster.erase(clu_it++); }
                else
                { ++clu_it; }
            }
        }
};

float linepoi_Dis(const Line& line, const Points& point)
{
    return fabs((line.A * point.x + line.B * point.y + line.C) / sqrt(pow(line.A, 2) + pow(line.B, 2)));
}

std::vector<Points> preSmooth(std::vector<Points>& prepoi)
{
    for(std::vector<attribute::Points>::iterator poi_smo = prepoi.begin() + 1; poi_smo != prepoi.end(); poi_smo++)
    {
        // if((*poi_smo).chebyshev_Dis(*(poi_smo - 1), 15.0))
        if(Points::chebyshev_Dis(*poi_smo, *(poi_smo - 1), 20.0))
        {
            (*poi_smo).x = (*(poi_smo - 1)).x;
        }
        // if(!((*poi_smo).chebyshev_Dis(*(poi_smo - 1), -15.0)))
        if(!Points::chebyshev_Dis(*poi_smo, *(poi_smo - 1), -20.0))
        {
            (*poi_smo).y = (*(poi_smo - 1)).y;
        }
    }
    std::vector<attribute::Points>::iterator poi_del = prepoi.begin() + 1;
    while(poi_del < prepoi.end())
    {
        if(((*poi_del).x == (*(poi_del - 1)).x) && ((*(poi_del)).y == (*(poi_del - 1)).y))
        {
            poi_del = prepoi.erase(poi_del);
        }
        else
        {
            ++poi_del;
        }
    }
    return prepoi;
}

}

#endif