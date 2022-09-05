#include <cmath>
#include "include/attribute.h"

float attribute::Points::euclid_Dis(const Points& p) const
{
    return sqrt(pow(p.x - this->x, 2) + pow(p.y - this->y, 2));
}

bool attribute::Points::chebyshev_Dis(const attribute::Points& p, float thres)
{
    return (fabs(this->x - p.x) - fabs(this->y - p.y)) < thres ? true : false;
}

float attribute::Points::linepoi_Dis(const attribute::Line& line)
{
    return fabs((line.A * this->x + line.B * this->y + line.C) / sqrt(pow(line.A, 2) + pow(line.B, 2)));
}

std::array<float, 2> attribute::Line::vect(void) const
{
    return {((this->combine).back().x - (this->combine).front().x), ((this->combine).back().y - (this->combine).front().y)};
}

float attribute::Line::length(void) const
{
    return (this->combine).back().euclid_Dis((this->combine).front());
}

attribute::Points attribute::Line::midpoi(void)
{
    return (this->combine).at(combine.size() / 2);
}

attribute::Scancluster::Scancluster(const std::vector<Points>& poi, const float euci_thres)
{
    std::vector<attribute::Points> tmp;
    tmp.reserve(poi.size() / 2);
    for(auto i = poi.begin(); i != poi.end(); i++)
    {
        if((*i).dis > 2000) { continue; }
        else
        {
            tmp.push_back(*i);
            if((i == poi.end() -1) && (!tmp.empty()))
            {
                cluster.push_back(tmp);
                continue;
            }
            else if(i->euclid_Dis(*(i - 1)) > euci_thres)
            {
                tmp.pop_back();
                cluster.push_back(tmp);
                tmp.clear();
                i--;
                continue;
            }
            else
            { continue; }
        }
    }
}

void attribute::Scancluster::delSmalldis(const unsigned int min_num)
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

std::vector<attribute::Points> attribute::preSmooth(std::vector<attribute::Points>& prepoi)
{
    for(std::vector<attribute::Points>::iterator poi_smo = prepoi.begin() + 1; poi_smo != prepoi.end(); poi_smo++)
    {
        if((*poi_smo).chebyshev_Dis(*(poi_smo - 1), 15.0))
        {
            (*poi_smo).x = (*(poi_smo)).x;
        }
        if(!((*poi_smo).chebyshev_Dis(*(poi_smo - 1), -15.0)))
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