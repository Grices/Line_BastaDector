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
        float x, y;
        // float dis = 0, theta = 0, power = 0;
    public:
        Points() {}
        Points(float _x, float _y) : x(_x), y(_y) {}
        // Points(float _x, float _y, float _dis, float _the, float _pow) : x(_x), y(_y), dis(_dis), theta(_the), power(_pow) {}
        // Points(const Points& _poi) : x(_poi.x), y(_poi.y) {} 
        ~Points() {}

    public:
        static float euclid_Dis(const Points& p, const Points& q)
        {
            return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2));
        }

        // static bool chebyshev_Dis(const Points& p, const Points& q, float thres)
        // {
        //     return (fabs(p.x - q.x) - fabs(p.y - q.y)) < thres ? true : false;
        // }

        static float cheby_Differ(const Points& p, const Points& q)
        {
            return fabs(p.x - q.x) - fabs(p.y - q.y);
        } 

        static float cheby_Propor(const Points& p, const Points& q)
        {
            return fabs(p.y - q.y) / fabs(p.x - q.x);
        }    

};

class Line
{
    public:
        float A, B, C;
        float slope, sigma; 
        std::vector<Points> combine;

    public:
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

class Vect
{
    public:
        float length;
        std::array<float, 2> vec;
        std::vector<Points> com;
    public:
        Vect(const Points& v_beg, const Points& v_end) 
        {
            this->vec = {v_end.x - v_beg.x, v_end.y - v_beg.y};
            this->length = sqrt(pow(v_end.x - v_beg.x, 2) + pow(v_end.y - v_beg.y, 2));
        } 
        Vect(std::vector<Points>& v_pp)
        {
            this->com = v_pp;
            this->vec = {v_pp.back().x - v_pp.front().x, v_pp.back().y - v_pp.front().y};
            this->length = sqrt(pow(v_pp.back().x - v_pp.front().x, 2) + pow(v_pp.back().y - v_pp.front().y, 2));
        }
        ~Vect()
        {
            
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

            for(auto i = poi.begin(); i != poi.end(); i++)
            {
                // float dis = pow(i->x, 2) + pow(i->y, 2);
                if(abs(i->x) > 2000 || abs(i->y) > 2000) { continue; }
                tmp.push_back(*i);
                // tmp.push_back(*i);
                for(auto j = i + 1; j < poi.end(); j++)
                {    
                    tmp.push_back(*j);   
                    if((j == poi.end() - 1) && (!tmp.empty()))
                    {
                        cluster.push_back(tmp);
                        i = j;
                        break;
                    }
                    else if(Points::euclid_Dis(*j, *(j - 1)) > euci_thres)
                    {
                        tmp.pop_back();
                        cluster.push_back(tmp);
                        tmp.clear();
                        i = j - 1;
                        break;
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

/* 点线距离 */
float linepoi_Dis(const Line& line, const Points& point)
{
    return fabs((line.A * point.x + line.B * point.y + line.C) / sqrt(pow(line.A, 2) + pow(line.B, 2)));
}

/* 锐化平滑函数 */
std::vector<Points> preSmooth(std::vector<Points>& prepoi, const float smo_thre)
{
    // for(std::vector<attribute::Points>::iterator poi_smo = prepoi.begin() + 1; poi_smo != prepoi.end(); poi_smo++)
    // {
    //     // if((*poi_smo).chebyshev_Dis(*(poi_smo - 1), 15.0))
    //     if(Points::chebyshev_Dis(*poi_smo, *(poi_smo - 1), smo_thre))
    //     {
    //         (*poi_smo).x = (*(poi_smo - 1)).x;
    //     }
    //     // if(!((*poi_smo).chebyshev_Dis(*(poi_smo - 1), -15.0)))
    //     if(!Points::chebyshev_Dis(*poi_smo, *(poi_smo - 1), -smo_thre))
    //     {
    //         (*poi_smo).y = (*(poi_smo - 1)).y;
    //     }
    // }
    
    attribute::Points tmp_p = *(prepoi.begin() + 1);
    for(std::vector<attribute::Points>::iterator poi_smo = prepoi.begin() + 1; poi_smo != prepoi.end(); poi_smo++)
    {
        tmp_p = *poi_smo;

        if(Points::cheby_Differ(tmp_p, *(poi_smo - 1)) > -smo_thre)
        {
            // tmp_p = *poi_smo;
            (*poi_smo).y = (*(poi_smo - 1)).y;
        }
        
        if(Points::cheby_Differ(tmp_p, *(poi_smo - 1)) < smo_thre)
        {
            // tmp_p = *poi_smo;
            (*poi_smo).x = (*(poi_smo - 1)).x;
        }
    }

    /* 删除重叠点 */
    std::vector<attribute::Points>::iterator poi_del = prepoi.begin() + 1;
    while(poi_del < prepoi.end())
    {
        // if((poi_del->x - (poi_del - 1)->x < 0.0001f) && ((poi_del)->y - (poi_del - 1)->y < 0.0001f))
        if(((*poi_del).x == (*(poi_del - 1)).x) && ((*(poi_del)).y == (*(poi_del - 1)).y))
        {
            poi_del = prepoi.erase(poi_del);
        }
        else
        {
            ++poi_del;
        }
    }

    /* 平滑出斜边 */
    for(auto poi_sno = prepoi.begin() + 2; poi_sno < prepoi.end() - 2; poi_sno++)
    {
        
        // if( ((Points::cheby_Dist(*(poi_sno + 1), *(poi_sno - 1)) > 0.90f) && 
        //     (Points::cheby_Dist(*(poi_sno + 1), *(poi_sno - 1)) < 1.10f)) &&
        //     ((Points::cheby_Dist(*(poi_sno + 3), *poi_sno) > 0.90f) &&
        //     (Points::cheby_Dist(*(poi_sno + 3), *poi_sno) < 1.10f)))
        
        // if((Points::cheby_Dist(*(poi_sno + 1), *(poi_sno - 1)) > 0.90f) && 
        //     (Points::cheby_Dist(*(poi_sno + 1), *(poi_sno - 1)) < 1.10f))
        float poi_sno1_judge = Points::cheby_Propor(*(poi_sno + 1), *(poi_sno - 1));
        float poi_sno2_judge = Points::cheby_Propor(*(poi_sno + 2), *poi_sno);
        float poi_sno3_judge = Points::cheby_Propor(*poi_sno, *(poi_sno - 2));
        if((poi_sno1_judge > 0.90f && poi_sno1_judge < 1.10f) && (poi_sno2_judge > 0.90f && poi_sno2_judge < 1.10f) && (poi_sno3_judge > 0.90f && poi_sno3_judge < 1.10f))
        // if( ((Points::cheby_Dist(*(poi_sno + 1), *(poi_sno - 1)) > 0.90f) && (Points::cheby_Dist(*(poi_sno + 1), *(poi_sno - 1)) < 1.10f)) && (((Points::cheby_Dist(*(poi_sno + 2), *poi_sno) > 0.90f) && (Points::cheby_Dist(*(poi_sno + 2), *poi_sno) < 1.10f)) && ((Points::cheby_Dist(*poi_sno, *(poi_sno - 2)) > 0.90f) && (Points::cheby_Dist(*poi_sno, *(poi_sno - 2)) < 1.10f))))
        {
            // std::cout << "cheby2" << std::endl;
            poi_sno->y = ((poi_sno - 1)->y + (poi_sno + 1)->y) / 2.0f;
            poi_sno->x = ((poi_sno - 1)->x + (poi_sno + 1)->x) / 2.0f;
            // tmp_p = *(poi_smo + 1);
        }
    }

    // 根据角度调整
    // attribute::Points tmp_poi;
    // for (std::vector<attribute::Points>::iterator poi_sno = prepoi.begin() + 1; poi_sno != prepoi.end(); poi_sno++)
    // {
    //     std::cout << Points::cheby_Dist(*poi_sno, *(poi_sno - 1)) << std::endl;
    //     tmp_poi = *poi_sno;

        // if((Points::cheby_Dist(tmp_poi, *(poi_sno - 1)) >= 0) && (Points::cheby_Dist(tmp_poi, *(poi_sno - 1)) <= 0.57735f))
        // {
        //     // std::cout << "cheby1" << std::endl;
        //     (*poi_sno).y = (*(poi_sno - 1)).y;
        // }
        // if((Points::cheby_Dist(tmp_poi, *(poi_sno - 1)) > 0.57735f) && (Points::cheby_Dist(tmp_poi, *(poi_sno - 1)) <= 1.73205f))
        // {
        //     // std::cout << "cheby2" << std::endl;
        //     (*poi_sno).y = (*(poi_sno - 1)).y + abs((*poi_sno).x - (*(poi_sno - 1)).x);
        // }
        // else if(Points::cheby_Dist(tmp_poi, *(poi_sno - 1)) > 1.73205f) // && (Points::cheby_Dist(tmp_poi, *(poi_sno - 1)) <= 57.28996f))
        // {
        //     // std::cout << "cheby3" << std::endl;
        //     (*poi_sno).x = (*(poi_sno - 1)).x;
        // }
        // else
        // {
        //     // std::cout << "Nan" << std::endl;
        //     continue;
        //     // (*poi_sno).x = (*(poi_sno - 1)).x;
        // }
    // }

    // std::vector<attribute::Points>::iterator poi_del = prepoi.begin() + 1;
    // while(poi_del < prepoi.end())
    // {
    //     if(((*poi_del).x == (*(poi_del - 1)).x) && ((*(poi_del)).y == (*(poi_del - 1)).y))
    //     {
    //         poi_del = prepoi.erase(poi_del);
    //     }
    //     else
    //     {
    //         ++poi_del;
    //     }
    // }
    
    return prepoi;
}

}

#endif