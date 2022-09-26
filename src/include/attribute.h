#ifndef _ATTRIBUTE_H_
#define _ATTRIBUTE_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <list>

namespace attribute
{
/* 点类 */
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

        static float cheby_Distance(const Points& p, const Points& q)
        {
            return std::max(fabs(p.x - q.x), fabs(p.y - q.y));
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

/* 直线类 */
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

/* 向量类 */
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

/* 基于引力的簇合并 点数*距离 */
class Gravate
{
    public:
        Points str;
        float M;
    public:
        Gravate(std::vector<Points>& p_M)
        {
            float Mx = 0.0;
            float My = 0.0;
            for(auto it_pi = p_M.begin(); it_pi != p_M.end(); it_pi++)
            {
                Mx += it_pi->x;
                My += it_pi->y;
            }
            Mx /= p_M.size();
            My /= p_M.size();

            Points tmp(Mx, My);
            this->str = tmp;
            this->M = p_M.size();
        }
};

/* 激光点聚类 包含聚类-筛选-簇合并 */
class Sacnpcluter
{
    public:
        std::vector<std::vector<Points>> cluster;

    public:
        // 构造 点分类
        Sacnpcluter(std::vector<Points>& poi, const float euci_thres)
        {
            std::vector<Points> tmp;
            tmp.reserve(poi.size());

            for(auto i = poi.begin(); i != poi.end(); i++)
            {
                if(abs(i->x) > 3000 || abs(i->y) > 3000) { continue; }
                tmp.push_back(*i);

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
        // 删除点数过小的簇
        void deleteSmall(const unsigned int min_num)
        {
            for(auto i = this->cluster.begin(); i != this->cluster.end(); )
            {
                if(i->size() < min_num)
                {
                    i = this->cluster.erase(i);
                }
                else
                {
                    i++;
                }
            }
        }
        // 簇合并
        void mergeCluster(const float mergedis)
        {
            for(auto it_clu = this->cluster.begin(); it_clu < this->cluster.end() - 1; )
            {
                // Gravate tmp(*it_clu);
                // Gravate tnp(*(it_clu + 1));
                // float gra_F = tmp.M * tnp.M / pow(Points::euclid_Dis(tmp.str, tnp.str), 2);

                float dist = Points::euclid_Dis(it_clu->back(), (it_clu + 1)->front());
                if(dist < mergedis)
                {
                    for(auto it_next = (it_clu + 1)->begin(); it_next != (it_clu + 1)->end(); it_next++)
                    {
                        it_clu->push_back(*it_next);
                    }

                    it_clu = cluster.erase(it_clu ++);
                }
                else
                { it_clu++; }
            }
        }
        // 析构
        ~Sacnpcluter(){}
};

/* 旧版本聚类方法 未添加簇合并方法 */
class Scancluster
{
    public:
        std::list<std::vector<Points>> cluster;

    public:
        Scancluster(std::vector<Points>& poi, const float euci_thres)
        {
            std::vector<attribute::Points> tmp;
            tmp.reserve(poi.size());

            for(auto i = poi.begin(); i != poi.end(); i++)
            {
                // float dis = pow(i->x, 2) + pow(i->y, 2);
                if(abs(i->x) > 3000 || abs(i->y) > 3000) { continue; }
                tmp.push_back(*i);
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

/* 滤波函数 高斯&均值 */
std::vector<Points> meanFilter(std::vector<Points>& poi, const int width)
{
    // 首尾均值填充
    // float a_mean_x = 0, a_mean_y = 0, z_mean_x = 0, z_mean_y = 0;
    // for(auto az = poi.begin(); az != poi.begin() + width; az++)
    // {
    //     a_mean_x += az->x;
    //     a_mean_y += az->y;
    // }
    // Points tmp_a(a_mean_x / width, a_mean_y / width);
    // for(auto za = poi.end() - width - 1; za != poi.end(); za++)
    // {
    //     z_mean_x += za->x;
    //     z_mean_y += za->y;
    // }
    // Points tmp_z(z_mean_x / width, z_mean_y / width);
    
    // 首尾循环填充 
    // for(int add = 0; add < width; add++)
    // {
    //     poi.insert(poi.begin(),*(poi.begin() + (add * 2 +  1)));
    //     // poi.insert(poi.begin(),tmp_a);
    //     poi.emplace_back(*(poi.end() - (add * 2 + 1)));
    //     // poi.emplace_back(tmp_z);
    // }

    // 首尾复制填充
    for(int add = 0; add < width; add++)
    {
        poi.insert(poi.begin(), poi.front());
        poi.emplace_back(poi.back());
    }


    /* 高斯滤波 */
    float sig = 1.0f; 
    float PI = 3.141592653589793f;
    float powerarr[width*2+1];
    float guasssum = 0.0f;
    // 高斯模板
    for(int p = 0; p < width*2+1; p++)
    {
        powerarr[p] = (1 / (2*PI*sig*sig)) * exp(-(p-width) * (p-width) / (2*sig*sig));
        guasssum += powerarr[p];
    }
    // 归一化
    for(int q = 0; q < width*2+1; q++)
    {
        powerarr[q] /= guasssum;
    }
    // 高斯平滑
    for(auto i = poi.begin() + width; i < poi.end() - width; i++)
    {
        float guass_x = 0.0f;
        float guass_y = 0.0f;
        // bool break_flag = false;
        int n = 0;
        for(auto j = i - width; j <= i + width; j++, n++)
        {
            // if(j > i - width)
            // {
            //     if(Points::euclid_Dis(*j, *(j-1)) > 100.0f) // 删除离群点
            //     {
            //         poi.erase(j++);
            //         break_flag = true;
            //         break;
            //     }
            // }

            guass_x += powerarr[n] * j->x;
            guass_y += powerarr[n] * j->y;
            
        }
        // if(break_flag) { continue; }
        i->x = guass_x;
        i->y = guass_y;
    } 

    /* 均值滤波 */
    // for(auto i = poi.begin() + width; i < poi.end() - width; i++)
    // {
    //     float mean_x = 0.0f;
    //     float mean_y = 0.0f;
    //     int count = 0;
    //     bool break_flag = false;
    //     for(auto j = i - width; j <= i + width; j++)
    //     {
    //         if(j < i + width)
    //         {
    //             if(Points::euclid_Dis(*j, *(j+1)) > 30.0f)
    //             {
    //                 break_flag = true;
    //                 break;
    //             }
    //         }
    //         count++;
    //         mean_x += j->x;
    //         mean_y += j->y;
    //     }
    //     if(break_flag) { continue; }
    //     mean_x /= count;
    //     mean_y /= count;
    //     i->x = mean_x;
    //     i->y = mean_y;
    // }
    
    return poi;
}

/* 拉普拉斯锐化 */
std::vector<Points> laplaceSharpen(std::vector<Points>& prepoi)
{
    // std::array<float, 3> laparr = {-1, 2, -1};
    std::vector<Points> tmp;
    tmp.reserve(prepoi.size());
    prepoi.insert(prepoi.begin(), prepoi.front());
    prepoi.emplace_back(prepoi.back());
    for(auto p = prepoi.begin() + 1; p != prepoi.end() - 1; p++)
    {
        Points tm(0, 0);
        float det_x = 2 * p->x - (p-1)->x - (p+1)->x;
        if(det_x < 0)
        {
            tm.x = p->x - det_x;
        }
        else
        {
            tm.x = p->x - det_x;
        }
        float det_y = 2 * p->y - (p-1)->y - (p+1)->y;
        if(det_y < 0)
        {
            tm.y = p->y - det_x;
        }
        else
        {
            tm.y = p->y + det_y;
        }
        tmp.emplace_back(tm);
    }
    // tmp.emplace_back(prepoi.back());
    return tmp;
}

/* 降采样 最大&平均池化 */
std::vector<Points> downSampling(std::vector<Points>& prepoi)
{
    // 平均池化
    if(!prepoi.size() % 2)
    {
        prepoi.emplace_back(prepoi.back());
    }
    for(auto i = prepoi.begin(); i < prepoi.end() - 1; i++)
    {
        i->x = (i->x + (i+1)->x) / 2;
        i->y = (i->y + (i+1)->y) / 2;
        prepoi.erase(i + 1);
    }
    // 最大池化
    // for(auto i = prepoi.begin(); i < prepoi.end() - 1; i++)
    // {
    //     bool judge = (fabs(i->x) + fabs(i->y)) < (fabs((i+1)->x), fabs((i+1)->y)) ? true : false;
    //     if(judge)
    //     {
    //         prepoi.erase(i + 1);
    //     }
    //     else
    //     {
    //         prepoi.erase(i);
    //     }
    // }
    return prepoi;
}

/* 自定义锐化 */
// std::vector<Points> preSmooth(std::vector<Points>& prepoi, const float smo_thre)
std::vector<Points> preSmooth(std::vector<Points>& prepoi)
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

    // attribute::Points tmp_p = *(prepoi.begin() + 1);
    int flag = 0;
    for(std::vector<attribute::Points>::iterator poi_smo = prepoi.begin() + 1; poi_smo != prepoi.end(); poi_smo++)
    {
        float cheper = Points::cheby_Propor(*poi_smo, *(poi_smo - 1));
        
        if(cheper > 2.747f)
        {
            poi_smo->x = (poi_smo - 1)->x;
            flag = 1;
        }
        else if(cheper < 0.363f)
        {
            poi_smo->y = (poi_smo - 1)->y;
            flag = 2;
        }
        else
        {
            if(flag == 1)
            {
                poi_smo->x = (poi_smo - 1)->x;
            }
            else if(flag == 2)
            {
                poi_smo->y = (poi_smo - 1)->y;
            }
            else
            {
                flag = 0;
                continue;
            }
        }


        // 比较当前点与前一点的切比雪夫差值
        // float chediff = Points::cheby_Differ(tmp_p, *(poi_smo - 1)); // x-y
        // tmp_p = *(poi_smo + 1); 

        // float chediff = Points::cheby_Differ(*poi_smo, *(poi_smo - 1));
        // if(chediff > smo_thre) // x > y
        // {
        //     // tmp_p = *poi_smo;
        //     poi_smo->y = (poi_smo - 1)->y;
        // }
        // if(chediff < -smo_thre)
        // {
        //     poi_smo->x = (poi_smo - 1)->x;
        // }

        // if(chediff > 0) // x方向差值更大
        // {
        //     // tmp_p = *poi_smo;
        //     poi_smo->y = (poi_smo - 1)->y;
        // }
        // else if(chediff < 0) // y方向差值更大
        // {
        //     // tmp_p = *poi_smo;
        //     poi_smo->x = (poi_smo - 1)->x;
        // }

        // else
        // {
        //     // if(chediff > 0)
        //     // {
        //     //     poi_smo->x -= poi_smo->x - (poi_smo - 1)->x;
        //     // }
        //     // else
        //     // {
        //     //     poi_smo->y -= poi_smo->y - (poi_smo - 1)->y;
        //     // }
        // }

        // 将tmp更新为当前点
        // tmp_p = current_p;
    }

    /* 删除重叠点 */
    std::vector<attribute::Points>::iterator poi_del = prepoi.begin() + 1;
    while(poi_del < prepoi.end())
    {
        if(fabs(poi_del->x - (poi_del + 1)->x) < 10.0f && fabs(poi_del->y - (poi_del - 1)->y) < 10.0f)
        // if((poi_del->x == (poi_del - 1)->x) && (poi_del->y == (poi_del - 1)->y))
        {
            poi_del = prepoi.erase(poi_del);
        }
        else
        {
            ++poi_del;
        }
    }

    /* 平滑出斜边 
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
        std::cout << poi_sno1_judge << ", " << poi_sno2_judge << ", " << poi_sno3_judge << std::endl;


        if((poi_sno1_judge > 0.90f && poi_sno1_judge < 1.10f) && (poi_sno2_judge > 0.90f && poi_sno2_judge < 1.10f) && (poi_sno3_judge > 0.90f && poi_sno3_judge < 1.10f))
        // if( ((Points::cheby_Dist(*(poi_sno + 1), *(poi_sno - 1)) > 0.90f) && (Points::cheby_Dist(*(poi_sno + 1), *(poi_sno - 1)) < 1.10f)) && (((Points::cheby_Dist(*(poi_sno + 2), *poi_sno) > 0.90f) && (Points::cheby_Dist(*(poi_sno + 2), *poi_sno) < 1.10f)) && ((Points::cheby_Dist(*poi_sno, *(poi_sno - 2)) > 0.90f) && (Points::cheby_Dist(*poi_sno, *(poi_sno - 2)) < 1.10f))))
        {
            // std::cout << "cheby2" << std::endl;
            poi_sno->y = ((poi_sno - 1)->y + (poi_sno + 1)->y) / 2.0f;
            poi_sno->x = ((poi_sno - 1)->x + (poi_sno + 1)->x) / 2.0f;
            // tmp_p = *(poi_smo + 1);
        }
    }
    */

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