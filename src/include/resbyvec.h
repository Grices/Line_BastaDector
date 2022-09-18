#ifndef _RESBYVEC_H_
#define _RESBYVEC_H_

#include "attribute.h"
// 计算向量夹角
float vectAngle(const attribute::Vect& vect_m, const attribute::Vect& vect_n)
{
    return acos((vect_m.vec[0] * vect_n.vec[0] + vect_m.vec[1] * vect_n.vec[1]) / (vect_m.length * vect_n.length)); 
}
// 计算两点距离
float poiDistan(const attribute::Points& p, const attribute::Points& q)
{
    return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2));
}


std::vector<attribute::Vect> getVect(const std::vector<attribute::Points>& pointcloud_)
{
    const float min_cludis = 250.0f;
    const unsigned int min_pois = 5;
    const float smo_dis = 10.0f;
    const float max_vec = 300.0f;
    // 聚类预处理 list<vector<points>>
    attribute::Scancluster ori_poin(pointcloud_, min_cludis);
    ori_poin.delSmalldis(min_pois);
    // std::vector<attribute::Line>* line_seg = new std::vector<attribute::Line>;
    std::vector<attribute::Vect> vect_seg;
    vect_seg.reserve(pointcloud_.size() / 2);

    // 遍历所有点集合
    auto poi_iter = ori_poin.cluster.begin();
    while(poi_iter != ori_poin.cluster.end())
    {
        // 平滑
        std::vector<attribute::Points> candi_point(attribute::preSmooth(*poi_iter, smo_dis));
        // 临时存储
        std::vector<attribute::Points> tmp_pois;
        tmp_pois.reserve(candi_point.size());
        
        poi_iter++;

        // 遍历集合中所有点
        for(auto begi = candi_point.begin(); begi < candi_point.end(); begi++)
        {
            tmp_pois.clear();
            tmp_pois.push_back(*begi);
            tmp_pois.push_back(*(begi + 1));
            
            attribute::Vect midvect(tmp_pois.front(), tmp_pois.back());
            // std::cout << midvect.length << ", " << tmp_pois.size() << std::endl;
            if(midvect.length > max_vec)
            { continue; }

            for(auto ptr = begi + 2; ptr < candi_point.end(); ptr++)
            {
                tmp_pois.push_back(*(ptr + 1));
                attribute::Vect midvect_m(tmp_pois.front(), *(tmp_pois.end() - 2));
                attribute::Vect midvect_n(*(tmp_pois.end()-2), tmp_pois.back());
                // 计算向量夹角
                // float between_ang = acos((midvect_m.vec[0] * midvect_n.vec[0] + midvect_m.vec[1] * midvect_n.vec[1]) / (midvect_m.length * midvect_n.length)); 
                float between_ang = vectAngle(midvect_m, midvect_n);
                // 向量夹角 > 60度
                if(between_ang > 0.001f)
                {
                    tmp_pois.pop_back();
                    // attribute::Line eline =lsfitting::leastSquare(tmp_pois);
                    // line_seg->push_back(eline);
                    attribute::Vect resu_vect(tmp_pois);
                    if(resu_vect.com.size() < 3 && resu_vect.length < 70.0)
                    {
                        // begi = ptr - 1;
                        break;
                    }
                    else
                    {
                        // std::cout << "q1: " << between_ang << std::endl;
                        vect_seg.push_back(resu_vect);
                        begi = ptr - 1;
                        break;
                    }
                }
                else if(ptr == candi_point.end() - 1)
                {
                    attribute::Vect resu_vect(tmp_pois);
                    vect_seg.push_back(resu_vect);
                    begi = ptr;
                    break;
                }
                else
                {
                    continue;
                }
            }            
        }
    }
    return vect_seg;
}

attribute::Points* getGoal(const std::vector<attribute::Vect>& vec_set)
{
    attribute::Points* goal_poi = new attribute::Points;
    std::vector<attribute::Vect> tmp_vec;
    // 筛选符合侧边宽度的向量
    for(std::vector<attribute::Vect>::const_iterator i = vec_set.begin(); i != vec_set.end() - 1; i++)
    {
        if(i->length > 170.0 && i->length < 190.0)
        {
            tmp_vec.push_back(*i);
        }
    }
    // 是否存在符合条件的向量
    if(tmp_vec.empty())
    {
        return NULL;
    }
    // 检测到单边的情况
    else if(tmp_vec.size() == 1)
    {
        (*goal_poi).x = (tmp_vec.begin().base()->com.front().x + tmp_vec.begin().base()->com.back().x) / 2;
        (*goal_poi).y = (tmp_vec.begin().base()->com.front().y + tmp_vec.begin().base()->com.back().y) / 2;
        return goal_poi;
    }
    // 检测到两条边
    else if(tmp_vec.size() == 2)
    {
        // 判断夹角
        if(vectAngle(tmp_vec.at(0), tmp_vec.at(1)) > 2.792f || vectAngle(tmp_vec.at(0), tmp_vec.at(1)) < 0.349f)
        {
            attribute::Points vec_left((tmp_vec.at(0).com.front().x + tmp_vec.at(0).com.back().x) / 2, (tmp_vec.at(0).com.front().y + tmp_vec.at(0).com.back().y) / 2);
            attribute::Points vec_righ((tmp_vec.at(1).com.front().x + tmp_vec.at(1).com.back().x) / 2, (tmp_vec.at(1).com.front().y + tmp_vec.at(1).com.back().y) / 2);            
            // 判断间距
            float lf_poi_dis = poiDistan(vec_left, vec_righ);
            if(lf_poi_dis > 300 && lf_poi_dis < 350)
            {
                goal_poi->x = (vec_left.x + vec_righ.x) / 2;
                goal_poi->y = (vec_left.y + vec_righ.y) / 2; 
                // 目标点移至回充检测范围
                if(tmp_vec.front().vec[0] < 0.001f)
                {
                    if(goal_poi->x < 0) {goal_poi->x += 950.0f;}
                    else {goal_poi->x -= 950.f;}
                }
                else if(tmp_vec.front().vec[1] < 0.001f)
                {
                    if(goal_poi->y < 0) {goal_poi->y += 950.0f;}
                    else {goal_poi->y -= 950.0f;}
                }
                return goal_poi;
            }
            else
            { return NULL; }
        }
        else
        { return NULL; }
    }
    else
    {
        for(std::vector<attribute::Vect>::iterator j = tmp_vec.begin(); j != tmp_vec.end(); j++)
        {
            attribute::Points jtmp_poi((j->com.front().x + j->com.back().x) / 2, (j->com.front().y + j->com.back().y) / 2);
            for(std::vector<attribute::Vect>::iterator k = j + 1; k < tmp_vec.end(); k++)
            {
                if(vectAngle(*j, *k) > 2.792f || vectAngle(*j, *k) < 0.349f)
                {
                    attribute::Points ktmp_poi((k->com.front().x + k->com.back().x) / 2, (k->com.front().y + k->com.back().y) / 2);
                    float jk_poi_dis = poiDistan(jtmp_poi, ktmp_poi);
                    if(jk_poi_dis > 300 && jk_poi_dis < 350)
                    {
                        goal_poi->x = (jtmp_poi.x + ktmp_poi.x) / 2;
                        goal_poi->y = (jtmp_poi.y + ktmp_poi.y) / 2;
                        if(tmp_vec.front().vec[0] < 0.001f)
                        {
                            if(goal_poi->x < 0) {goal_poi->x += 950.0f;}
                            else {goal_poi->x -= 950.f;}
                        }
                        else if(tmp_vec.front().vec[1] < 0.001f)
                        {
                            if(goal_poi->y < 0) {goal_poi->y += 950.0f;}
                            else {goal_poi->y -= 950.0f;}
                        }
                        return goal_poi;
                    }                   
                }               
            }
        }
        return NULL;
    }
}

#endif