#ifndef _RELAESE_H_
#define _RELAESE_H_

#include "licifit.h"

std::vector<attribute::Line>* getTarget(std::vector<attribute::Points>& pointcloud_)
{
    constexpr float euci_thres = 200.0f;
    constexpr unsigned int min_sum = 30; // 簇的最少点数
    // constexpr float sig_max = 20.0f;

    attribute::Scancluster ori_poin(pointcloud_, euci_thres);
    std::cout << "origin cluster: " << ori_poin.cluster.size() << std::endl;
    
    ori_poin.delSmalldis(min_sum);
    std::cout << "delete less-point cluster: " << ori_poin.cluster.size() << std::endl;

    std::vector<attribute::Line>* line_seg = new std::vector<attribute::Line>;
    line_seg->reserve(pointcloud_.size() / min_sum);

    std::vector<attribute::Points> candi_point;
    candi_point.reserve(pointcloud_.size() / min_sum);

    // for(auto poi_iter = ori_poin.cluster.begin(); poi_iter != ori_poin.cluster.end(); poi_iter++)
    std::list<std::vector<attribute::Points>>::iterator poi_iter = ori_poin.cluster.begin();
    while(poi_iter != ori_poin.cluster.end())
    {
        // std::vector<attribute::Points> candi_point(attribute::preSmooth(*poi_iter)); // 拷贝构造
        candi_point.clear();
        candi_point = attribute::preSmooth(*(poi_iter));

        std::vector<attribute::Points> tmp_point;
        tmp_point.reserve(candi_point.size());
        poi_iter++;
        // std::vector<attribute::Points>::iterator begi = candi_point.begin();
        // while(begi < candi_point.end())
        for(std::vector<attribute::Points>::iterator begi = candi_point.begin(); begi != candi_point.end(); begi++)
        {
            tmp_point.clear();
            tmp_point.push_back(*begi);
            tmp_point.push_back(*(begi + 1));

            attribute::Line eline = lsfitting::leastSquare(tmp_point);
            float scope = eline.slope;
            for(auto ptr = begi + 2; ptr < candi_point.end(); ptr++)
            {
                tmp_point.push_back(*ptr);
                attribute::Line miline = lsfitting::leastSquare(tmp_point);
                // if((abs(miline.slope - scope) > 0.001f) || (miline.sigma > sig_max))
                if(!(fabs(miline.slope - scope) < 0.01f))
                {
                    tmp_point.pop_back();
                    if(tmp_point.size() < 3)
                    {
                        ++begi;
                        break;
                    }
                    else
                    {
                        eline.combine = tmp_point;
                        line_seg->push_back(eline);
                        begi = ptr;
                        break;
                    }
                }
                else if(ptr == candi_point.end() - 1)
                {
                    eline.combine = tmp_point;
                    line_seg->push_back(eline);
                    begi = ptr;
                    // begi = candi_point.end();
                    break;
                }
            }
        }
    }
    std::cout << "line_seg size: " << line_seg->size() << std::endl;
    return line_seg;
}

bool lineRelation(attribute::Line& base, attribute::Line& side)
{
    const float side_maxlen = 190.0f;
    const float side_minlen = 140.0f;
    const float max_angle = 1.658f;
    const float min_angle = 1.484f;
    float base_len = attribute::Line::length(base);
    float side_len = attribute::Line::length(side);

    float son = attribute::Line::vect(base)[0] * attribute::Line::vect(side)[0] + attribute::Line::vect(base)[1] * attribute::Line::vect(side)[1];
    float mom = base_len * side_len;
    float angle = acos(son / mom);

    if(angle > min_angle && angle < max_angle && side_len > side_minlen && side_len < side_maxlen)
    { return true; }
    else
    { return false; }
}

attribute::Points* goalPoint(std::vector<attribute::Line>* pre_line)
{
    const float base_maxlen = 350.0f;
    const float base_minlen = 280.0f;
    attribute::Points* goal = new attribute::Points;
    goal = nullptr;

    std::cout << "pre_line size: " << pre_line->size() << std::endl;
    
    for(auto i = pre_line->begin(); i != pre_line->end(); i++)
    {
        float preline_len = attribute::Line::length(*i);
        
        std::cout << "line length: " << preline_len << std::endl;
        
        if(preline_len > base_minlen && preline_len < base_maxlen)
        {
            if(i == pre_line->begin())
            {
                if(lineRelation(*i, *(i + 1)))
                {
                    *goal = attribute::Line::midpoi(*i);
                }
            }
            else if(i == (pre_line->end() - 1))
            {
                if(lineRelation(*i, *(i - 1)))
                {
                    *goal = attribute::Line::midpoi(*i);
                }
            }
            else
            {
                if(lineRelation(*i, *(i - 1)) || lineRelation(*i, *(i + 1)))
                {
                    *goal = attribute::Line::midpoi(*i);
                }
            }
        }
        else
        {
            continue;
        }
    }
    return goal;
}

#endif