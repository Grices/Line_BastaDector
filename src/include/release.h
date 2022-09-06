#ifndef _RELAESE_H_
#define _RELAESE_H_

#include "licifit.h"

std::vector<attribute::Line>* getTarget(std::vector<attribute::Points>& pointcloud_)
{
    constexpr float euci_thres = 150.0f;
    constexpr unsigned int min_sum = 30;
    constexpr float sig_max = 20.0f;

    attribute::Scancluster ori_poin(pointcloud_, euci_thres);
    ori_poin.delSmalldis(min_sum);
    std::vector<attribute::Line>* line_seg = new std::vector<attribute::Line>;
    line_seg->reserve(pointcloud_.size() / min_sum);

    std::list<std::vector<attribute::Points>>::iterator poi_iter = ori_poin.cluster.begin();
    while(poi_iter != ori_poin.cluster.end())
    {
        // ����ƽ�� ��������
        std::vector<attribute::Points> candi_point(attribute::preSmooth(*poi_iter));
        // �洢�м��
        std::vector<attribute::Points> tmp_point;
        tmp_point.reserve(candi_point.size());
        std::vector<attribute::Points>::iterator begi = candi_point.begin();
        while(begi < candi_point.end())
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
                if((miline.slope != scope) || (miline.sigma > sig_max))
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
                    begi = candi_point.end();
                    break;
                }
            }
        }
    }
    return line_seg;
}

bool lineRelation(attribute::Line& base, attribute::Line& side)
{
    const float side_maxlen = 190.0f;
    const float side_minlen = 140.0f;
    const float max_angle = 1.658f;
    const float min_angle = 1.484f;
    float son = base.vect()[0] * side.vect()[0] + base.vect()[1] * side.vect()[1];
    float mom = base.length() * side.length();
    float angle = acos(son / mom);

    if(angle > min_angle && angle < max_angle && side.length() > side_minlen && side.length() < side_maxlen)
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

    for(auto i = pre_line->begin(); i != pre_line->end(); i++)
    {
        if(i->length() > base_minlen && i->length() < base_maxlen)
        {
            if(i == pre_line->begin())
            {
                if(lineRelation(*i, *(i + 1)))
                {
                    *goal = (*i).midpoi();
                }
            }
            else if(i == (pre_line->end() - 1))
            {
                if(lineRelation(*i, *(i - 1)))
                {
                    *goal = (*i).midpoi();
                }
            }
            else
            {
                if(lineRelation(*i, *(i - 1)) || lineRelation(*i, *(i + 1)))
                {
                    *goal = (*i).midpoi();
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