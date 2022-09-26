#ifndef _RESBYVEC_H_
#define _RESBYVEC_H_

#include "attribute.h"
/* 向量夹角 */
float vectAngle(const attribute::Vect& vect_m, const attribute::Vect& vect_n)
{
    return acos((vect_m.vec[0] * vect_n.vec[0] + vect_m.vec[1] * vect_n.vec[1]) / (vect_m.length * vect_n.length)); 
}
/* 两点距离 */
float poiDistan(const attribute::Points& p, const attribute::Points& q)
{
    return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2));
}

/* 向量提取 */
std::vector<attribute::Vect> getVect(std::vector<attribute::Points>& pointcloud_)
{
    const float min_cludis = 130.0f;
    const unsigned int min_pois = 10;
    const int fli_width = 3;
    const float vec_judge_angle = 0.523f;
    const float origin_veclen = 200.0f; // 初始点构成的向量长度
    // 聚类-筛选
    attribute::Sacnpcluter ori_poin(pointcloud_, min_cludis);
    ori_poin.deleteSmall(min_pois);
    ori_poin.mergeCluster(100.0f);
    // 定义存储向量的容器
    std::vector<attribute::Vect> vect_seg;
    vect_seg.reserve(pointcloud_.size() / 2);
    // 定义存放簇数据的中间容器
    // 遍历所有点集合
    auto poi_iter = ori_poin.cluster.begin();
    while(poi_iter != ori_poin.cluster.end())
    {
        // std::vector<attribute::Points> candi_point(attribute::preSmooth(attribute::downSampling(attribute::meanFilter(*poi_iter, fli_width))));
        std::vector<attribute::Points> candi_point;
        // candi_point.clear();
        candi_point = attribute::meanFilter(*poi_iter, fli_width); // 高斯/均值 平滑
        candi_point = attribute::downSampling(candi_point); // 平均/最大 池化
        candi_point = attribute::preSmooth(candi_point);    // chebyshev比例 锐化
        poi_iter++;

        // 临时存放向量所包含的点
        std::vector<attribute::Points> tmp_pois;
        tmp_pois.reserve(candi_point.size());
        // 遍历聚类中的点
        for(auto begi = candi_point.begin(); begi < candi_point.end(); begi++)
        {
            tmp_pois.clear();
            tmp_pois.push_back(*begi);
            tmp_pois.push_back(*(begi + 1));
            
            attribute::Vect midvect(tmp_pois.front(), tmp_pois.back());
            // std::cout << "origin_veclen: " << midvect.length << std::endl;
            // 若初始两点的向量过长 continue
            if(midvect.length > origin_veclen)
            { continue; }

            for(auto ptr = begi + 2; ptr < candi_point.end(); ptr++)
            {
                tmp_pois.push_back(*ptr);
                attribute::Vect midvect_m(tmp_pois.front(), *(tmp_pois.end() - 2));
                attribute::Vect midvect_n(*(tmp_pois.end()-2), tmp_pois.back());
                // 计算向量夹角
                // float between_ang = acos((midvect_m.vec[0] * midvect_n.vec[0] + midvect_m.vec[1] * midvect_n.vec[1]) / (midvect_m.length * midvect_n.length)); 
                float between_ang = vectAngle(midvect_m, midvect_n);
                // std::cout << "between_ang: " << between_ang << std::endl;
                // 向量夹角 > 40度(弧度 0.689) 30度(0.523)
                if(between_ang > vec_judge_angle)
                {
                    tmp_pois.pop_back();
                    // attribute::Line eline =lsfitting::leastSquare(tmp_pois);
                    // line_seg->push_back(eline);
                    attribute::Vect resu_vect(tmp_pois);
                    // if(resu_vect.com.size() < 5 && (resu_vect.length < 100.0 || resu_vect.length > 200.0))
                    if(resu_vect.com.size() < 4 && (resu_vect.length < 150.0f || resu_vect.length > 250.0f))
                    {
                        // begi = ptr - 1;
                        break;
                    }
                    else
                    {
                        // std::cout << "q1: " << between_ang << std::endl;
                        vect_seg.push_back(resu_vect);
                        begi = ptr - 2;
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

/* 向量合并 */
std::vector<attribute::Vect> vectMerge(std::vector<attribute::Vect>& vec)
{
    for(auto i = vec.begin() + 1; i < vec.end();)
    {
        // 向量长度<250mm && 两向量距离<25mm && 两向量趋于平行
        if((i->length < 250.0f) && (poiDistan((i-1)->com.back(), *(i->com.begin()+1))) < 25.0f && (vectAngle(*i, *(i - 1)) > 2.62f || vectAngle(*i, *(i - 1)) < 0.52f))
        {
            for(auto j = i->com.begin(); j != i->com.end(); j++)
            {
                (i-1)->com.push_back(*j);
            }
            *(i-1) = attribute::Vect((i-1)->com);
            i = vec.erase(i);
        }
        else
        {
            i++;
        }
    }
    return vec;
}

/* 获取目标点 */
attribute::Points* getFinalgoal(const std::vector<attribute::Vect>& vec_set)
{
    attribute::Points* goal_point = new attribute::Points;
    // std::vector<attribute::Vect> tmp_vec;
    
    std::vector<int> side_posi; // 存放侧边索引
    std::vector<int> base_posi; // 存放底边索引
    side_posi.reserve(vec_set.size() / 2);
    base_posi.reserve(vec_set.size() / 2);
    int tmp_num = 0;
    // 遍历向量 获取边的位置索引
    for(std::vector<attribute::Vect>::const_iterator i = vec_set.begin(); i != vec_set.end(); i++, tmp_num++)
    {
        if(i->length > 150.0f && i->length < 200.0f)
        {
            // tmp_vec.emplace_back(*i);
            side_posi.push_back(tmp_num);
        }
        if(i->length > 280.0f && i->length < 350.0f)
        {
            base_posi.push_back(tmp_num);
        }   
    }
    // 若底边或侧边均未检测到
    if(side_posi.empty() && base_posi.empty())
    { return NULL;}
    // 若检测到两条以上的侧边
    else if(side_posi.size() >= 2)
    {
        int last_vec_flag = vec_set.size() - 1;
        for(auto lcb = side_posi.begin(); lcb < side_posi.end(); lcb++)
        {
            float l_x = (vec_set.at(*lcb).com.front().x + vec_set.at(*lcb).com.back().x) / 2;
            float l_y = (vec_set.at(*lcb).com.front().y + vec_set.at(*lcb).com.back().y) / 2;
            attribute::Points lc_midpoi(l_x, l_y);
            // 计算两侧边的距离是否为底边长度
            for(auto rcb = lcb + 1; rcb < side_posi.end(); rcb++)
            {
                // 判断向量是否平行 >160' 或 <20'
                float vec_ang = vectAngle(vec_set.at(*lcb), vec_set.at(*rcb));
                if(vec_ang > 2.792f || vec_ang < 0.349f)
                {
                    float r_x = (vec_set.at(*rcb).com.front().x + vec_set.at(*rcb).com.back().x) / 2;
                    float r_y = (vec_set.at(*rcb).com.front().y + vec_set.at(*rcb).com.back().y) / 2;
                    attribute::Points rc_midpoi(r_x, r_y);
                    // 判断两向量中心点的距离
                    float poi_dist = poiDistan(lc_midpoi,rc_midpoi);
                    if(poi_dist > 280.0f && poi_dist < 350.0f)
                    {
                        goal_point->x = (lc_midpoi.x + rc_midpoi.x) / 2;
                        goal_point->y = (lc_midpoi.y + rc_midpoi.y) / 2;
                        // 将目标点移至检测区
                        if(fabs(vec_set.at(*lcb).vec[0]) < fabs(vec_set.at(*lcb).vec[1]))
                        {
                            if(goal_point->y <= 0)
                            { goal_point->y += 950.0f;}
                            else
                            { goal_point->y -= 950.0f;}
                        }
                        else
                        {
                            if(goal_point->x <= 0)
                            { goal_point->x += 950.0f;}
                            else
                            { goal_point->x -= 950.0f;}
                        }
                        return goal_point;
                    }
                    else
                    { continue; }
                }
            }
            // 判断是否为单一侧边
            bool si_detect_flag = false;
            int si_detect_posi = -1;
            if(*lcb == 0)
            {
                if(vec_set[1].length > 280.0f && vec_set[1].length < 350.0f)
                {
                    float sia_ang = vectAngle(vec_set[0], vec_set[1]);
                    float sia_dis = std::min(poiDistan(vec_set[0].com.back(), vec_set[1].com.front()), poiDistan(vec_set[0].com.back(), vec_set[1].com.back()));
                    if(sia_ang > 1.22 && sia_ang < 1.91 && sia_dis < 50.0f)
                    {
                        // goal_point->x = (vec_set[1].com.front().x + vec_set[1].com.back().x) / 2;
                        // goal_point->y = (vec_set[1].com.front().y + vec_set[1].com.back().y) / 2;
                        si_detect_posi = 1;
                        si_detect_flag = true;
                    }
                }
            }
            else if(*lcb == last_vec_flag)
            {
                if(vec_set[1].length > 280.0f && vec_set[1].length < 350.0f)
                {
                    float sib_ang = vectAngle(vec_set[*lcb], vec_set[*lcb - 1]);
                    float sib_dis = std::min(poiDistan(vec_set[*lcb - 1].com.back(), vec_set.back().com.front()), poiDistan(vec_set[*lcb - 1].com.back(), vec_set.back().com.back()));
                    if(sib_ang > 1.22 && sib_ang < 1.91 && sib_dis < 50.0f)
                    {
                        // goal_point->x = (vec_set[*lcb - 1].com.front().x + vec_set[*lcb - 1].com.back().x) / 2;
                        // goal_point->y = (vec_set[*lcb - 1].com.front().y + vec_set[*lcb - 1].com.back().y) / 2;
                        si_detect_posi = *lcb - 1;
                        si_detect_flag = true;
                    }
                }
            }
            else
            {
                if(vec_set[*lcb - 1].length > 280.0f && vec_set[*lcb - 1].length < 350.0f)
                {
                    float sic1_ang = vectAngle(vec_set[*lcb], vec_set[*lcb - 1]);
                    float sic1_dis = std::min(poiDistan(vec_set[*lcb - 1].com.back(), vec_set[*lcb].com.front()), poiDistan(vec_set[*lcb - 1].com.back(), vec_set[*lcb].com.back()));
                    if(sic1_ang > 1.22f && sic1_ang < 1.91f && sic1_dis < 50.0f)
                    {
                        si_detect_posi = *lcb -1;
                        si_detect_flag = true;
                    }
                }
                else if(vec_set[*lcb + 1].length > 280.0f && vec_set[*lcb + 1].length < 350.0f)
                {
                    float sic2_ang = vectAngle(vec_set[*lcb], vec_set[*lcb + 1]);
                    float sic2_dis = std::min(poiDistan(vec_set[*lcb].com.back(), vec_set[*lcb + 1].com.front()), poiDistan(vec_set[*lcb].com.back(), vec_set[*lcb + 1].com.back()));
                    if(sic2_ang > 1.22f && sic2_ang < 1.91f && sic2_dis < 50.0f)
                    {
                        si_detect_posi = *lcb + 1;
                        si_detect_flag = true;
                    }
                }    
            }
            if(si_detect_flag)
            {
                goal_point->x = (vec_set[si_detect_posi].com.front().x + vec_set[si_detect_posi].com.back().x) / 2;
                goal_point->y = (vec_set[si_detect_posi].com.front().y + vec_set[si_detect_posi].com.back().y) / 2;
                if(fabs(vec_set[si_detect_posi].vec[0]) < vec_set[si_detect_posi].vec[1])
                {
                    if(goal_point->x <= 0)
                    { goal_point->x += 950.0f;}
                    else
                    { goal_point->x -= 950.0f;}
                }
                else
                {
                    if(goal_point->y <= 0)
                    { goal_point->y += 950.0f;}
                    else
                    { goal_point->y -= 950.0f;}
                }
                return goal_point;
            }
        }
    }
    // 若检测到一条侧边
    else if(side_posi.size() == 1)
    {
        int prr = vec_set.size() - 1; // vec_set的最后元素索引
        int tmp_posi = -1;
        bool detect_flag = false;
        // 若侧边为第一条向量
        if(*(side_posi.data()) == 0)
        {
            // 相邻向量为底边
            if(vec_set.at(1).length > 280.0f && vec_set.at(1).length < 350.0f)
            {
                float si_angle = vectAngle(vec_set.at(0), vec_set.at(1));
                float si_dista = std::min(poiDistan(vec_set[0].com.back(), vec_set[1].com.front()), poiDistan(vec_set[0].com.back(), vec_set[1].com.back()));
                // 向量夹角 70-110‘
                if(si_angle > 1.22f && si_angle < 1.91f && si_dista < 50.0f)
                {
                    // goal_point->y = (vec_set[1].com.front().y + vec_set[1].com.back().y) / 2;
                    // goal_point->x = (vec_set[1].com.front().x + vec_set[1].com.back().x) / 2;
                    // return goal_point;
                    tmp_posi = 1;
                    detect_flag = true;
                }
            }
        }
        // 若侧边为最后一条向量
        else if(*side_posi.data() == prr)
        {
            float blen = vec_set.at(*(side_posi.data()) - 1).length;
            if(blen > 280.0f && blen < 350.0f)
            {
                float bl_ang = vectAngle(vec_set.back(), *(vec_set.end()-2));
                float bl_dis = std::min(poiDistan(vec_set[prr - 1].com.back(), vec_set[prr].com.front()), poiDistan(vec_set[prr - 1].com.back(), vec_set[prr].com.back()));
                if(bl_ang > 1.22f && bl_ang < 1.91f && bl_dis < 50.0f)
                {
                    // goal_point->x = ((vec_set.end()-2)->com.front().x + (vec_set.end()-2)->com.back().x) / 2;
                    // goal_point->y = ((vec_set.end()-2)->com.front().y + (vec_set.end()-2)->com.back().y) / 2;
                    // return goal_point;
                    tmp_posi = prr;
                    detect_flag = true;
                }
            }
        }
        // 若侧边不为首尾向量
        else
        {
            int pcd = *side_posi.data();
            float clen = vec_set.at(pcd - 1).length;
            float dlen = vec_set.at(pcd + 1).length;
            
            if(clen > 280.0f && clen < 350.0f)
            {
                float cc_dis = std::min(poiDistan(vec_set[pcd - 1].com.back(), vec_set[pcd].com.front()), poiDistan(vec_set[pcd - 1].com.back(), vec_set[pcd].com.back()));
                float cc_ang = vectAngle(vec_set[pcd - 1], vec_set[pcd]);
                if(cc_ang > 1.22f && cc_ang < 1.91f && cc_dis < 50.0f)
                {
                    tmp_posi = pcd - 1;
                    detect_flag = true;
                }
                // choice_flag = *(side_posi.data()) - 1;
            }
            else if(dlen > 280.0f && dlen< 350.0f)
            {
                float dd_dis = std::min(poiDistan(vec_set[pcd].com.back(), vec_set[pcd + 1].com.front()), poiDistan(vec_set[pcd].com.back(), vec_set[pcd + 1].com.back()));
                float dd_ang = vectAngle(vec_set[pcd], vec_set[pcd + 1]);
                if(dd_ang > 1.22f && dd_ang < 1.91f && dd_dis < 50.0f)
                {
                    tmp_posi = pcd + 1;
                    detect_flag = true;
                }
                // choice_flag = *(side_posi.data()) + 1;
            }

            if(detect_flag)
            {
                goal_point->x = (vec_set[tmp_posi].com.front().x + vec_set[tmp_posi].com.back().x) / 2;
                goal_point->y = (vec_set[tmp_posi].com.front().y + vec_set[tmp_posi].com.back().y) / 2;
                if(fabs(vec_set[tmp_posi].vec[0]) < vec_set[tmp_posi].vec[1])
                {
                    if(goal_point->x <= 0)
                    { goal_point->x += 950.0f;}
                    else
                    { goal_point->x -= 950.0f;}
                }
                else
                {
                    if(goal_point->y <= 0)
                    { goal_point->y += 950.0f;}
                    else
                    { goal_point->y -= 950.0f;}
                }
                return goal_point;
            }
            // if(choice_flag != -1)
            // {
            //     float cd_ang = vectAngle(vec_set[*(side_posi.data())], vec_set[choice_flag]);
            //     if(cd_ang > 1.22f && cd_ang < 1.91f)
            //     {
            //         goal_point->x = (vec_set[choice_flag].com.front().x + vec_set[choice_flag].com.back().x) / 2;
            //         goal_point->y = (vec_set[choice_flag].com.front().y + vec_set[choice_flag].com.back().y) / 2;
            //         // 移出目标点
            //         return goal_point;
            //     }
            // }
        }
        
    }
    // 若未检测到侧边
    else
    {
        // 若检测到一条底边
        if(base_posi.size() == 1)
        {
            bool detection_flag = false;
            int bbs = vec_set.size() - 1;
            if(*base_posi.data() == 0)
            {
                float klen = vec_set[1].length;
                if(klen > 110.0f && klen < 150.0f)
                {
                    float k_ang = vectAngle(vec_set[0], vec_set[1]);
                    float k_dis = std::min(poiDistan(vec_set[0].com.back(), vec_set[1].com.front()), poiDistan(vec_set[0].com.back(), vec_set[1].com.back()));
                    if(k_ang > 1.22f && k_ang < 1.91f && k_dis < 50.0f)
                    {
                        detection_flag = true;
                        // goal_point->x = (vec_set[0].com.front().x + vec_set[0].com.back().x) / 2;
                        // goal_point->y = (vec_set[0].com.front().y + vec_set[0].com.back().y) / 2;
                        // return goal_point;
                    }

                }
            }
            else if(*base_posi.data() == bbs)
            {
                float llen = (vec_set.end() - 2)->length;
                if(llen > 110.0f && llen < 150.0f)
                {
                    float l_ang = vectAngle(vec_set.back(), *(vec_set.end() - 2));
                    float l_dis = std::min(poiDistan(vec_set[bbs].com.back(), vec_set.back().com.front()), poiDistan(vec_set[bbs].com.back(), vec_set.back().com.back()));
                    if(l_ang > 1.22f && l_ang < 1.91f && l_dis < 50.0f)
                    { detection_flag = true; }
                }
            }
            else
            {
                int side_po = *(base_posi.data());
                float mlen = vec_set[side_po - 1].length;
                float nlen = vec_set[side_po + 1].length;
                // 相邻向量是否为基站侧边
                if (mlen > 110.0f && mlen < 150.0f)
                {
                    float m_ang = vectAngle(vec_set[side_po], vec_set[side_po - 1]);
                    float m_dis = std::min(poiDistan(vec_set[side_po - 1].com.back(), vec_set[side_po].com.front()), poiDistan(vec_set[side_po - 1].com.back(), vec_set[side_po].com.back()));
                    if(m_ang > 1.22f && m_ang < 1.91f && m_dis < 50.0f)
                    {
                        detection_flag = true;
                        // goal_point->x = (vec_set[*base_posi.data()].com.front().x + vec_set[*base_posi.data()].com.back().x) / 2;
                        // goal_point->y = (vec_set[*base_posi.data()].com.front().y + vec_set[*base_posi.data()].com.back().y) / 2;
                    }
                }
                else if(nlen > 110.0f && nlen < 150.0f )
                {
                    float n_ang = vectAngle(vec_set[side_po], vec_set[side_po + 1]);
                    float n_dis = std::min(poiDistan(vec_set[side_po].com.back(), vec_set[side_po + 1].com.front()), poiDistan(vec_set[side_po].com.back(), vec_set[side_po + 1].com.back()));
                    if(n_ang > 1.22f && n_ang < 1.91f && n_dis < 50.0f)
                    {
                        detection_flag = true;
                        // goal_point->x = (vec_set[*base_posi.data()].com.front().x + vec_set[*base_posi.data()].com.back().x) / 2;
                        // goal_point->y = (vec_set[*base_posi.data()].com.front().y + vec_set[*base_posi.data()].com.back().y) / 2;
                    }
                }
                // 移出目标点
                // return goal_point;
            }
            // 若该向量符合底边特征 获取目标点
            if(detection_flag)
            {
                goal_point->x = (vec_set[*base_posi.data()].com.front().x + vec_set[*base_posi.data()].com.back().x) / 2;
                goal_point->y = (vec_set[*base_posi.data()].com.front().y + vec_set[*base_posi.data()].com.back().y) / 2;
                // 移动目标点至检测区
                if(fabs(vec_set[*base_posi.data()].vec[0]) < fabs(vec_set[*base_posi.data()].vec[1]))
                {
                    if(goal_point->x <= 0)
                    { goal_point->x += 950.0f;}
                    else
                    { goal_point->x -= 950.0f;}
                }
                else
                {
                    if(goal_point->y <= 0)
                    { goal_point->y += 950.0f;}
                    else
                    { goal_point->y -= 950.0f;}
                }
                return goal_point;
            }

        }
        // 若检测到多条底边
        else if(base_posi.size() >= 2)
        {
            std::vector<int> base_flag;
            base_flag.reserve(base_posi.size());

            for(auto p = base_posi.begin(); p != base_posi.end(); p++)
            {
                // float slen = vec_set[*p].length;
                float xlen = vec_set[*p - 1].length;
                float ylen = vec_set[*p + 1].length;
                
                if(xlen > 120.0f && xlen < 190.0f)
                {
                    float sx_ang = vectAngle(vec_set[*p], vec_set[*p - 1]);
                    float sx_dis = std::min(poiDistan(vec_set[*p - 1].com.back(), vec_set[*p].com.front()), poiDistan(vec_set[*p - 1].com.back(), vec_set[*p].com.back()));
                    if(sx_ang > 1.22f && sx_ang < 1.91f && sx_dis < 50.0f)
                    {
                        base_flag.push_back(*p);
                    }
                }
                else if(ylen > 120.0f && ylen < 190.0f)
                {
                    float sy_ang = vectAngle(vec_set[*p], vec_set[*p + 1]);
                    float sy_dis = std::min(poiDistan(vec_set[*p].com.back(), vec_set[*p + 1].com.front()), poiDistan(vec_set[*p].com.back(), vec_set[*p + 1].com.back()));
                    if(sy_ang > 1.22f && sy_ang < 1.91f && sy_dis < 50.0f)
                    {
                        base_flag.push_back(*p);

                    }
                }
            }
            if(!base_flag.empty())
            {
                float mindis = 1000.0f;
                int b_flag = -1;
                for(auto b = base_flag.begin(); b != base_flag.end(); b++)
                {
                    float minb = fabs(*b - 180.0f);
                    if(minb < mindis)
                    { 
                        mindis = minb;
                        b_flag = *b;
                    }
                }
                goal_point->x = (vec_set[b_flag].com.front().x + vec_set[b_flag].com.back().x) / 2;
                goal_point->y = (vec_set[b_flag].com.front().y + vec_set[b_flag].com.back().y) / 2;
                if(fabs(vec_set[b_flag].vec[0]) < fabs(vec_set[b_flag].vec[1]))
                {
                    if(goal_point->x <= 0)
                    { goal_point->x += 950.0f;}
                    else
                    { goal_point->x -= 950.0f;}
                }
                else
                {
                    if(goal_point->y <= 0)
                    { goal_point->y += 950.0f;}
                    else
                    { goal_point->y -= 950.0f;}
                }
                return goal_point;
            }
        }
    }
    return NULL;
}


/* 获取目标点-旧版 */
// attribute::Points* getGoal(const std::vector<attribute::Vect>& vec_set)
// {
//     attribute::Points* goal_poi = new attribute::Points;
//     std::vector<attribute::Vect> tmp_vec;
//     // 筛选符合侧边宽度的向量
//     for(std::vector<attribute::Vect>::const_iterator i = vec_set.begin(); i != vec_set.end() - 1; i++)
//     {
//         if(i->length > 160.0 && i->length < 200.0)
//         {
//             tmp_vec.push_back(*i);
//         }
//     }
//     // 是否存在符合条件的向量
//     if(tmp_vec.empty())
//     {
//         return NULL;
//     }
//     // 检测到单边的情况
//     else if(tmp_vec.size() == 1)
//     {
//         (*goal_poi).x = (tmp_vec.begin().base()->com.front().x + tmp_vec.begin().base()->com.back().x) / 2;
//         (*goal_poi).y = (tmp_vec.begin().base()->com.front().y + tmp_vec.begin().base()->com.back().y) / 2;
//         return goal_poi;
//     }
//     // 检测到两条边
//     else if(tmp_vec.size() == 2)
//     {
//         // 判断夹角
//         if(vectAngle(tmp_vec.at(0), tmp_vec.at(1)) > 2.792f || vectAngle(tmp_vec.at(0), tmp_vec.at(1)) < 0.349f)
//         {
//             attribute::Points vec_left((tmp_vec.at(0).com.front().x + tmp_vec.at(0).com.back().x) / 2, (tmp_vec.at(0).com.front().y + tmp_vec.at(0).com.back().y) / 2);
//             attribute::Points vec_righ((tmp_vec.at(1).com.front().x + tmp_vec.at(1).com.back().x) / 2, (tmp_vec.at(1).com.front().y + tmp_vec.at(1).com.back().y) / 2);            
//             // 判断间距
//             float lf_poi_dis = poiDistan(vec_left, vec_righ);
//             if(lf_poi_dis > 280 && lf_poi_dis < 330)
//             {
//                 goal_poi->x = (vec_left.x + vec_righ.x) / 2;
//                 goal_poi->y = (vec_left.y + vec_righ.y) / 2; 
//                 // 目标点移至回充检测范围
//                 if(tmp_vec.front().vec[0] < 0.001f)
//                 {
//                     if(goal_poi->x < 0) {goal_poi->x += 950.0f;}
//                     else {goal_poi->x -= 950.0f;}
//                 }
//                 else if(tmp_vec.front().vec[1] < 0.001f)
//                 {
//                     if(goal_poi->y < 0) {goal_poi->y += 950.0f;}
//                     else {goal_poi->y -= 950.0f;}
//                 }
//                 return goal_poi;
//             }
//             else
//             { return NULL; }
//         }
//         else
//         { return NULL; }
//     }
//     else
//     {
//         for(std::vector<attribute::Vect>::iterator j = tmp_vec.begin(); j != tmp_vec.end(); j++)
//         {
//             attribute::Points jtmp_poi((j->com.front().x + j->com.back().x) / 2, (j->com.front().y + j->com.back().y) / 2);
//             for(std::vector<attribute::Vect>::iterator k = j + 1; k < tmp_vec.end(); k++)
//             {
//                 if(vectAngle(*j, *k) > 2.792f || vectAngle(*j, *k) < 0.349f)
//                 {
//                     attribute::Points ktmp_poi((k->com.front().x + k->com.back().x) / 2, (k->com.front().y + k->com.back().y) / 2);
//                     float jk_poi_dis = poiDistan(jtmp_poi, ktmp_poi);
//                     if(jk_poi_dis > 300 && jk_poi_dis < 350)
//                     {
//                         goal_poi->x = (jtmp_poi.x + ktmp_poi.x) / 2;
//                         goal_poi->y = (jtmp_poi.y + ktmp_poi.y) / 2;
//                         if(tmp_vec.front().vec[0] < 0.001f)
//                         {
//                             if(goal_poi->x < 0) {goal_poi->x += 950.0f;}
//                             else {goal_poi->x -= 950.f;}
//                         }
//                         else if(tmp_vec.front().vec[1] < 0.001f)
//                         {
//                             if(goal_poi->y < 0) {goal_poi->y += 950.0f;}
//                             else {goal_poi->y -= 950.0f;}
//                         }
//                         return goal_poi;
//                     }                   
//                 }               
//             }
//         }
//         return NULL;
//     }
// }

#endif