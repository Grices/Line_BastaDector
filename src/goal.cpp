#include "include/resbyvec.h"


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
    // 若检测到两条以上的侧边
    if(side_posi.size() >= 2)
    {
        // 计算两侧边的距离是否为底边长度
        for(auto lcb = side_posi.begin(); lcb < side_posi.end(); lcb++)
        {
            float l_x = (vec_set.at(*lcb).com.front().x + vec_set.at(*lcb).com.back().x) / 2;
            float l_y = (vec_set.at(*lcb).com.front().y + vec_set.at(*lcb).com.back().y) / 2;
            attribute::Points lc_midpoi(l_x, l_y);

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
                    else
                    { continue; }
                }


            }
        }
    }
    // 若检测到一条侧边
    else if(side_posi.size() == 1)
    {
        // 若侧边为第一条向量
        if(*(side_posi.data()) == 0)
        {
            // 相邻向量为底边
            if(vec_set.at(1).length > 280.0f && vec_set.at(1).length < 350.0f)
            {
                float si_angle = vectAngle(vec_set.at(0), vec_set.at(1));
                // 向量夹角 70-110‘
                if(si_angle > 1.22f && si_angle < 1.91f)
                {
                    goal_point->x = (vec_set[1].com.front().x + vec_set[1].com.back().x) / 2;
                    goal_point->y = (vec_set[1].com.front().y + vec_set[1].com.back().y) / 2;
                    // 移出目标点

                    return goal_point;
                }
            }
        }
        // 若侧边为最后一条向量
        else if(*(side_posi.data()) == vec_set.size() - 1)
        {
            float blen = vec_set.at(*(side_posi.data()) - 1).length;
            if(blen > 280.0f && blen < 350.0f)
            {
                float bl_ang = vectAngle(vec_set.back(), *(vec_set.end()-2));
                if(bl_ang > 1.22f && bl_ang < 1.91f)
                {
                    goal_point->x = ((vec_set.end()-2)->com.front().x + (vec_set.end()-2)->com.back().x) / 2;
                    goal_point->y = ((vec_set.end()-2)->com.front().y + (vec_set.end()-2)->com.back().y) / 2;
                    // 移出目标点

                    return goal_point;
                }
            }
        }
        // 若侧边不为首尾向量
        else
        {
            float clen = vec_set.at(*(side_posi.data()) - 1).length;
            float dlen = vec_set.at(*(side_posi.data()) + 1).length;
            int choice_flag = -1;
            if(clen > 280.0f && clen < 350.0f)
            {
                choice_flag = *(side_posi.data()) - 1;
            }
            else if(dlen > 280.0f && dlen < 350.0f)
            {
                choice_flag = *(side_posi.data()) + 1;
            }

            if(choice_flag != -1)
            {
                float cd_ang = vectAngle(vec_set[*(side_posi.data())], vec_set[choice_flag]);
                if(cd_ang > 1.22f && cd_ang < 1.91f)
                {
                    goal_point->x = (vec_set[choice_flag].com.front().x + vec_set[choice_flag].com.back().x) / 2;
                    goal_point->y = (vec_set[choice_flag].com.front().y + vec_set[choice_flag].com.back().y) / 2;
                    // 移出目标点

                    return goal_point;
                }
            }

        }
        
    }
    // 若未检测到侧边
    else
    {
        // 若检测到一条底边
        if(base_posi.size() == 1)
        {
            bool detection_flag = false;
            if(*base_posi.data() == 0)
            {
                float klen = vec_set[1].length;
                float k_ang = vectAngle(vec_set[0], vec_set[1]);
                if(klen > 110.0f && klen < 150.0f && k_ang > 1.22f && k_ang < 1.91f)
                {
                    detection_flag = true;
                    // goal_point->x = (vec_set[0].com.front().x + vec_set[0].com.back().x) / 2;
                    // goal_point->y = (vec_set[0].com.front().y + vec_set[0].com.back().y) / 2;
                    // return goal_point;
                }
            }
            else if(*base_posi.data() == vec_set.size() - 1)
            {
                float llen = (vec_set.end() - 2)->length;
                float l_ang = vectAngle(vec_set.back(), *(vec_set.end() - 2));
                if(llen > 110.0f && llen < 150.0f && l_ang > 1.22f && l_ang < 1.91f)
                { detection_flag = true; }
            }
            else
            {
                float mlen = vec_set[*(base_posi.data()) - 1].length;
                float m_ang = vectAngle(vec_set[*(base_posi.data())], vec_set[*(base_posi.data()) - 1]);
                float nlen = vec_set[*(base_posi.data()) + 1].length;
                float n_ang = vectAngle(vec_set[*(base_posi.data())], vec_set[*(base_posi.data()) + 1]);
                // int side_flag = 0;
                // 相邻向量是否为基站侧边
                if (mlen > 110.0f && mlen < 150.0f && m_ang > 1.22f && m_ang < 1.91f)
                {
                    detection_flag = true;
                    // goal_point->x = (vec_set[*base_posi.data()].com.front().x + vec_set[*base_posi.data()].com.back().x) / 2;
                    // goal_point->y = (vec_set[*base_posi.data()].com.front().y + vec_set[*base_posi.data()].com.back().y) / 2;
                }
                else if(nlen > 110.0f && nlen < 150.0f && n_ang > 1.22f && n_ang < 1.91f)
                {
                    detection_flag = true;
                    // goal_point->x = (vec_set[*base_posi.data()].com.front().x + vec_set[*base_posi.data()].com.back().x) / 2;
                    // goal_point->y = (vec_set[*base_posi.data()].com.front().y + vec_set[*base_posi.data()].com.back().y) / 2;
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
                float slen = vec_set[*p].length;
                float xlen = vec_set[*p - 1].length;
                float ylen = vec_set[*p + 1].length;
                float sx_ang = vectAngle(vec_set[*p], vec_set[*p - 1]);
                float sy_ang = vectAngle(vec_set[*p], vec_set[*p + 1]);

                if(xlen > 110.0f && xlen < 150.0f && sx_ang > 1.22f && sx_ang < 1.91f)
                {
                    base_flag.push_back(*p);
                }
                else if(ylen > 110.0f && ylen < 150.0f && sy_ang > 1.22f && sy_ang < 1.91f)
                {
                    base_flag.push_back(*p);
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