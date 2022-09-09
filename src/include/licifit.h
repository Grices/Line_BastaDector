#ifndef _LICIFIT_H_
#define _LICIFIT_H_

#include "attribute.h"

namespace lsfitting
{
    float fitSigma(const std::vector<attribute::Points>& point, const attribute::Line& line)
    {
        float sigma = 0;
        for(auto i = point.begin(); i != point.end(); i++)
        {
            sigma += fabs(((i->x) * line.A + (i->y) * line.B + line.C) / sqrt(pow(line.A, 2) + pow(line.B, 2)));
        }
        return sqrt(sigma / point.size());
    }

    inline attribute::Line leastSquare(const std::vector<attribute::Points>& vec_point)
    {
        int size = vec_point.size();

        float x_mean = 0.0f;
        float y_mean = 0.0f;

        for(int i = 0; i < size; i++)
        {
            x_mean += vec_point[i].x;
            y_mean += vec_point[i].y;
        }
        x_mean /= size;
        y_mean /= size;

        float Dxx = 0.0f, Dxy = 0.0f, Dyy = 0.0f;
        for(int j = 0; j < size; j++)
        {
            Dxx += (vec_point[j].x - x_mean) * (vec_point[j].x - x_mean);
            Dxy += (vec_point[j].x - x_mean) * (vec_point[j].y - y_mean);
            Dyy += (vec_point[j].y - y_mean) * (vec_point[j].y - y_mean);
        }
        float lambda = ((Dxx + Dyy) - sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0;
        // if(lambda < 0.00001f) {lambda = 0.0f;}
        float den = sqrt(Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));
        float fa = Dxy / den;
        float fb = (lambda - Dxx) / den;
        float fc = -fa * x_mean - fb * y_mean;
        float lk = -fa / fb;
        attribute::Line fit_line(fa, fb, fc);
        // fit_line.A = fa;
        // fit_line.B = fb;
        // fit_line.C = fc;
        float sig = fitSigma(vec_point, fit_line);
        fit_line.slope = lk;
        fit_line.sigma = sig;
        
        return fit_line;
    }

    float vectJudge(const std::vector<attribute::Points>& vec_poin, const attribute::Points& pan_poin)
    {
        // 容器向量 [x1, y1]
        std::array<float, 2> vec = {vec_poin.back().x - vec_poin.front().x, vec_poin.back().y - vec_poin.front().y};
        float vec_len = attribute::Points::euclid_Dis(vec_poin.back(), vec_poin.front());   
        // 新向量 [x2, y2]
        std::array<float, 2> pan = {pan_poin.x - vec_poin.back().x, pan_poin.y - vec_poin.back().y};
        float pan_len = attribute::Points::euclid_Dis(pan_poin, vec_poin.back());
        //向量夹角
        float vec_theta = acos((vec[0] * pan[0] + vec[1] * pan[1]) / (vec_len * pan_len));
        return vec_theta;
    }

}

#endif