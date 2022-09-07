#ifndef _LICIFIT_H_
#define _LICIFIT_H_

#include <cmath>
#include "attribute.h"

namespace lsfitting
{
    float fitSigma(const std::vector<attribute::Points>& point, const attribute::Line& line)
    {
        float sigma_sum = 0;
        for(auto i = point.begin(); i != point.end(); i++)
        {
            sigma_sum += fabs(((i->x) * line.A + (i->y) * line.B + line.C) / sqrt(pow(line.A, 2) + pow(line.B, 2)));
        }
        return sigma_sum;
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

}

#endif