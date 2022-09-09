#ifndef _CIRCLEFIT_H_
#define _CIRCLEFIT_H_

#include "attribute.h"

namespace cirfiting
{
    // float Sigma(const std::vector<attribute::Points>& data, const attribute::Line& cirline, const float x_mean, const float y_mean)
    // {
    //     float sum = 0., dx, dy;
    //     for(int i = 0; i < data.size(); ++i)
    //     {
    //         dx = x_mean - circle.a;
    //         dy = y_mean - circle.b;
    //         sum += pow((sqrt(dx*dx + dy*dy) - circle.r), 2);
    //     }
    //     return sqrt(sum / (data.n));
    // }

    attribute::Line cirFit(const std::vector<attribute::Points>& vec_point)
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

        int i, iter, IterMAX = 10;
        float Xi, Yi, Zi;
        float Mz, Mxx, Mxy, Mxz, Myy, Myz, Mzz, Cov_xy, Var_z;
        Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0;
        for(i = 0; i < size; ++i)
        {
            Xi = vec_point[i].x - x_mean;
            Yi = vec_point[i].y - y_mean;
            Zi = Xi*Xi + Yi*Yi;

            Mxx += Xi*Xi; Mxy += Xi*Yi; Mxz += Xi*Zi;
            Myy += Yi*Yi; Myz += Yi*Zi; Mzz += Zi*Zi;
        }
        Mxx /= size; Mxy /= size; Mxz /= size;
        Myy /= size; Myz /= size; Mzz /= size;

        float A0, A1, A2, A3;
        Mz = Mxx + Myy;
        Cov_xy = Mxx*Myy - pow(Mxy,2);
        Var_z = Mzz - Mz*Mz;
        A3 = (4)*Mz; A2 = (-3)*Mz*Mz -Mzz;
        A1 = Var_z*Mz + (4)*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
        A0 = Mxz*(Mxz*Myy - Myz*Mxz) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
        float A22 = A2 + A2; float A33 = A3 + A3 + A3;

        float x = 0., y = A0;
        float Dy, xnew, ynew;
        for(iter = 0; iter < IterMAX; iter++)
        {
            Dy = A1 + x*(A22 + A33*x);
            xnew = x - y/Dy;
            if((xnew == x) || (!std::isfinite(xnew))) {break;}
            ynew = A0 + xnew*(A1 + xnew*(A2 + xnew*A3));
            if(abs(ynew) >= abs(y)) {break;}
            x = xnew; y = ynew;
        }

        float DET = x*x - x*Mz + Cov_xy;
        float Xcenter = (Mxz*(Myy - x) - Myz*Mxy) / DET / 2;
        float Ycenter = (Myz*(Mxx - x) - Mxz*Mxy) / DET / 2;
        
    }
}

#endif