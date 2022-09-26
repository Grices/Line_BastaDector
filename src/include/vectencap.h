#ifndef _VECTNCAP_H_
#define _VECTNCAP_H_

#include "resbyvec.h"

class Extracvect
{
    private:
        attribute::Points finpoi;
    public:
        Extracvect(std::vector<attribute::Points>& gemini) 
        {
            std::vector<attribute::Vect> vct = getVect(gemini);
            vct = vectMerge(vct);
            attribute::Points* goa = getFinalgoal(vct);
            if(goa != NULL)
            {
                this->finpoi = *goa;
            }
            else
            {
                this->finpoi = attribute::Points(0.0f,0.0f);
            }
        }
        
        attribute::Points showGoal(void)
        {
            return this->finpoi;
        }
        
        ~Extracvect() {}

};


#endif