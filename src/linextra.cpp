#include <iostream>
#include "include/release.h"

class Extrali
{
    private:
        attribute::Points* target;
    public:
        Extrali(std::vector<attribute::Points>& scanpoints)
        {
            this->target = goalPoint(getTarget(scanpoints));
        }
}; 

int main()
{
    std::cout << "Hello World!" << std::endl;
    return 0;
}