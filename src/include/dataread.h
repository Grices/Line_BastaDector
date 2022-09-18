#ifndef _DATAREADE_H_
#define _DATAREADE_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "attribute.h"

class ReadtoPoints
{
    private:
        struct lds_point
        {
            float x, y, dis, theta, power;
            lds_point() {}
            lds_point(float _x, float _y, float _dis, float _the, float _pow) : x(_x), y(_y), dis(_dis), theta(_the), power(_pow) {}
        };

    public:
        std::vector<attribute::Points> m_read_points;
        std::vector<std::vector<attribute::Points>> mf_points;
        
    public:
        // std::vector<attribute::Points>& GetPoints(void)
        // {
        //     return this->m_read_points;
        // }

        ReadtoPoints(const std::string& filepath)
        {
            std::ifstream iFile(filepath, std::ios::binary);
            if(!iFile.is_open())
            {
                std::cout << "File not exit." << std::endl;
                return;
            }
            else
            {
                int data_size = 0;
                while (iFile.peek() != EOF)
                {
                    iFile.read((char*)(&data_size), sizeof(data_size));
                    this->m_read_points.clear();
                    this->m_read_points.reserve(data_size);
                    for(int i = 0; i < data_size; ++i)
                    {
                        lds_point tmp;
                        iFile.read(reinterpret_cast<char*>(&tmp), sizeof(lds_point));
                        // attribute::Points tnp(tmp.x, tmp.y, tmp.dis, tmp.theta, tmp.power);
                        attribute::Points tnp(tmp.x, tmp.y);
                        m_read_points.emplace_back(tnp);
                    }
                    this->mf_points.emplace_back(m_read_points);
                    // std::cout << mf_points.size() << ", " << mf_points.front().size() << std::endl;
                }
            }
            iFile.close();
        }        
};

#endif