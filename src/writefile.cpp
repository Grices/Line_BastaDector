// #include <pcl/point_cloud.h>
// #include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "include/resbyfit.h"
#include "include/resbyvec.h"
#include "include/vectencap.h"

class Readdata
{
    public:
        struct ldpoints
        {
            float x, y, dis, theta, power;
            ldpoints() {}
            ldpoints(float _x, float _y, float _dis, float _the, float _pow) : x(_x), y(_y), dis(_dis), theta(_the), power(_pow) {}
        };
    public:
        std::vector<attribute::Points> mread_point;
    public:
        void operator()(void)
        {
            std::ifstream iFile("/home/penghua/BasestaDect/src/dzbin_data/lds_pointdz9.bin", std::ios::binary);
            if(!iFile.is_open())
            {
                std::cout << "File not exits. " << std::endl;
                return;
            }
            else
            {
                int datasize = 0;
                int fileti = 0; //循环写入

                iFile.read((char*)(&datasize), sizeof(datasize));
                this->mread_point.clear();
                this->mread_point.reserve(datasize);
                for(int i = 0; i < datasize; ++i)
                {
                    ldpoints tmp;
                    iFile.read(reinterpret_cast<char*>(&tmp), sizeof(ldpoints));
                    // attribute::Points tnp(tmp.x, tmp.y, tmp.dis, tmp.theta, tmp.power);
                    attribute::Points tnp(tmp.x, tmp.y);
                    this->mread_point.emplace_back(tnp);
                }


                /* 原始数据 */
                std::ofstream oFile("/home/penghua/BasestaDect/src/dz0.csv", std::ios::trunc | std::ios::binary); //单个写入
                oFile.seekp(std::ios::beg);
                
                for(std::vector<attribute::Points>::iterator j = mread_point.begin(); j != mread_point.end(); j++)
                {
                    if(abs(j->x) <3000.0f && abs(j->y) < 3000.0f) 
                    {
                        oFile << j->x << "," << j->y << std::endl;
                    }
                }

                /* 预处理 */
                // attribute::Sacnpcluter ori_poin(this->mread_point, 130);
                // ori_poin.deleteSmall(10);
                // ori_poin.mergeCluster(100.0f);

                // std::vector<attribute::Points> cav; //平滑
                // for(auto x = ori_poin.cluster.begin(); x != ori_poin.cluster.end(); x++)
                // {
                //     char filepath[128]; //循环写入
                //     std::sprintf(filepath, "/home/penghua/BasestaDect/src/csv_data/dztz_%d.csv", fileti); //循环写入
                //     std::ofstream oFile(filepath, std::ios::trunc | std::ios::binary); //循环写入
                //     // for(auto y = x->begin(); y != x->end(); y++)
                //     cav.clear(); //平滑
                //     cav = attribute::meanFilter(*x, 3);
                //     cav = attribute::downSampling(cav);
                //     // cav = attribute::laplaceSharpen(cav);
                //     cav = attribute::preSmooth(cav); //平滑
                //     for(auto y = cav.begin(); y != cav.end(); y++) //平滑
                //     {
                //         // std::cout << y->x << "," << y->y << "," << y->dis << "," << y->theta << "," << y->power << std::endl;
                //         oFile << y->x << "," << y->y << std::endl;
                //     }

                //     fileti++; //循环写入
                // }

                /* 提取线段 */
                // std::vector<attribute::Line>* lisd = getTarget(this->mread_point);
                
                // for(auto k = lisd->begin(); k != lisd->end(); k++)
                // {
                //     char filepath[128]; //循环写入
                //     std::sprintf(filepath, "/home/penghua/BasestaDect/src/csv_data/dztz_%d.csv", fileti); //循环写入
                //     std::ofstream oFile(filepath, std::ios::trunc | std::ios::binary); //循环写入
                //     for(auto l = k->combine.begin(); l != k->combine.end(); l++)
                //     {
                //         oFile << l->x << "," << l->y << "," << l->dis << "," << l->theta << "," << l->power << std::endl;
                //     }
                //     oFile.close(); //循环写入
                //     fileti++; //循环写入
                // }
                // attribute::Points* target = goalPoint(getTarget(this->mread_point));
                // if(target != NULL)
                // {
                //     std::cout << "GOAL: " << target->x << ", " << target->y << std::endl;
                // }
                // else
                // {
                //     std::cout << "There is no target" << std::endl;
                // }

                /* 提取向量 */
                std::vector<attribute::Vect> vcsd = getVect(this->mread_point);
                vcsd = vectMerge(vcsd);
                for(auto k = vcsd.begin(); k != vcsd.end(); k++)
                {
                    char filepath[128]; //循环写入
                    std::sprintf(filepath, "/home/penghua/BasestaDect/src/csv_data/dztz_%d.csv", fileti); //循环写入
                    std::ofstream oFile(filepath, std::ios::trunc | std::ios::binary); //循环写入
                    for(auto l = k->com.begin(); l != k->com.end(); l++)
                    {
                        // oFile << l->x << "," << l->y << "," << l->dis << "," << l->theta << "," << l->power << std::endl;
                        oFile << l->x << "," << l->y << std::endl;
                    }
                    // std::cout << "file" << std::endl;
                    oFile.close(); //循环写入
                    fileti++; //循环写入
                }

                /* 获取目标点 */
                // attribute::Points* point = getFinalgoal(vcsd);
                // if(point != NULL)
                // {
                //     std::cout << point->x << ", " << point->y << std::endl;
                // }
                // else
                // { 
                //     std::cout << "There is no goal." << std::endl; 
                // }
                // delete point;
                /* 封装方式获取目标点 */
                Extracvect tess(this->mread_point);
                attribute::Points gg = tess.showGoal();
                std::cout << gg.x << ", " << gg.y << std::endl;
                // oFile.close(); //单个写入
            }
            iFile.close();
        }
};

int main()
{
    Readdata wr;
    wr();

    return 0;
}