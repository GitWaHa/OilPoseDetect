#pragma once

#include <vector>
#include <string>

class Fusion
{
private:
    /* data */
public:
    Fusion(/* args */) = default;
    virtual ~Fusion() = default;

    virtual void fusion(std::string img_folder, int num, const float *target_pos, std::string save_ply_path) = 0;
};
