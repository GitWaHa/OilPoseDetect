#pragma once

#include <stdio.h>
#include <string>
#include <iostream>

extern "C" void TSDF_Fusion(const char *data_folder, int frame_nums, const float *target_pos, const char *save_path);