#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>

std::vector<std::vector<std::vector<unsigned char>>> readPNG(const std::string& filename);
std::vector<std::vector<float>> readCSV(const std::string& filename);

void showRGBImage(const std::vector<std::vector<std::vector<unsigned char>>>& image);
void showcsvImage(const std::vector<std::vector<float>>& matrix);
void showDepthImage(const std::vector<std::vector<float>>& matrix);