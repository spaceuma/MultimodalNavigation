#include "imageUtils.hpp"

std::vector<std::vector<std::vector<unsigned char>>> readPNG(const std::string& filename) 
{
    std::vector<std::vector<std::vector<unsigned char>>> image;

    // Read the image from the PNG file
    cv::Mat cvImage = cv::imread(filename, cv::IMREAD_UNCHANGED);

    if (cvImage.empty()) {
        std::cerr << "Error reading the image: " << filename << std::endl;
        // Return an empty vector in case of an error
        return image;
    }

    // Get the height, width, and number of channels of the image
    int imgHeight = cvImage.rows;
    int imgWidth = cvImage.cols;
    int imgChannels = cvImage.channels();

    // Resize the vector to match the image dimensions
    image.resize(imgHeight, std::vector<std::vector<unsigned char>>(imgWidth, std::vector<unsigned char>(imgChannels)));

    // Copy the data from the image to the vector
    for (int i = 0; i < imgHeight; ++i) {
        for (int j = 0; j < imgWidth; ++j) {
            cv::Vec3b pixel = cvImage.at<cv::Vec3b>(i, j);
            for (int c = 0; c < imgChannels; ++c) {
                image[i][j][c] = pixel[imgChannels - c - 1];  // Reverse the order of channels (BGR to RGB)
            }
        }
    }

    return image;
}

void showRGBImage(const std::vector<std::vector<std::vector<unsigned char>>>& image) 
{
    if (image.empty() || image[0].empty()) {
        std::cerr << "Error: Empty image vector." << std::endl;
        return;
    }

    int height = image.size();
    int width  = image[0].size();

    cv::Mat cvImageFromMat = cv::Mat(cv::Size(width, height), CV_8UC3);

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            // Swap the red and blue channels (BGR instead of RGB)
            cvImageFromMat.at<cv::Vec3b>(i, j) = cv::Vec3b(
                image[i][j][2], // Blue channel
                image[i][j][1], // Green channel
                image[i][j][0]  // Red channel
            );
        }
    }

    // Display the image
    cv::imshow("Created Image", cvImageFromMat);
    cv::waitKey(10);
}


std::vector<std::vector<float>> readCSV(const std::string& filename) 
{
    std::vector<std::vector<float>> matrix;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error while opening CSV: " << filename << std::endl;
        return matrix;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::vector<float> row;
        std::stringstream ss(line);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stof(cell));
        }

        matrix.push_back(row);
    }

    file.close();
    return matrix;
}

void showcsvImage(const std::vector<std::vector<float>>& matrix) 
{
    if (matrix.empty() || matrix[0].empty()) {
        std::cerr << "Empty." << std::endl;
        return;
    }

    int rows = matrix.size();
    int cols = matrix[0].size();

    cv::Mat normalizedImage(rows, cols, CV_32FC1);
    cv::Mat scaledImage(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            normalizedImage.at<float>(i, j) = matrix[i][j];
        }
    }

    cv::normalize(normalizedImage, scaledImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::imshow("Thermal Image", scaledImage);
    cv::waitKey(10);
}


void showDepthImage(const std::vector<std::vector<float>>& matrix) 
{
    if (matrix.empty() || matrix[0].empty()) {
        std::cerr << "Empty." << std::endl;
        return;
    }

    int rows = matrix.size();
    int cols = matrix[0].size();

    cv::Mat normalizedImage(rows, cols, CV_32FC1);
    cv::Mat scaledImage(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            normalizedImage.at<float>(i, j) = matrix[i][j];
        }
    }

    cv::normalize(normalizedImage, scaledImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat falseColorsMap;
    applyColorMap(scaledImage, falseColorsMap, cv::COLORMAP_OCEAN);

    cv::imshow("Depth Image", falseColorsMap);
    cv::waitKey(10);
}