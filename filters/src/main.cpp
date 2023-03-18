#include "filters.hpp"

#include "opencv2/opencv.hpp"

#include <iostream>
#include <string>

int test_color_filter(int argc, char* argv[]) {
    std::string color;
    switch (argc) {
        case 1:
            std::cerr << "Missing color." << std::endl;
            return 1;
        case 2:
            std::cerr << "Missing image file." << std::endl;
            return 2;
        default:
            color = argv[1];
    }

    cv::Scalar upper, lower;
    if (color == "red") {
        upper = filters::predefined_colors::red_upper;
        lower = filters::predefined_colors::red_lower;
    } else if (color == "green") {
        upper = filters::predefined_colors::green_upper;
        lower = filters::predefined_colors::green_lower;
    } else if (color == "blue") {
        upper = filters::predefined_colors::blue_upper;
        lower = filters::predefined_colors::blue_lower;
    } else {
        std::cerr << "Unkown color: " << color << std::endl;
        return 3;
    }

    cv::Mat image, mask;
    int key;
    for (int i = 2; i < argc; i++) {
        image = cv::imread(argv[i]);
        cv::resize(image, image, cv::Size(image.cols * 800 / image.rows, 800));

        //cv::bitwise_and(image, image, mask, create_color_mask(image, lower, upper));
        mask = filters::create_color_mask(image, lower, upper);

        cv::imshow("original", image);
        cv::imshow("mask image", mask);
        key = cv::waitKey();
        if (key == 27) {//ESC
            break;
        } else if (key == 115) {//s
            cv::Mat output;
            cv::bitwise_and(image, image, output, mask);
            cv::imwrite("example_image" + std::to_string(i) + ".png", output);
        }
    }

    return 0;
}

std::map<int, std::string> CV_TYPES_MAP = {
    {0, "CV_8UC1"},
    {1, "CV_8SC1"},
    {2, "CV_16UC1"},
    {3, "CV_16SC1"},
    {4, "CV_32SC1"},
    {5, "CV_32FC1"},
    {6, "CV_64FC1"},
    {8, "CV_8UC2"},
    {9, "CV_8SC2"},
    {10, "CV_16UC2"},
    {11, "CV_16SC2"},
    {12, "CV_32SC2"},
    {13, "CV_32FC2"},
    {14, "CV_64FC2"},
    {16, "CV_8UC3"},
    {17, "CV_8SC3"},
    {18, "CV_16UC3"},
    {19, "CV_16SC3"},
    {20, "CV_32SC3"},
    {21, "CV_32FC3"},
    {22, "CV_64FC3"},
    {24, "CV_8UC4"},
    {25, "CV_8SC4"},
    {26, "CV_16UC4"},
    {27, "CV_16SC4"},
    {28, "CV_32SC4"},
    {29, "CV_32FC4"},
    {30, "CV_64FC4"},
};

int test_range_filter(int argc, char* argv[]) {
    switch (argc) {
        case 1:
            std::cerr << "Missing image file." << std::endl;
            return 1;
        case 2:
            std::cerr << "Missing depth file." << std::endl;
            return 2;
        default:
            break;
    }

    cv::Mat image = cv::imread(argv[1]);
    cv::Mat depth = cv::imread(argv[2], cv::IMREAD_ANYDEPTH);
    cv::Size size(image.cols * 800 / image.rows, 800);

    cv::resize(image, image, size);
    cv::resize(depth, depth, size);

    cv::Mat mask = filters::create_range_mask(depth, 0.7f, 14.995f, 15.0f);

    cv::imshow("original", image);
    cv::imshow("mask image", mask);
    int key = cv::waitKey();
    if (key == 115) {//s
        cv::Mat output;
        cv::bitwise_and(image, image, output, mask);
        cv::imwrite("images/depth_masked_example_image.png", output);
    }

    return 0;
}

int main(int argc, char* argv[]) {
    return test_range_filter(argc, argv);
}