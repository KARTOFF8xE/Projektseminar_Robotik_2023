#include "encoding_filter.hpp"

filters::encoding_info_t filters::get_encoding_info(std::string msg_encoding) {
    encoding_info_t encoding_info;

    if (msg_encoding == sensor_msgs::image_encodings::RGB8) {
        encoding_info.type = CV_8UC3;
        encoding_info.conversion = cv::COLOR_RGB2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::RGBA8) {
        encoding_info.type = CV_8UC4;
        encoding_info.conversion = cv::COLOR_RGBA2BGRA;
    } else if (msg_encoding == sensor_msgs::image_encodings::RGB16) {
        encoding_info.type = CV_8UC3;
        encoding_info.conversion = cv::COLOR_RGB2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::RGBA16) {
        encoding_info.type = CV_8UC4;
        encoding_info.conversion = cv::COLOR_RGBA2BGRA;
    } else if (msg_encoding == sensor_msgs::image_encodings::BGR8) {
        encoding_info.type = CV_8UC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::BGRA8) {
        encoding_info.type = CV_8UC4;
    } else if (msg_encoding == sensor_msgs::image_encodings::BGR16) {
        encoding_info.type = CV_8UC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::BGRA16) {
        encoding_info.type = CV_8UC4;
    } else if (msg_encoding == sensor_msgs::image_encodings::MONO8) {
        encoding_info.type = CV_8UC1;
        encoding_info.conversion = cv::COLOR_GRAY2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::MONO16) {
        encoding_info.type = CV_8UC1;
        encoding_info.conversion = cv::COLOR_GRAY2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
        encoding_info.type = CV_8UC1;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_8UC2) {
        encoding_info.type = CV_8UC2;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_8UC3) {
        encoding_info.type = CV_8UC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_8UC4) {
        encoding_info.type = CV_8UC4;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_8SC1) {
        encoding_info.type = CV_8SC1;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_8SC2) {
        encoding_info.type = CV_8SC2;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_8SC3) {
        encoding_info.type = CV_8SC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_8SC4) {
        encoding_info.type = CV_8SC4;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        encoding_info.type = CV_16UC1;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_16UC2) {
        encoding_info.type = CV_16UC2;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_16UC3) {
        encoding_info.type = CV_16UC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_16UC4) {
        encoding_info.type = CV_16UC4;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_16SC1) {
        encoding_info.type = CV_16SC1;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_16SC2) {
        encoding_info.type = CV_16SC2;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_16SC3) {
        encoding_info.type = CV_16SC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_16SC4) {
        encoding_info.type = CV_16SC4;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_32SC1) {
        encoding_info.type = CV_32SC1;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_32SC2) {
        encoding_info.type = CV_32SC2;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_32SC3) {
        encoding_info.type = CV_32SC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_32SC4) {
        encoding_info.type = CV_32SC4;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        encoding_info.type = CV_32FC1;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_32FC2) {
        encoding_info.type = CV_32FC2;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_32FC3) {
        encoding_info.type = CV_32FC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_32FC4) {
        encoding_info.type = CV_32FC4;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_64FC1) {
        encoding_info.type = CV_64FC1;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_64FC2) {
        encoding_info.type = CV_64FC2;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_64FC3) {
        encoding_info.type = CV_64FC3;
    } else if (msg_encoding == sensor_msgs::image_encodings::TYPE_64FC4) {
        encoding_info.type = CV_64FC4;
    //Ref: https://stackoverflow.com/questions/33634628/what-is-the-meaning-of-bayer-pattern-code-in-opencv
    } else if (msg_encoding == sensor_msgs::image_encodings::BAYER_RGGB8) { // BG == RGGB
        encoding_info.type = CV_8UC4;
        encoding_info.conversion  = cv::COLOR_BayerBG2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::BAYER_BGGR8) { // RG == BGGR
        encoding_info.type = CV_8UC4;
        encoding_info.conversion  = cv::COLOR_BayerRG2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::BAYER_GBRG8) { // GR == GBRG
        encoding_info.type = CV_8UC4;
        encoding_info.conversion  = cv::COLOR_BayerGR2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::BAYER_GRBG8) { // GB == GRBG
        encoding_info.type = CV_8UC4;
        encoding_info.conversion  = cv::COLOR_BayerGB2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::BAYER_RGGB16) { // BG == RGGB
        encoding_info.type = CV_16UC4;
        encoding_info.conversion  = cv::COLOR_BayerBG2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::BAYER_BGGR16) { // RG == BGGR
        encoding_info.type = CV_16UC4;
        encoding_info.conversion  = cv::COLOR_BayerRG2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::BAYER_GBRG16) { // GR == GBRG
        encoding_info.type = CV_16UC4;
        encoding_info.conversion  = cv::COLOR_BayerGR2BGR;
    } else if (msg_encoding == sensor_msgs::image_encodings::BAYER_GRBG16) { // GB == GRBG
        encoding_info.type = CV_16UC4;
        encoding_info.conversion  = cv::COLOR_BayerGB2BGR;
    //TODO: https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
    } else if (msg_encoding == sensor_msgs::image_encodings::YUV422) {// UYUV version: http://www.fourcc.org/pixel-format/yuv-uyvy
        encoding_info.type = CV_8UC3;
        encoding_info.conversion  = cv::COLOR_YUV2BGR_UYVY;
    } else if (msg_encoding == sensor_msgs::image_encodings::YUV422_YUY2) { // YUYV version: http://www.fourcc.org/pixel-format/yuv-yuy2/
        encoding_info.type = CV_8UC3;
        encoding_info.conversion  = cv::COLOR_YUV2RGB_YUYV;
    } else {
        throw std::invalid_argument("Can not decompose encoding: " + msg_encoding);
    }

    return encoding_info;
}