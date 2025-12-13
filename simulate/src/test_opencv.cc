#include <opencv2/opencv.hpp>
#include <iostream>
 
int main() {
    // Load the image
    cv::Mat image = cv::imread("/home/lupinjia/Downloads/Astronaut.png", cv::IMREAD_GRAYSCALE);
    
    // print the data of image
    if(image.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
        return -1;
    }
    std::cout << "Image data: " << image << std::endl;
    // Display the image in a window
    cv::imshow("Displayed Image", image);
 
    // Wait for a key press before closing the window
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}