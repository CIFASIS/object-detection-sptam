#include<iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include<dirent.h>
#include<string.h>

using namespace std;
using namespace cv;

int main()
{
    vector<String> filenames; // notice here that we are using the Opencv's embedded "String" class
    String folder = "/home/taihu/datasets/KITTI/00/image_0/"; // again we are using the Opencv's embedded "String" class

    glob(folder, filenames); // new function that does the job ;-)

    for(size_t i = 0; i < filenames.size(); ++i)
    {
        Mat src = imread(filenames[i]);

        if(!src.data)
            cerr << "Problem loading image!!!" << endl;
        else {
          imshow("output",src);
          waitKey(0);
        }
    }
}
