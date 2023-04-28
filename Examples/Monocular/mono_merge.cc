#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <stdio.h>
#include <sys/time.h>
#include <thread>

#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;
using namespace cv;

void LoadImages(const string& strPathToSequence, vector<string>& imageFilenames, vector<double>& timestamps);
void ExecuteSLAM(ORB_SLAM3::System& SLAM, vector<string>& iamgeFilenames, vector<double>& timestamps);

int main(int argc, char **argv) {
    if (argc != 6) {
        cerr << endl << "Usage: ./mono_nuc path_to_vocabulary path_to_settings1 path_to_sequence1 path_to_settings2 path_to_sequence2" << endl;        
        return 1;
    }

    // Retrieve paths to images
    vector<string> imageFilenames1;
    vector<string> imageFilenames2;
    vector<double> timestamps1;
    vector<double> timestamps2;
    LoadImages(string(argv[3]), imageFilenames1, timestamps1);
    LoadImages(string(argv[5]), imageFilenames2, timestamps2);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM1(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true, 0, "", "iphone_1");
    ORB_SLAM3::System SLAM2(argv[1], argv[4], ORB_SLAM3::System::MONOCULAR, true, 0, "", "iphone_2");

    thread RunSLAM1([&SLAM1, &imageFilenames1, &timestamps1] {
        ExecuteSLAM(SLAM1, imageFilenames1, timestamps1);
    });

    thread RunSLAM2([&SLAM2, &imageFilenames2, &timestamps2] {
        ExecuteSLAM(SLAM2, imageFilenames2, timestamps2);
    });

    RunSLAM1.join();
    RunSLAM2.join();
    
    return 0;
}

void LoadImages(const string& strPathToSequence, vector<string>& imageFilenames, vector<double>& timestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            timestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/";

    const int nTimes = timestamps.size();
    imageFilenames.resize(nTimes);

    for (int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        imageFilenames[i] = strPrefixLeft + ss.str() + ".jpg";
    }
}

void ExecuteSLAM(ORB_SLAM3::System& SLAM, vector<string>& iamgeFilenames, vector<double>& timestamps)
{
    // Vector for tracking time statistics
    int nImages = iamgeFilenames.size();
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    float imageScale = SLAM.GetImageScale();

    // Main loop
    cv::Mat im;
    for (int ni = 0; ni < nImages; ni++) {
        // Read image from file
        im = cv::imread(iamgeFilenames[ni], cv::IMREAD_UNCHANGED);
        double tframe = timestamps[ni];

        if (im.empty()) {
            cerr << endl
                 << "Failed to load image at: " << iamgeFilenames[ni] << endl;
            return;
        }

        if (imageScale != 1.0f) {
            int width = imageScale * im.cols;
            int height = imageScale * im.rows;
            cv::resize(im, im, cv::Size(width, height));
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, tframe);


        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = timestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - timestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;
}