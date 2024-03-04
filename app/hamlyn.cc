// Include Libraries
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>




#include <Eigen/Core>

// Namespace to nullify use of cv::function(); syntax
using namespace std;
using namespace cv;

void readObj(std::vector<double> &xyz, std::vector<int> &faces, int &number_vertices, int &number_faces) {
    std::string file_path = "../data/Hamlyn/ReferenceMesh2.obj";
    // std::vector<double> points;
    // Dateistream-Objekt zum Lesen der Objektdatei erstellen
    std::ifstream obj_file(file_path);

    // Überprüfen, ob die Datei erfolgreich geöffnet wurde
    if (!obj_file.is_open()) {
        std::cerr << "Incorrect path to the Obj-file." << std::endl;
        return;
    }

    // Lesen und Ausgabe des Inhalts der Objektdatei
    std::string line;
    std::getline(obj_file, line);
    std::getline(obj_file, line);
    std::getline(obj_file, line);
    std::stringstream sstream(line);
    std::istringstream iss;
    std::string word;
    std::string token;
    sstream >> word;sstream >> word;
    sstream >> word;
    sstream >> word;sstream >> word;
    // std::cout << word << std::endl;
    number_vertices = stoi(word);

    std::getline(obj_file, line);
    sstream.str(line);
    sstream.seekg(0);
    sstream.seekp(0);
    // std::stringstream sstream(line);
    sstream >> word;sstream >> word;
    sstream >> word;
    sstream >> word;sstream >> word;
    // std::cout << word << std::endl;
    number_faces = stoi(word);

    // std::cout << number << std::endl;
    
    // double points[number_vertices * 3];
    // double faces[number_faces * 3];
    int idx_points = 0;
    while (std::getline(obj_file, line)) {
        if(line[0] != 'v' && line[0] != 'f')
            continue;
        if(line[1] == 'n')
            continue;
            
        if (line[0] == 'v'){
            // std::stringstream sstream(line);
            sstream.str(line);
            sstream.seekg(0);
            sstream.seekp(0);
            sstream >> word;sstream >> word;
            double x = stod(word);
            sstream >> word;
            double y = stod(word);
            sstream >> word;
            double z = stod(word);
            xyz.push_back(x);
            xyz.push_back(y);
            xyz.push_back(z);
            idx_points++;
        } else if(line[0] == 'f') {
            sstream.str(line);
            sstream.seekg(0);
            sstream.seekp(0);
            sstream >> word;sstream >> word;
            iss.str(word);
            std::getline(iss, token, '/');
            int f1 = stoi(token);
            sstream >> word;
            iss.str(word);
            
            std::getline(iss, token, '/');
            int f2 = stoi(token);
            sstream >> word;
            iss.str(word);
            
            std::getline(iss, token, '/');
            int f3 = stoi(token);
            faces.push_back(f1);
            faces.push_back(f2);
            faces.push_back(f3);
        }
        // std::cout << line[0] == 'v' << std::endl;
        // std::cout << line << std::endl;
        // std::stringstream sstream(line);
  		// std::string word;

        // while(sstream >> word) {
        //     std::cout << word << std::endl;
        // }
        
    }
    
    // double *tmp = points.data();
    // xyz = tmp;
    // Schließen der Datei
    obj_file.close();
    // for (int i = 0; i< 1680; i++) {
    //     std::cout << xyz[i*3 + 0] << " " << xyz[i*3 + 1] << " " << xyz[i*3 + 2] << " " << std::endl;  
    // }
    // exit(1);
}


int main()
{
    Mat pre_frame, cur_frame, pre_frame_gray, cur_frame_gray;
    
    VideoCapture cap("../data/Hamlyn/output.mp4");
    if (!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
    }
    Eigen::Matrix3d K;
    K <<    391.656525, 0.000000, 165.964371,
            0.000000, 426.835144, 154.498138,
            0.000000, 0.000000, 1.000000;
    double fx = K(0,0);
    double fy = K(1,1);
    double cx = K(0,2);
    double cy = K(1,2);
    std::vector<double> xyz;
    std::vector<int> faces;
    int number_vertices, number_faces;

    // Read Object file
    readObj(xyz, faces, number_vertices, number_faces);
    int frameWidth = 360;
    int frameHeight = 288;
    int frameRate = 30;

    // Erstellen Sie einen VideoWriter, um das Video zu schreiben
    cv::VideoWriter video("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), frameRate, cv::Size(frameWidth,frameHeight));


    cap >> pre_frame;

    // creating a mask
    
    cv::Mat hsvImage;
    cv::cvtColor(pre_frame, hsvImage, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsvImage, channels);
    cv::Mat pre_processing_mask;
    int thresholdValue = 30; // Passen Sie den Schwellenwert nach Bedarf an
    cv::threshold(channels[2], pre_processing_mask, thresholdValue, 255, cv::THRESH_BINARY);
    cv::imshow("Helligkeitsschwellenwert", pre_processing_mask);
    // cv::waitKey(0);

    // create reference observation
    
    std::vector<cv::Point2f> p0;// = {point0, cv::Point2f(180,140), cv::Point2f(200, 200)};
    int error_counter=0;
    for (int i = 0; i< number_vertices; i++) {
        double x = xyz[i*3 + 0];
        double y = xyz[i*3 + 1];
        double z = xyz[i*3 + 2];
        if (z < 50) {
            std::cout <<" asad\n";
        }
        
        double u = fx*x/z + cx;
        double v = fy*y/z + cy;
        // p0.push_back(cv::Point2d(u,v));
        if(u < 2 || v < 5 || u > 360 || v > 288) {
            std::cerr << "Error, u or v is not in the boarder" << std::endl;
            error_counter++;
            continue;
        }
        if (pre_processing_mask.at<uchar>(int(v),int(u)) == 255)
            p0.push_back(cv::Point2d(u,v));

    }


    // VideoCapture cap("../data/Hamlyn/output.mp4");
        
    // Print error message if the stream is invalid
    
    
 
    
    cvtColor(pre_frame, pre_frame_gray, COLOR_BGR2GRAY);

    Mat mask = Mat::zeros(pre_frame_gray.size(), pre_frame_gray.type());
    vector<Scalar> colors;
    colors.push_back(Scalar(255,0,0));
    // Read the frames to the last frame
    while (1)
    {
        // Initialise frame matrix
        
        cap >> cur_frame;
        
        if(cur_frame.empty())
            break;

        // feature tracking!
        cvtColor(cur_frame, cur_frame_gray, COLOR_BGR2GRAY);
        std::vector<cv::Point2f> p1;
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(pre_frame_gray, cur_frame_gray, p0, p1, status, err, Size(15,15),5, criteria);
        vector<Point2f> good_new;
 
        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(p1[i]);
                // Draw the tracks
                line(cur_frame, p1[i], p0[i], Scalar(0, 255, 0), 2);
                circle(cur_frame, p1[i], 2, Scalar(0, 0, 255), -1);
            }
        }
        
        imshow("Frame", cur_frame);
        // video.write(cur_frame);
        //wait 20 ms between successive frames and break the loop if key q is pressed
        int key = waitKey(50);
        if (key == 'q')
        {
            cout << "q key is pressed by the user. Stopping the video" << endl;
            break;
        }
 
 
  }
  // Release the video capture object
  cap.release();
  destroyAllWindows();
  return 0;
}