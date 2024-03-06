#include "helper.h"
#include <map>

void verifyPoints(std::vector<double> &xyz, bool *usable_point, int &number_vertices, Eigen::Matrix3d K, cv::Mat mask) {

    double fx = K(0,0);
    double fy = K(1,1);
    double cx = K(0,2);
    double cy = K(1,2);
    int error_counter=0;
    
    int num_points = 0;
    for (int i = 0; i< number_vertices; i++) {
        usable_point[i] = false;
        double x = xyz[i*3 + 0];
        double y = xyz[i*3 + 1];
        double z = xyz[i*3 + 2];
        // if (z < 50) {
        //     std::cout <<" asad\n";
        // }
        
        double u = fx*x/z + cx;
        double v = fy*y/z + cy;
        // p0.push_back(cv::Point2d(u,v));
        // also masking here!
        if(u < 2 || v < 5 || u > 360 || v > 288) {
            // std::cerr << "Error, u or v is not in the boarder" << std::endl;
            // std::cout << u << " " << v << std::endl;
            error_counter++;
            continue;
        }
        if (mask.at<uchar>(int(v),int(u)) == 255) {
            // p0.push_back(cv::Point2d(u,v));
            usable_point[i] = true;
            num_points++;
        }

    }
    number_vertices = num_points;
}


void remapping(std::vector<double> &xyz, std::vector<int> &faces, double *verticess, bool *usabale_faces, bool *usable_point, int &number_faces, int &num_points) {
    int num_faces = 0;
    std::map<int,int> mapping;
    std::vector<int> new_faces;
    for(int i=0;i<number_faces;i++) {
        usabale_faces[i] = false;
        int f1 = faces[i*3];
        int f2 = faces[i*3+1];
        int f3 = faces[i*3+2];

        if(usable_point[f1] && usable_point[f2] && usable_point[f3]) {
            usabale_faces[i] = true;
            num_faces++;
            // v_mask[f1*num_points + f1] = 1;
            mapping[f1] = 0;
            mapping[f2] = 0;
            mapping[f3] = 0;
            
        } 
    }
    // std::cout << mapping.size() << std::endl;

    int counter = 0;
    for (auto& mapp : mapping) {
        mapp.second = counter;
        counter++;
    }
    // std::cout << num_points << std::endl;
    // exit(1);
    num_points = counter;
    double vertices[num_points*3];
    // double reference[num_points*3];
    // counter = 0;
    for(int i=0;i<number_faces;i++) {
        if(usabale_faces[i]) {
            int f1 = faces[i*3];
            int f2 = faces[i*3+1];
            int f3 = faces[i*3+2];    
            
            int f1_new = mapping[f1];
            int f2_new = mapping[f2];
            int f3_new = mapping[f3];
            // counter++;
            // if ((f1_new >= 2575) || (f2_new >= 2575) || (f3_new >= 2575))
            //     counter++;
            new_faces.push_back(f1_new);
            new_faces.push_back(f2_new);
            new_faces.push_back(f3_new);

            vertices[f1_new*3] = xyz[f1*3];
            vertices[f1_new*3+1] = xyz[f1*3+1];
            vertices[f1_new*3+2] = xyz[f1*3+2];

            vertices[f2_new*3] = xyz[f2*3];
            vertices[f2_new*3+1] = xyz[f2*3+1];
            vertices[f2_new*3+2] = xyz[f2*3+2];

            vertices[f3_new*3] = xyz[f3*3];
            vertices[f3_new*3+1] = xyz[f3*3+1];
            vertices[f3_new*3+2] = xyz[f3*3+2];

            // reference[f1_new*3] = xyz[f1*3];
            // reference[f1_new*3+1] = xyz[f1*3+1];
            // reference[f1_new*3+2] = xyz[f1*3+2];

            // reference[f2_new*3] = xyz[f2*3];
            // reference[f2_new*3+1] = xyz[f2*3+1];
            // reference[f2_new*3+2] = xyz[f2*3+2];

            // reference[f3_new*3] = xyz[f3*3];
            // reference[f3_new*3+1] = xyz[f3*3+1];
            // reference[f3_new*3+2] = xyz[f3*3+2];

            
        }
    }
    number_faces = num_faces;
    verticess = vertices;
}