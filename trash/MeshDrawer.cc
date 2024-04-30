#include "MeshDrawer.h"
#include <pangolin/pangolin.h>

namespace Viewer {
    MeshDrawer::MeshDrawer() {

    }

    void MeshDrawer::addVertices(std::vector<Eigen::Vector3d> &vertices) {
        vertices_ = vertices;
    }

    void MeshDrawer::addTriangles(std::vector<Eigen::Vector3i> &triangles) {
        triangles_ = triangles;
    }

    void MeshDrawer::addTextureImage(cv::Mat &texture) {
        texture_ = texture.clone();
    }

    void MeshDrawer::drawMesh(double alpha, bool drawedges) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_TEXTURE_2D);
        cv::Mat fin_image;
        std::vector<cv::Mat> channels;
        channels.resize(3);
        cv::split(this->texture_, channels);

        cv::Mat AlphaChannel(this->texture_.rows, this->texture_.cols, CV_8UC1, cv::Scalar(alpha * 255));

        channels.push_back(AlphaChannel);
        cv::merge(channels, fin_image);

        GLuint texID;
        glGenTextures(1, &texID);
        glBindTexture(GL_TEXTURE_2D, texID);
        glPixelStoref(GL_UNPACK_ALIGNMENT, 1);
        glLineWidth(1);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, fin_image.cols, fin_image.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, fin_image.ptr());

        glBegin(GL_TRIANGLES);
        glColor3f(1, 1, 1); // set global color to white, otherwise this color will be (somehow) added to the texture

        for(int i=0; i < triangles_.size(); i++) {
            int t1 = triangles_[i].x();
            int t2 = triangles_[i].y();
            int t3 = triangles_[i].z();
            
            float v1_x = vertices_[t1].x();
            float v1_y = vertices_[t1].y();
            float v1_z = vertices_[t1].z();

            float v2_x = vertices_[t2].x();
            float v2_y = vertices_[t2].y();
            float v2_z = vertices_[t2].z();

            float v3_x = vertices_[t3].x();
            float v3_y = vertices_[t3].y();
            float v3_z = vertices_[t3].z();
         
            glTexCoord2f(v1_x/3, v1_y/3);
            glVertex3f(v1_x, v1_y, v1_z);
            glTexCoord2f(v2_x/3, v2_y/3);
            glVertex3f(v2_x, v2_y, v2_z);
            glTexCoord2f(v3_x/3, v3_y/3);
            glVertex3f(v3_x, v3_y, v3_z);
        }
        glEnd();
        glDeleteTextures(1, &texID); //TODO load all textures once and only remove when it change

        glDisable(GL_TEXTURE_2D);

        if(drawedges) {
            glBegin(GL_LINES);
            glColor3f(0.1, 0.1, 0.1);
            for(int i=0; i < triangles_.size(); i++) {
                int t1 = triangles_[i].x();
                int t2 = triangles_[i].y();
                int t3 = triangles_[i].z();

                float v1_x = vertices_[t1].x();
                float v1_y = vertices_[t1].y();
                float v1_z = vertices_[t1].z();

                float v2_x = vertices_[t2].x();
                float v2_y = vertices_[t2].y();
                float v2_z = vertices_[t2].z();

                float v3_x = vertices_[t3].x();
                float v3_y = vertices_[t3].y();
                float v3_z = vertices_[t3].z();

                glVertex3f(v1_x, v1_y, v1_z);
                glVertex3f(v2_x, v2_y, v2_z);
                
                glVertex3f(v1_x, v1_y, v1_z);
                glVertex3f(v3_x, v3_y, v3_z);
                
                glVertex3f(v3_x, v3_y, v3_z);
                glVertex3f(v2_x, v2_y, v2_z);
                
            }
            glEnd();
        }
    }
    
}