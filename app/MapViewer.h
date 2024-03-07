// #include "pangolin_viewer/color_scheme.h"

#include <memory>
#include <mutex>

#include <pangolin/pangolin.h>



class MapViewer {
    public:
        MapViewer();

        void run();
        void shutdown();

    private:
        // Create menu panel
	    void create_menu_panel();
        // void draw_current_cam_pose();
        // void draw_camera(float w);
        // void draw_frustrum(const float w);

	    void draw_line(const float x1, const float y1, const float z1,
                   const float x2, const float y2, const float z2) const;



        // menu panel
        std::unique_ptr<pangolin::Var<bool>> menu_terminate_;  
        static constexpr float map_viewer_width_ = 1024;
   	    static constexpr float map_viewer_height_ = 768;                
        float viewpoint_x_=0.0, viewpoint_y_=-10.0, viewpoint_z_=-1.0, viewpoint_f_=2000.0; 

        pangolin::View d_cam_;
        std::unique_ptr<pangolin::OpenGlRenderState> s_cam_;
        pangolin::OpenGlMatrix gl_cam_pose_wc_;
        bool is_terminated_ = false;
};


