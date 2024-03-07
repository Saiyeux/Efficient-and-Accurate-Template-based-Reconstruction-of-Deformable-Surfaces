#include "MapViewer.h"

#include <iostream>



MapViewer::MapViewer() {
    std::cout << "hasdasd asadi\n"; 
}

void MapViewer::create_menu_panel() {
	pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(230));
	menu_terminate_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Terminate", false, false));
}

void MapViewer::run() {
    pangolin::CreateWindowAndBind("STBR", 1024, 768);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
    s_cam_ = std::unique_ptr<pangolin::OpenGlRenderState>(
					new pangolin::OpenGlRenderState(
						pangolin::ProjectionMatrix(map_viewer_width_, map_viewer_height_, viewpoint_f_, viewpoint_f_,
															map_viewer_width_ / 2, map_viewer_height_ / 2, 0.1, 1e6),
															pangolin::ModelViewLookAt(viewpoint_x_, viewpoint_y_, viewpoint_z_,
															0,0,0, 0.0, -1.0, 0.0)));
    d_cam_ = pangolin::CreateDisplay()
					.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -map_viewer_width_ / map_viewer_height_)
					.SetHandler(new pangolin::Handler3D(*s_cam_));
    create_menu_panel();
    gl_cam_pose_wc_.SetIdentity();
		glClearColor(1.0f,1.0f,1.f,0.0f);

    while(1) {
        // set rendering state
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		d_cam_.Activate(*s_cam_);

        // draw triangles!



        pangolin::FinishFrame();
		if(is_terminated_ || *menu_terminate_) {
			break;
		}
    }

}