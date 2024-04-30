#include "MeshViewer.h"
// #include <GL/glew.h>

int main()
{
    
    MeshViewer::MeshViewer *a;
    // Mesh_Viewer::MeshViewer1 *a;
    a = new MeshViewer::MeshViewer();
    // sample();
    a->run();
    return 0;
}