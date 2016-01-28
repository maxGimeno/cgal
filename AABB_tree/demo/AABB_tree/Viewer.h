#ifndef VIEWER_H
#define VIEWER_H

#include <qglviewer.h>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>


// forward declarations
class QWidget;
class Scene;
struct axis_data
{
    std::vector<float> *vertices;
    std::vector<float> *normals;
    std::vector<float> *colors;
};

class Viewer : public QGLViewer{

  Q_OBJECT

public:
  Viewer(QWidget * parent);

  // overload several QGLViewer virtual functions
  void draw();

  /*renders the scene in a frame buffer object, in a gray scale that represents the depth value of the rendered objects,
  so it can use glReadPixels and determine the z-value of a point clicked in the scene. To do so the function requires a vector containing
  all the programs used in the draw function of the scene, so it can use them with a special fragment shader.
    */
  virtual void initializeGL();
  void setScene(Scene* pScene);
  bool areOpenGLBuffersInitialized;
  bool settingPivotPoint;


protected:
  virtual void mousePressEvent(QMouseEvent* e);
  virtual void mouseReleaseEvent(QMouseEvent* e);
  
private:
  Scene* m_pScene;
  bool m_custom_mouse;
  QOpenGLFunctions *gl;



}; // end class Viewer

#endif // VIEWER_H
