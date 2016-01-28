#include "Viewer.h"
#include "Scene.h"
#include <QMouseEvent>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QMessageBox>
#include <QDebug>
#include <manipulatedCameraFrame.h>
#include <cmath>

#include <QGLFunctions>
#include <CGAL/Qt/CreateOpenGLContext.h>

Viewer::Viewer(QWidget* parent)
  : QGLViewer(parent),
    m_pScene(NULL),
    m_custom_mouse(false)
{
    settingPivotPoint = false;
}

void Viewer::setScene(Scene* pScene)
{
    this->m_pScene = pScene;
}
void Viewer::draw()
{
    QGLViewer::draw();
    QOpenGLFunctions *gl = QOpenGLContext::currentContext()->functions();

    if(m_pScene != NULL )
    {
        gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        m_pScene->draw(this);
    }
    else
    {
        QMessageBox box;
        box.setText("Error : No scene");
        box.exec();
    }

   if(settingPivotPoint)
   {
    }
}

void Viewer::initializeGL()
{
  QGLViewer::initializeGL();
  setBackgroundColor(::Qt::white);
  m_pScene->initGL(this);
}



void Viewer::mousePressEvent(QMouseEvent* e)
{
    if ( e->modifiers() == Qt::ControlModifier  || frame_manipulation)
    {
        m_pScene->set_fast_distance(true);
        // Refresh distance function
        m_pScene->cutting_plane(true);
        m_custom_mouse = true;
    }

    //setMouseBinding(Qt::Key(0),Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION);
    if(settingPivotPoint)
    {
        makeCurrent();
        if(!setPivotPointFromPixelGLES(m_pScene->programs, camera(), e->pos()))
            camera()->setPivotPoint(sceneCenter());
       setVisualHintsMask(1,2000);


    }
    else if(!settingPivotPoint && frame_manipulation)
    {
        setMouseBinding(Qt::Key(0),Qt::NoModifier, Qt::LeftButton, FRAME, ROTATE);


    }
    else
    {
        setMouseBinding(Qt::NoModifier, Qt::LeftButton, CAMERA, ROTATE);

    }

    QGLViewer::mousePressEvent(e);
}

void Viewer::mouseReleaseEvent(QMouseEvent* e)
{
  if ( m_custom_mouse )
  {
    m_pScene->set_fast_distance(false);
    // Recompute distance function
    QApplication::setOverrideCursor(Qt::WaitCursor);
    m_pScene->cutting_plane(true);
    QApplication::restoreOverrideCursor();
      
    m_custom_mouse = false;
  }
  
  QGLViewer::mouseReleaseEvent(e);
}

