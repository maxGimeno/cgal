#ifndef VIEWER_INTERFACE_H
#define VIEWER_INTERFACE_H

#include <QGLViewer/qglviewer.h>
#include <QWidget>
#include <QPoint>
#include <QOpenGLFunctions>
#include <CGAL/Qt/CreateOpenGLContext.h>
#include <QOpenGLShaderProgram>
// forward declarations
class QWidget;
class Scene_draw_interface;
class QMouseEvent;
class QKeyEvent;

#include "../Viewer_config.h" // for VIEWER_EXPORT

class VIEWER_EXPORT Viewer_interface : public QGLViewer, public QOpenGLFunctions{

  Q_OBJECT

public:

  mutable int is_two_sides;
  mutable std::vector<QOpenGLShaderProgram*> program_list;
  Viewer_interface(QWidget* parent) : QGLViewer(parent) {}
  virtual ~Viewer_interface() {}

  virtual void setScene(Scene_draw_interface* scene) = 0;
  virtual bool antiAliasing() const = 0;

  // Those two functions are defined in Viewer.cpp
  static bool readFrame(QString, qglviewer::Frame&);
  static QString dumpFrame(const qglviewer::Frame&);

  virtual bool inFastDrawing() const = 0;
#if !ANDROID
  typedef void (APIENTRYP PFNGLDRAWARRAYSINSTANCEDARBPROC) (GLenum mode, GLint first, GLsizei count, GLsizei primcount);
  typedef void (APIENTRYP PFNGLVERTEXATTRIBDIVISORARBPROC) (GLuint index, GLuint divisor);
  typedef void (APIENTRYP PFNGLFRAMEBUFFERTEXTURE2DEXTPROC) (GLuint target, GLuint attachment, GLuint textarget, GLuint texture, GLint level);
  PFNGLDRAWARRAYSINSTANCEDARBPROC glDrawArraysInstanced;
  PFNGLVERTEXATTRIBDIVISORARBPROC glVertexAttribDivisor;
  PFNGLFRAMEBUFFERTEXTURE2DEXTPROC glFramebufferTexture2D;

#endif
  bool shift_pressed;
  bool extension_is_found;
  GLfloat pickMatrix_[16];
  //!Sets the binding for SHIFT+LEFT CLICK to SELECT (initially used in Scene_polyhedron_selection_item.h)
  void setBindingSelect()
  {
#if QGLVIEWER_VERSION >= 0x020501
    setMouseBinding(Qt::ShiftModifier, Qt::LeftButton, SELECT);
#else
    setMouseBinding(Qt::SHIFT + Qt::LeftButton, SELECT);
#endif

#if ANDROID
    selection_mode = true;
#endif

  }
  //!Sets the binding for SHIFT+LEFT CLICK to NO_CLICK_ACTION (initially used in Scene_polyhedron_selection_item.h)
  void setNoBinding()
  {
#if QGLVIEWER_VERSION >= 0x020501
    setMouseBinding(Qt::ShiftModifier, Qt::LeftButton, NO_CLICK_ACTION);
#else
    setMouseBinding(Qt::SHIFT + Qt::LeftButton, NO_CLICK_ACTION);
#endif
#if ANDROID
    selection_mode = false;
#endif
  }

Q_SIGNALS:
  void selected(int);
  void requestContextMenu(QPoint global_pos);
  void selectedPoint(double, double, double);
  void selectionRay(double, double, double, double, double, double);

public Q_SLOTS:
  virtual void setAntiAliasing(bool b) = 0;
  virtual void setTwoSides(bool b) = 0;

  virtual void turnCameraBy180Degres() = 0;

  virtual QString dumpCameraCoordinates() = 0;
  virtual bool moveCameraToCoordinates(QString, 
                                       float animation_duration = 0.5f) = 0;

}; // end class Viewer_interface

#endif // VIEWER_INTERFACE_H
