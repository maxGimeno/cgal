#include "Viewer.h"
#include <CGAL/gl.h>
#include <CGAL/Three/Scene_draw_interface.h>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QGLViewer/manipulatedCameraFrame.h>
#include <QDebug>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <cmath>
#include <QTime>

class Viewer_impl {
#include <QOpenGLContext>
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif
//just so it can compile.
#ifndef GL_VERTEX_PROGRAM_POINT_SIZE
#define GL_VERTEX_PROGRAM_POINT_SIZE 1
#endif
public:
  CGAL::Three::Scene_draw_interface* scene;
  bool antialiasing;
  bool twosides;
  bool macro_mode;
  bool inFastDrawing;
  
  void draw_aux(bool with_names, Viewer*);

  //! Contains all the programs for the item rendering.
  mutable std::vector<QOpenGLShaderProgram*> shader_programs;
};
Viewer::Viewer(QWidget* parent, bool antialiasing)
  : CGAL::Three::Viewer_interface(parent)
{
  d = new Viewer_impl;
  d->scene = 0;
  d->antialiasing = antialiasing;
  d->twosides = false;
  d->macro_mode = false;
  d->inFastDrawing = true;
  d->shader_programs.resize(NB_OF_PROGRAMS);
  shift_pressed = false;
  no_picking = false;
  setShortcut(EXIT_VIEWER, 0);
  setShortcut(DRAW_AXIS, 0);
  setKeyDescription(Qt::Key_T,
                    tr("Turn the camera by 180 degrees"));
  setKeyDescription(Qt::Key_M,
                    tr("Toggle macro mode: useful to view details very near from the camera, "
                       "but decrease the z-buffer precision"));
  setKeyDescription(Qt::Key_A,
                      tr("Toggle the axis system visibility."));
#if QGLVIEWER_VERSION >= 0x020501
  //modify mouse bindings that have been updated
  setMouseBinding(Qt::Key(0), Qt::NoModifier, Qt::LeftButton, RAP_FROM_PIXEL, true, Qt::RightButton);
  setMouseBindingDescription(Qt::ShiftModifier, Qt::RightButton,
                             tr("Select and pop context menu"));
  setMouseBinding(Qt::Key_R, Qt::NoModifier, Qt::LeftButton, RAP_FROM_PIXEL);
  //use the new API for these
  setMouseBinding(Qt::ShiftModifier, Qt::LeftButton, SELECT);
#else
  setMouseBinding(Qt::SHIFT + Qt::LeftButton, SELECT);
  setMouseBindingDescription(Qt::SHIFT + Qt::RightButton,
                             tr("Selects and display context "
                                "menu of the selected item"));
#endif // QGLVIEWER_VERSION >= 2.5.0
  for(int i=0; i<16; i++)
      pickMatrix_[i]=0;
  pickMatrix_[0]=1;
  pickMatrix_[5]=1;
  pickMatrix_[10]=1;
  pickMatrix_[15]=1;
  prev_radius = sceneRadius();
  axis_are_displayed = true;
}

Viewer::~Viewer()
{
  delete d;
}

void Viewer::setScene(CGAL::Three::Scene_draw_interface* scene)
{
  d->scene = scene;
}

bool Viewer::antiAliasing() const
{
  return d->antialiasing;
}

void Viewer::setAntiAliasing(bool b)
{
  d->antialiasing = b;
  update();

}

void Viewer::setTwoSides(bool b)
{
  d->twosides = b;
  update();

}


void Viewer::setFastDrawing(bool b)
{
  d->inFastDrawing = b;
  update();
}

bool Viewer::inFastDrawing() const
{
  return (d->inFastDrawing
          && (camera()->frame()->isSpinning()
              || camera()->frame()->isManipulated()));
}

void Viewer::draw()
{
  glEnable(GL_DEPTH_TEST);
  d->draw_aux(false, this);
}

void Viewer::fastDraw()
{
  d->draw_aux(false, this);
}

void Viewer::initializeGL()
{
  QGLViewer::initializeGL();
  initializeOpenGLFunctions();
#if !ANDROID
  glDrawArraysInstanced = (PFNGLDRAWARRAYSINSTANCEDARBPROC)this->context()->getProcAddress("glDrawArraysInstancedARB");
  if(!glDrawArraysInstanced)
  {
      qDebug()<<"glDrawArraysInstancedARB : extension not found. Spheres will be displayed as points.";
      extension_is_found = false;
  }
  else
      extension_is_found = true;

  glVertexAttribDivisor = (PFNGLVERTEXATTRIBDIVISORARBPROC)this->context()->getProcAddress("glVertexAttribDivisorARB");
  if(!glDrawArraysInstanced)
  {
      qDebug()<<"glVertexAttribDivisorARB : extension not found. Spheres will be displayed as points.";
      extension_is_found = false;
  }
  else
      extension_is_found = true;
#endif

  setBackgroundColor(::Qt::white);
  vao[0].create();
  for(int i=0; i<3; i++)
    buffers[i].create();
  d->scene->initializeGL();

  //Vertex source code
  const char vertex_source[] =
  {
      "attribute highp vec4 vertex;\n"
      "attribute highp vec3 normal;\n"
      "attribute highp vec4 colors;\n"
      "uniform highp mat4 mvp_matrix;\n"
      "uniform highp mat4 ortho_mat;\n"
      "uniform highp mat4 mv_matrix; \n"
      "uniform highp float width; \n"
      "uniform highp float height; \n"
      "varying highp vec4 fP; \n"
      "varying highp vec3 fN; \n"
      "varying highp vec4 color; \n"
      "void main(void)\n"
      "{\n"
      "   color = colors; \n"
      "   fP = mv_matrix * vertex; \n"
      "   fN = mat3(mv_matrix)* normal; \n"
      "   highp vec4 temp = highp vec4(mvp_matrix * vertex); \n"
      "   highp vec4 ort = ortho_mat * highp vec4(width-150.0, height-150.0, 0,0); \n"
      "   highp float ratio = width/height; \n"
      "   gl_Position =  ort +highp vec4(temp.x, temp.y, temp.z, 1.0); \n"
      "} \n"
      "\n"
  };
  //Fragment source code
  const char fragment_source[] =
  {
      "varying highp vec4 color; \n"
      "varying highp vec4 fP; \n"
      "varying highp vec3 fN; \n"
      "uniform highp vec4 light_pos;  \n"
      "uniform highp vec4 light_diff; \n"
      "uniform highp vec4 light_spec; \n"
      "uniform highp vec4 light_amb;  \n"
      "uniform highp float spec_power ; \n"

      "void main(void) { \n"
      "  highp vec3 L = light_pos.xyz - fP.xyz; \n"
      "  highp vec3 V = -fP.xyz; \n"
      "  highp vec3 N; \n"
      "   if(fN == highp vec3(0.0,0.0,0.0)) \n"
      "       N = highp vec3(0.0,0.0,0.0); \n"
      "   else \n"
      "       N = normalize(fN); \n"
      "   L = normalize(L); \n"
      "   V = normalize(V); \n"
      "   highp vec3 R = reflect(-L, N); \n"
      "   highp vec4 diffuse = max(abs(dot(N,L)),0.0) * light_diff*color; \n"
      "   highp vec4 specular = pow(abs(dot(R,V)), spec_power) * light_spec; \n"
      "gl_FragColor = color*light_amb + diffuse + specular; \n"
      "} \n"
      "\n"
  };
  QOpenGLShader *vertex_shader = new QOpenGLShader(QOpenGLShader::Vertex);
  if(!vertex_shader->compileSourceCode(vertex_source))
  {
      std::cerr<<"Compiling vertex source FAILED"<<std::endl;
  }

  QOpenGLShader *fragment_shader= new QOpenGLShader(QOpenGLShader::Fragment);
  if(!fragment_shader->compileSourceCode(fragment_source))
  {
      std::cerr<<"Compiling fragmentsource FAILED"<<std::endl;
  }

  if(!rendering_program.addShader(vertex_shader))
  {
      std::cerr<<"adding vertex shader FAILED"<<std::endl;
  }
  if(!rendering_program.addShader(fragment_shader))
  {
      std::cerr<<"adding fragment shader FAILED"<<std::endl;
  }
  rendering_program.bindAttributeLocation("vertex",0);
  if(!rendering_program.link())
  {
      //std::cerr<<"linking Program FAILED"<<std::endl;
      qDebug() << rendering_program.log();

  if(!context()->isOpenGLES())
  {
    gl->glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  }
}
}
#include <QMouseEvent>

void Viewer::mousePressEvent(QMouseEvent* event)
{
  if(event->button() == Qt::RightButton &&
     event->modifiers().testFlag(Qt::ShiftModifier))
  {
    select(event->pos());
    requestContextMenu(event->globalPos());
    event->accept();
  }
  else {
      if(frame_manipulation)
      {
          setMouseBinding(Qt::Key(0),Qt::NoModifier, Qt::LeftButton, FRAME, ROTATE);
      }
      else
      {
          setMouseBinding(Qt::NoModifier, Qt::LeftButton, CAMERA, ROTATE);

      }
          QGLViewer::mousePressEvent(event);
  }
}

void Viewer::keyPressEvent(QKeyEvent* e)
{
  if(!e->modifiers()) {
    if(e->key() == Qt::Key_T) {
      turnCameraBy180Degres();
      return;
    }
    else if(e->key() == Qt::Key_M) {
      d->macro_mode = ! d->macro_mode;

      if(d->macro_mode) {
          camera()->setZNearCoefficient(0.0005f);
      } else {
        camera()->setZNearCoefficient(0.005f);
      }
      this->displayMessage(tr("Macro mode: %1").
                           arg(d->macro_mode ? tr("on") : tr("off")));



      return;
    }
    else if(e->key() == Qt::Key_A) {
          axis_are_displayed = !axis_are_displayed;
#if !ANDROID
          this->updateGL();
#else
          update();
#endif
        }
  }
  //forward the event to the scene (item handling of the event)
  if (! d->scene->keyPressEvent(e) )
    QGLViewer::keyPressEvent(e);
}

void Viewer::turnCameraBy180Degres() {
  qglviewer::Camera* camera = this->camera();
  using qglviewer::ManipulatedCameraFrame;

  ManipulatedCameraFrame frame_from(*camera->frame());
  camera->setViewDirection(-camera->viewDirection());
  ManipulatedCameraFrame frame_to(*camera->frame());

  camera->setOrientation(frame_from.orientation());
  camera->interpolateTo(frame_to, 0.5f);
}

void Viewer_impl::draw_aux(bool with_names, Viewer* viewer)
{
  if(scene == 0)
    return;
#if !ANDROID
  viewer->glLineWidth(1.0f);
  viewer->glPointSize(2.f);
  viewer->glEnable(GL_POLYGON_OFFSET_FILL);
  viewer->glPolygonOffset(1.0f,1.0f);
  viewer->glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

  viewer->glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  if(twosides)
    viewer->glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  else
    viewer->glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

  if(!viewer->context()->isOpenGLES())
  {
      if(antialiasing)
      {
          glEnable(GL_MULTISAMPLE);
      }
      else
      {
          glDisable(GL_MULTISAMPLE);
      }
  }
  else
  {
    viewer->glDisable(GL_BLEND);
    viewer->glDisable(GL_LINE_SMOOTH);
    viewer->glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
    viewer->glBlendFunc(GL_ONE, GL_ZERO);
  }
#endif
  if(with_names && !viewer->no_picking)
    scene->drawWithNames(viewer);
  else
    scene->draw(viewer);
  #if !ANDROID
  viewer->glDisable(GL_POLYGON_OFFSET_FILL);
  viewer->glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
#endif
}

void Viewer::drawWithNames(const QPoint &point)
{
  QGLViewer::draw();
  d->scene->picking_target = point;
  d->draw_aux(true, this);
}

void Viewer::postSelection(const QPoint& pixel)
{/*
    //Avoids a segfault. I don't know where the segfault comes from but it only
    //hapens in a situation that should not exist, so this should do the trick.
#if ANDROID
    if(selection_mode)
    {
#endif*/
  bool found = false;
  std::vector<QOpenGLShaderProgram*> list;
  for(int i=0; i<NB_OF_PROGRAMS; i++)
  {
    QOpenGLShaderProgram *program = d->shader_programs[i];
    if(program)
      list.push_back(program);
  }
  qglviewer::Vec point = pointUnderPixelGLES(list,camera(),pixel, found);
  if(found) {
    Q_EMIT selectedPoint(point.x,
                       point.y,
                       point.z);
    Q_EMIT selected(this->selectedName());
    const qglviewer::Vec orig = camera()->position();
    const qglviewer::Vec dir = point - orig;
    Q_EMIT selectionRay(orig.x, orig.y, orig.z,
                      dir.x, dir.y, dir.z);
  }

/*
#if ANDROID
    }
#endif
*/
}
bool CGAL::Three::Viewer_interface::readFrame(QString s, qglviewer::Frame& frame)
{
  QStringList list = s.split(" ", QString::SkipEmptyParts);
  if(list.size() != 7)
    return false;
  float vec[3];
  for(int i = 0; i < 3; ++i)
  {
    bool ok;
    vec[i] = list[i].toFloat(&ok);
    if(!ok) return false;
  }
  float orient[4];
  for(int i = 0; i < 4; ++i)
  {
    bool ok;
    orient[i] = list[i + 3].toFloat(&ok);
    if(!ok) return false;
  }
  frame.setPosition(qglviewer::Vec(vec[0],
                                   vec[1],
                                   vec[2]));
  frame.setOrientation(orient[0],
                       orient[1],
                       orient[2],
                       orient[3]);
  return true;
}

QString CGAL::Three::Viewer_interface::dumpFrame(const qglviewer::Frame& frame) {
  const qglviewer::Vec pos = frame.position();
  const qglviewer::Quaternion q = frame.orientation();

  return QString("%1 %2 %3 %4 %5 %6 %7")
    .arg(pos[0])
    .arg(pos[1])
    .arg(pos[2])
    .arg(q[0])
    .arg(q[1])
    .arg(q[2])
    .arg(q[3]);
}

bool Viewer::moveCameraToCoordinates(QString s, float animation_duration) {
  qglviewer::Frame new_frame;
  if(readFrame(s, new_frame)) {
    camera()->interpolateTo(new_frame, animation_duration);
    return true;
  }
  else
    return false;
}

QString Viewer::dumpCameraCoordinates()
{
  if(camera()->frame()) {
    return dumpFrame(*camera()->frame());
  } else {
    return QString();
  }
}

void Viewer::attrib_buffers(int program_name) const {
    GLint is_both_sides = 0;
    //ModelViewMatrix used for the transformation of the camera.
    QMatrix4x4 mvp_mat;
    // ModelView Matrix used for the lighting system
    QMatrix4x4 mv_mat;
    // transformation of the manipulated frame
    QMatrix4x4 f_mat;
    // used for the picking. Is Identity except while selecting an item.
    QMatrix4x4 pick_mat;
    f_mat.setToIdentity();
    //fills the MVP and MV matrices.
    GLfloat d_mat[16];
    this->camera()->getModelViewProjectionMatrix(d_mat);
    //Convert the GLdoubles matrices in GLfloats
    for (int i=0; i<16; ++i){
        mvp_mat.data()[i] = GLfloat(d_mat[i]);
    }
    this->camera()->getModelViewMatrix(d_mat);
    for (int i=0; i<16; ++i)
        mv_mat.data()[i] = GLfloat(d_mat[i]);
    for (int i=0; i<16; ++i)
        pick_mat.data()[i] = this->pickMatrix_[i];

    mvp_mat = pick_mat * mvp_mat;

    QVector4D position(0.0f,0.0f,1.0f, 1.0f );
    QVector4D ambient(0.4f, 0.4f, 0.4f, 0.4f);
    // Diffuse
    QVector4D diffuse(1.0f, 1.0f, 1.0f, 1.0f);
    // Specular
    QVector4D specular(0.0f, 0.0f, 0.0f, 1.0f);
    QOpenGLShaderProgram* program = getShaderProgram(program_name);
    program->bind();
    switch(program_name)
    {
    case PROGRAM_NO_SELECTION:
        program->setUniformValue("mvp_matrix", mvp_mat);

        program->setUniformValue("f_matrix",f_mat);
        break;
    case PROGRAM_WITH_LIGHT:
        program->setUniformValue("mvp_matrix", mvp_mat);
        program->setUniformValue("mv_matrix", mv_mat);
        program->setUniformValue("f_matrix",f_mat);
        program->setUniformValue("light_pos", position);
        program->setUniformValue("light_diff",diffuse);
        program->setUniformValue("light_spec", specular);
        program->setUniformValue("light_amb", ambient);
        program->setUniformValue("spec_power", 51.8f);
        program->setUniformValue("is_two_side", is_both_sides);
        break;
    case PROGRAM_C3T3:
        program->setUniformValue("mvp_matrix", mvp_mat);
        program->setUniformValue("mv_matrix", mv_mat);
        program->setUniformValue("light_pos", position);
        program->setUniformValue("light_diff",diffuse);
        program->setUniformValue("light_spec", specular);
        program->setUniformValue("light_amb", ambient);
        program->setUniformValue("spec_power", 51.8f);
        program->setUniformValue("is_two_side", is_both_sides);
        break;
    case PROGRAM_C3T3_EDGES:
        program->setUniformValue("mvp_matrix", mvp_mat);
        break;
    case PROGRAM_WITHOUT_LIGHT:
        program->setUniformValue("mvp_matrix", mvp_mat);
        program->setUniformValue("mv_matrix", mv_mat);

        program->setUniformValue("light_pos", position);
        program->setUniformValue("light_diff", diffuse);
        program->setUniformValue("light_spec", specular);
        program->setUniformValue("light_amb", ambient);
        program->setUniformValue("spec_power", 51.8f);
        program->setUniformValue("is_two_side", is_both_sides);
        program->setAttributeValue("normals", 0.0,0.0,0.0);
        program->setUniformValue("f_matrix",f_mat);


        break;
    case PROGRAM_WITH_TEXTURE:

        program->setUniformValue("mvp_matrix", mvp_mat);
        program->setUniformValue("mv_matrix", mv_mat);
        program->setUniformValue("light_pos", position);
        program->setUniformValue("light_diff",diffuse);
        program->setUniformValue("light_spec", specular);
        program->setUniformValue("light_amb", ambient);
        program->setUniformValue("spec_power", 51.8f);
        program->setUniformValue("s_texture",0);
        program->setUniformValue("f_matrix",f_mat);

        break;
    case PROGRAM_PLANE_TWO_FACES:
        program->setUniformValue("mvp_matrix", mvp_mat);
        program->setUniformValue("mv_matrix", mv_mat);
        program->setUniformValue("light_pos", position);
        program->setUniformValue("light_diff",diffuse);
        program->setUniformValue("light_spec", specular);
        program->setUniformValue("light_amb", ambient);
        program->setUniformValue("spec_power", 51.8f);
        program->setUniformValue("is_two_side", is_both_sides);
        break;

    case PROGRAM_WITH_TEXTURED_EDGES:

        program->setUniformValue("mvp_matrix", mvp_mat);
        program->setUniformValue("s_texture",0);

        break;
    case PROGRAM_INSTANCED:

        program->setUniformValue("mvp_matrix", mvp_mat);
        program->setUniformValue("mv_matrix", mv_mat);

        program->setUniformValue("light_pos", position);
        program->setUniformValue("light_diff",diffuse);
        program->setUniformValue("light_spec", specular);
        program->setUniformValue("light_amb", ambient);
        program->setUniformValue("spec_power", 51.8f);
        program->setUniformValue("is_two_side", is_both_sides);

        break;
    case PROGRAM_INSTANCED_WIRE:
        program->setUniformValue("mvp_matrix", mvp_mat);
        break;
    }
    program->release();
}


void Viewer::pickMatrix(GLfloat x, GLfloat y, GLfloat width, GLfloat height,
GLint viewport[4])
{
 //GLfloat m[16];
 GLfloat sx, sy;
 GLfloat tx, ty;

 sx = viewport[2] / width;
 sy = viewport[3] / height;
 tx = (viewport[2] + 2.0 * (viewport[0] - x)) / width;
 ty = (viewport[3] + 2.0 * (viewport[1] - y)) / height;

 #define M(row, col) pickMatrix_[col*4+row]
  M(0, 0) = sx;
  M(0, 1) = 0.0;
  M(0, 2) = 0.0;
  M(0, 3) = tx;
  M(1, 0) = 0.0;
  M(1, 1) = sy;
  M(1, 2) = 0.0;
  M(1, 3) = ty;
  M(2, 0) = 0.0;
  M(2, 1) = 0.0;
  M(2, 2) = 1.0;
  M(2, 3) = 0.0;
  M(3, 0) = 0.0;
  M(3, 1) = 0.0;
  M(3, 2) = 0.0;
  M(3, 3) = 1.0;
 #undef M

 //pickMatrix_[i] = m[i];
}
void Viewer::beginSelection(const QPoint &point)
{
#if !ANDROID
    QGLViewer::beginSelection(point);
    //set the picking matrix to allow the picking
    static GLint viewport[4];
    camera()->getViewport(viewport);
    pickMatrix(point.x(), point.y(), selectRegionWidth(), selectRegionHeight(), viewport);
#endif
}
void Viewer::endSelection(const QPoint& point)
{
#if !ANDROID
  QGLViewer::endSelection(point);
   //set the pick matrix to Identity
    for(int i=0; i<16; i++)
        pickMatrix_[i]=0;
    pickMatrix_[0]=1;
    pickMatrix_[5]=1;
    pickMatrix_[10]=1;
    pickMatrix_[15]=1;
#endif
}

void Viewer::makeArrow(float R, int prec, qglviewer::Vec from, qglviewer::Vec to, qglviewer::Vec color, AxisData &data)
{
    qglviewer::Vec temp = to-from;
    QVector3D dir = QVector3D(temp.x, temp.y, temp.z);
    QMatrix4x4 mat;
    mat.setToIdentity();
    mat.translate(from.x, from.y, from.z);
    mat.scale(dir.length());
    dir.normalize();
    float angle = 0.0;
    if(std::sqrt((dir.x()*dir.x()+dir.y()*dir.y())) > 1)
        angle = 90.0f;
    else
        angle =acos(dir.y()/std::sqrt(dir.x()*dir.x()+dir.y()*dir.y()+dir.z()*dir.z()))*180.0/M_PI;

    QVector3D axis;
    axis = QVector3D(dir.z(), 0, -dir.x());
    mat.rotate(angle, axis);

    //Head
    const float Rf = static_cast<float>(R);
    for(int d = 0; d<360; d+= 360/prec)
    {
        float D = (float) (d * M_PI / 180.);
        float a = (float) std::atan(Rf / 0.33);
        QVector4D p(0., 1., 0, 1.);
        QVector4D n(Rf*2.*sin(D), sin(a), Rf*2.*cos(D), 1.);
        QVector4D pR = mat*p;
        QVector4D nR = mat*n;

        //point A1
        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back((float)color.x);
        data.colors->push_back((float)color.y);
        data.colors->push_back((float)color.z);

        //point B1
        p = QVector4D(Rf*2.*sin(D), 0.66f, Rf*2.* cos(D), 1.f);
        n = QVector4D(sin(D), sin(a), cos(D), 1.);
        pR = mat*p;
        nR = mat*n;
        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back((float)color.x);
        data.colors->push_back((float)color.y);
        data.colors->push_back((float)color.z);
        //point C1
        D = (d+360/prec)*M_PI/180.0;
        p = QVector4D(Rf*2.* sin(D), 0.66f, Rf *2.* cos(D), 1.f);
        n = QVector4D(sin(D), sin(a), cos(D), 1.0);
        pR = mat*p;
        nR = mat*n;

        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back((float)color.x);
        data.colors->push_back((float)color.y);
        data.colors->push_back((float)color.z);

    }

    //cylinder
    //body of the cylinder
    for(int d = 0; d<360; d+= 360/prec)
    {
        //point A1
        float D = d*M_PI/180.0;
        QVector4D p(Rf*sin(D), 0.66f, Rf*cos(D), 1.f);
        QVector4D n(sin(D), 0.f, cos(D), 1.f);
        QVector4D pR = mat*p;
        QVector4D nR = mat*n;

        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back(color.x);
        data.colors->push_back(color.y);
        data.colors->push_back(color.z);
        //point B1
        p = QVector4D(Rf * sin(D),0,Rf*cos(D), 1.0);
        n = QVector4D(sin(D), 0, cos(D), 1.0);
        pR = mat*p;
        nR = mat*n;


        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back(color.x);
        data.colors->push_back(color.y);
        data.colors->push_back(color.z);
          //point C1
        D = (d+360/prec)*M_PI/180.0;
        p = QVector4D(Rf * sin(D),0,Rf*cos(D), 1.0);
        n = QVector4D(sin(D), 0, cos(D), 1.0);
        pR = mat*p;
        nR = mat*n;
        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back(color.x);
        data.colors->push_back(color.y);
        data.colors->push_back(color.z);
        //point A2
        D = (d+360/prec)*M_PI/180.0;

        p = QVector4D(Rf * sin(D),0,Rf*cos(D), 1.0);
        n = QVector4D(sin(D), 0, cos(D), 1.0);
        pR = mat*p;
        nR = mat*n;
        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back((float)color.x);
        data.colors->push_back((float)color.y);
        data.colors->push_back((float)color.z);
        //point B2
        p = QVector4D(Rf * sin(D), 0.66f, Rf*cos(D), 1.f);
        n = QVector4D(sin(D), 0, cos(D), 1.0);
        pR = mat*p;
        nR = mat*n;
        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back((float)color.x);
        data.colors->push_back((float)color.y);
        data.colors->push_back((float)color.z);
        //point C2
        D = d*M_PI/180.0;
        p = QVector4D(Rf * sin(D), 0.66f, Rf*cos(D), 1.f);
        n = QVector4D(sin(D), 0.f, cos(D), 1.f);
        pR = mat*p;
        nR = mat*n;
        data.vertices->push_back(pR.x());
        data.vertices->push_back(pR.y());
        data.vertices->push_back(pR.z());
        data.normals->push_back(nR.x());
        data.normals->push_back(nR.y());
        data.normals->push_back(nR.z());
        data.colors->push_back(color.x);
        data.colors->push_back(color.y);
        data.colors->push_back(color.z);

    }
}

void Viewer::drawVisualHintsGLES()
{
    QGLViewer::drawVisualHintsGLES();
    if(axis_are_displayed)
    {
        QMatrix4x4 mvpMatrix;
        QMatrix4x4 mvMatrix;
        float mat[16];
        //Keeps the axis from being clipped
        camera()->getModelViewProjectionMatrix(mat);
        //nullifies the translation
        mat[12]=0;
        mat[13]=0;
        mat[14]=0;
        for(int i=0; i < 16; i++)
        {
            mvpMatrix.data()[i] = (float)mat[i];
        }
        camera()->getModelViewMatrix(mat);
        for(int i=0; i < 16; i++)
        {
            mvMatrix.data()[i] = (float)mat[i];
        }
        //Keeps the lighing from changing according to the position and orientation of the camera.
        mvMatrix.data()[12] = 0;
        mvMatrix.data()[13] = 0;
        mvMatrix.data()[14] = 1;
        QVector4D	position(0.0f,0.0f,0.0f,1.0f );
        // define material
        QVector4D	ambient;
        QVector4D	diffuse;
        QVector4D	specular;
        GLfloat      shininess ;
        // Ambient
        ambient[0] = 0.29225f;
        ambient[1] = 0.29225f;
        ambient[2] = 0.29225f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.50754f;
        diffuse[1] = 0.50754f;
        diffuse[2] = 0.50754f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.0f;
        specular[1] = 0.0f;
        specular[2] = 0.0f;
        specular[3] = 0.0f;
        // Shininess
        shininess = 51.2f;
        rendering_program.bind();
        rendering_program.setUniformValue("light_pos", position);
        rendering_program.setUniformValue("mvp_matrix", mvpMatrix);
        rendering_program.setUniformValue("mv_matrix", mvMatrix);
        rendering_program.setUniformValue("light_diff", diffuse);
        rendering_program.setUniformValue("light_spec", specular);
        rendering_program.setUniformValue("light_amb", ambient);
        rendering_program.setUniformValue("spec_power", shininess);
        vao[0].bind();
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(v_Axis.size() / 3));
        rendering_program.release();
        vao[0].release();
    }

}

void Viewer::resizeGL(int w, int h)
{
    QGLViewer::resizeGL(w,h);
    qglviewer::Vec dim = qglviewer::Vec(w,h, 0) ;
    GLfloat ortho[16];
    QMatrix4x4 orthoMatrix;
    ortho[0]  = 1.0/width(); ortho[1]  = 0; ortho[2]  = 0; ortho[3]  = -0.0;
    ortho[4]  = 0; ortho[5]  = 1.0/height(); ortho[6]  = 0; ortho[7]  = -0.0;
    ortho[8]  = 0; ortho[9]  = 0; ortho[10] = 2.0/(camera()->zNear()-camera()->zFar()); ortho[11] = -(camera()->zNear()+camera()->zFar())/(-camera()->zNear()+camera()->zFar());
    ortho[12] = 0; ortho[13] = 0; ortho[14] = 0; ortho[15] = 1;
    for(int i=0; i < 16; i++)
    {
        orthoMatrix.data()[i] = (float)ortho[i];
    }

    QVector4D length(60,60,60, 1.0);
    length = orthoMatrix * length;
    AxisData data;
    v_Axis.resize(0);
    n_Axis.resize(0);
    c_Axis.resize(0);
    data.vertices = &v_Axis;
    data.normals = &n_Axis;
    data.colors = &c_Axis;
    float l = length.x()*w/h;
    makeArrow(0.06,10, qglviewer::Vec(0,0,0),qglviewer::Vec(l,0,0),qglviewer::Vec(1,0,0), data);
    makeArrow(0.06,10, qglviewer::Vec(0,0,0),qglviewer::Vec(0,l,0),qglviewer::Vec(0,1,0), data);
    makeArrow(0.06,10, qglviewer::Vec(0,0,0),qglviewer::Vec(0,0,l),qglviewer::Vec(0,0,1), data);


    vao[0].bind();
    buffers[0].bind();
    buffers[0].allocate(v_Axis.data(), static_cast<int>(v_Axis.size()) * sizeof(float));
    rendering_program.enableAttributeArray("vertex");
    rendering_program.setAttributeBuffer("vertex",GL_FLOAT,0,3);
    buffers[0].release();

    buffers[1].bind();
    buffers[1].allocate(n_Axis.data(), static_cast<int>(n_Axis.size() * sizeof(float)));
    rendering_program.enableAttributeArray("normal");
    rendering_program.setAttributeBuffer("normal",GL_FLOAT,0,3);
    buffers[1].release();

    buffers[2].bind();
    buffers[2].allocate(c_Axis.data(), static_cast<int>(c_Axis.size() * sizeof(float)));
    rendering_program.enableAttributeArray("colors");
    rendering_program.setAttributeBuffer("colors",GL_FLOAT,0,3);
    buffers[2].release();

    rendering_program.release();
    vao[0].release();



    rendering_program.bind();
    rendering_program.setUniformValue("width", (float)dim.x);
    rendering_program.setUniformValue("height", (float)dim.y);
    rendering_program.setUniformValue("ortho_mat", orthoMatrix);
    rendering_program.release();
}
QPointF posipoint;
bool Viewer::event(QEvent *e)
{
    if(e->type() == QEvent::TouchBegin)
    {
      chrono.start();
      posipoint = static_cast<QTouchEvent*>(e)->touchPoints().first().pos();
      QGLViewer::event(e);
      return true;
    }
   else if(e->type() == QEvent::TouchEnd)
    {
     if( posipoint == static_cast<QTouchEvent*>(e)->touchPoints().first().pos() && chrono.elapsed() >=1000)
     {qDebug()<<"show menu"; requestContextMenu(static_cast<QTouchEvent*>(e)->touchPoints().first().pos().toPoint()); }
     else{qDebug()<<posipoint<<" vs "<<static_cast<QTouchEvent*>(e)->touchPoints().first().pos().toPoint()<<", "<<chrono.elapsed();}
     QGLViewer::event(e);
     return true;
    }

    return  QGLViewer::event(e);
}
/*
qglviewer::Vec Viewer::pointUnderPixelGLES(std::vector<QOpenGLShaderProgram*> programs, qglviewer::Camera*const camera, const QPoint& pixel, bool& found)
{
    makeCurrent();

    static const int size = programs.size();

    std::vector<datas> original_shaders;
    //The fragmentertex source code
    const char grayscale_fragment_source[] =
    {
        "void main(void) { \n"
        "gl_FragColor = vec4(vec3(gl_FragCoord.z), 1.0); \n"
        "} \n"
        "\n"
    };

    for(int i=0; i<size; i++)
    {
        for(int j=0; j<programs[i]->shaders().size(); j++)
        {
            if(programs[i]->shaders().at(j)->shaderType() == QOpenGLShader::Fragment)
            {
                //copies the original shaders of each program
                datas c;
                c.code = programs[i]->shaders().at(j)->sourceCode();
                c.program_index = i;
                c.shader_index = j;
                original_shaders.push_back(c);
                //replace their fragment shaders so they display in a grayscale
                programs[i]->shaders().at(j)->compileSourceCode(grayscale_fragment_source);
            }
            programs[i]->link();
        }
}
    //determines the size of the buffer
    int deviceWidth = camera->screenWidth();
    int deviceHeight = camera->screenHeight();
    int rowLength = deviceWidth * 4; // data asked in RGBA,so 4 bytes.
    //the FBO in which the grayscale image will be rendered
    QOpenGLFramebufferObject *fbo = new QOpenGLFramebufferObject(deviceWidth, deviceHeight);
    fbo->bind();
    //make the lines thicker so it is easier to click
    gl->glLineWidth(10.0);
    //draws the image in the fbo
    paintGL();
    gl->glLineWidth(1.0);
    const static int dataLength = rowLength * deviceHeight;
    GLubyte* buffer = new GLubyte[dataLength];
    // Qt uses upper corner for its origin while GL uses the lower corner.
    gl->glReadPixels(pixel.x(), deviceHeight-1-pixel.y(), 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
    //reset the fbo to the one rendered on-screen, now that we have our information
    fbo->release();
    delete fbo;
    //resets the originals programs
    for(int i=0; i<(int)original_shaders.size(); i++)
    {
        programs[original_shaders[i].program_index]->shaders().at(original_shaders[i].shader_index)->compileSourceCode(original_shaders[i].code);
        programs[original_shaders[i].program_index]->link();
    }
    //depth value needs to be between 0 and 1.
    float depth = buffer[0]/255.0;
    delete buffer;
    qglviewer::Vec point(pixel.x(), pixel.y(), depth);
    point = camera->unprojectedCoordinatesOf(point);
    //if depth is 1, then it is the zFar plane that is hit, so there is nothing rendered along the ray.
     found = depth<1;
     //qDebug()<<"pointUnderPixel";
     return point;
}*/

QOpenGLShaderProgram* Viewer::getShaderProgram(int name) const
{
    // workaround constness issues in Qt
    Viewer* viewer = const_cast<Viewer*>(this);

    switch(name)
    {
    /// @TODO: factorize this code   
    case PROGRAM_C3T3:
        if(d->shader_programs[PROGRAM_C3T3])
        {
            return d->shader_programs[PROGRAM_C3T3];
        }

        else
        {

            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_c3t3.v"))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_c3t3.f"))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_C3T3] = program;
            return program;
        }
        break;
    case PROGRAM_C3T3_EDGES:
        if(d->shader_programs[PROGRAM_C3T3_EDGES])
        {
            return d->shader_programs[PROGRAM_C3T3_EDGES];
        }

        else
        {

            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_c3t3_edges.v"))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_c3t3_edges.f"))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_C3T3_EDGES] = program;
            return program;
        }
        break;
    case PROGRAM_WITH_LIGHT:
        if(d->shader_programs[PROGRAM_WITH_LIGHT])
        {
            return d->shader_programs[PROGRAM_WITH_LIGHT];
        }

        else
        {

            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_with_light.v"))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_with_light.f"))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_WITH_LIGHT] = program;
            return program;
        }
        break;
    case PROGRAM_WITHOUT_LIGHT:
        if( d->shader_programs[PROGRAM_WITHOUT_LIGHT])
        {
            return d->shader_programs[PROGRAM_WITHOUT_LIGHT];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_without_light.v"))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_without_light.f"))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_WITHOUT_LIGHT] = program;
            return program;
        }
        break;
    case PROGRAM_NO_SELECTION:
        if( d->shader_programs[PROGRAM_NO_SELECTION])
        {
            return d->shader_programs[PROGRAM_NO_SELECTION];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_without_light.v"))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_no_light_no_selection.f"))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_NO_SELECTION] = program;
            return program;
        }
        break;
    case PROGRAM_WITH_TEXTURE:
        if( d->shader_programs[PROGRAM_WITH_TEXTURE])
        {
            return d->shader_programs[PROGRAM_WITH_TEXTURE];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_with_texture.v"))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_with_texture.f"))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_WITH_TEXTURE] = program;
            return program;
        }
        break;
    case PROGRAM_PLANE_TWO_FACES:
        if(d->shader_programs[PROGRAM_PLANE_TWO_FACES])
        {
            return d->shader_programs[PROGRAM_PLANE_TWO_FACES];
        }

        else
        {

            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_without_light.v"))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_plane_two_faces.f"))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_PLANE_TWO_FACES] = program;
            return program;
        }
        break;

    case PROGRAM_WITH_TEXTURED_EDGES:
        if( d->shader_programs[PROGRAM_WITH_TEXTURED_EDGES])
        {
            return d->shader_programs[PROGRAM_WITH_TEXTURED_EDGES];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_with_textured_edges.v" ))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_with_textured_edges.f" ))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_WITH_TEXTURED_EDGES] = program;
            return program;

        }
        break;
    case PROGRAM_INSTANCED:
        if( d->shader_programs[PROGRAM_INSTANCED])
        {
            return d->shader_programs[PROGRAM_INSTANCED];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_instanced.v" ))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_with_light.f" ))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_INSTANCED] = program;
            return program;

        }
        break;
    case PROGRAM_INSTANCED_WIRE:
        if( d->shader_programs[PROGRAM_INSTANCED_WIRE])
        {
            return d->shader_programs[PROGRAM_INSTANCED_WIRE];
        }
        else
        {
            QOpenGLShaderProgram *program = new QOpenGLShaderProgram(viewer);
            if(!program->addShaderFromSourceFile(QOpenGLShader::Vertex,":/cgal/Polyhedron_3/resources/shader_instanced.v" ))
            {
                std::cerr<<"adding vertex shader FAILED"<<std::endl;
            }
            if(!program->addShaderFromSourceFile(QOpenGLShader::Fragment,":/cgal/Polyhedron_3/resources/shader_without_light.f" ))
            {
                std::cerr<<"adding fragment shader FAILED"<<std::endl;
            }
            program->bindAttributeLocation("vertex",0);
            program->link();
            d->shader_programs[PROGRAM_INSTANCED_WIRE] = program;
            return program;

        }
        break;
    default:
        std::cerr<<"ERROR : Program not found."<<std::endl;
        return 0;
    }
}
void Viewer::wheelEvent(QWheelEvent* e)
{
    if(e->modifiers().testFlag(Qt::ShiftModifier))
    {
        float delta = e->delta();
        if(delta>0)
        {
            camera()->setZNearCoefficient(camera()->zNearCoefficient() * 1.01);
        }
        else
            camera()->setZNearCoefficient(camera()->zNearCoefficient() / 1.01);
        update();
    }
    else
        QGLViewer::wheelEvent(e);
}
std::vector<QOpenGLShaderProgram*> Viewer::getPrograms()
{
  return d->shader_programs;
}
