#include "Scene_rational_bezier_curve_item.h"
#define foreach Q_FOREACH

#include <dtkLog>

#include <dtkRationalBezierCurve>
#include <dtkContinuousGeometryUtils>
#include <algorithm>
#include <QMenu>

typedef Scene_rational_bezier_curve_item_priv D;
//Vertex source code
struct Scene_rational_bezier_curve_item_priv{

  Scene_rational_bezier_curve_item_priv(const dtkRationalBezierCurve& dtk_rational_bezier_curve,
                        Scene_rational_bezier_curve_item* parent)
    : m_rational_bezier_curve(dtk_rational_bezier_curve), item(parent)

  {
      initialized = false;
      dtkLogger::instance().attachConsole();
      dtkLogger::instance().setLevel(dtkLog::Error);

      dtkDebug() << "Generating intersection lines...";
      dtkContinuousGeometryPrimitives::Point_3 p3D(0,0,0);
        for(double i = 0.; i < 1.; i += 0.01) {
            curve->evaluatePoint(i,p.data());
            surface->evaluatePoint(p[0], p[1], p_3.data());
            file << p_3 << std::endl;
        }

      (*trim)->curve2D().evaluatePoint(knots[0], p.data());
      m_nurbs_surface.evaluatePoint(p[0], p[1], p3D.data());
      for(int j=0; j<3; ++j)
          intersection.push_back(p3D[j]);
      for(float f = knots[0]+1/100.0*(knots[length-1]-knots[0]);
                      f<knots[length-1] - 1/100.0*(knots[length-1]-knots[0]);
                      f+=1/100.0*(knots[length-1]-knots[0]))
                      {
                          (*trim)->curve2D().evaluatePoint(f, p.data());
                          m_nurbs_surface.evaluatePoint(p[0], p[1], p3D.data());
                          for(int j=0; j<3; ++j)
                              intersection.push_back(p3D[j]);
                          for(int j=0; j<3; ++j)
                              intersection.push_back(p3D[j]);
                      }
                  (*trim)->curve2D().evaluatePoint(knots[length-1], p.data());
                  m_nurbs_surface.evaluatePoint(p[0], p[1], p3D.data());
                  for(int j=0; j<3; ++j)
                      intersection.push_back(p3D[j]);
              }
      }
      dtkDebug() << "Intersection lines generated...";
  }

  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const
  {
    dtkDebug() << "Initializing buffer...";

    // ///////////////////////////////////////////////////////////////////
    // Creates VBO EBO and VAO
    // ///////////////////////////////////////////////////////////////////

    m_program = item->getShaderProgram(Scene_nurbs_item::PROGRAM_NO_SELECTION, viewer);
    m_program->bind();
    item->vaos[SEGMENTS]->bind();
    item->buffers[B_SEGMENTS].bind();

    item->buffers[B_SEGMENTS].allocate(intersection.data(), static_cast<int>(intersection.size() * sizeof(float)));
    m_program->enableAttributeArray("vertex");
    m_program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
    m_program->disableAttributeArray("colors");

    item->buffers[B_SEGMENTS].release();
    item->vaos[SEGMENTS]->release();
    m_program->release();

    initialized = true;

    dtkDebug() << "Buffers initialized...";
  }
  const dtkRationalBezierCurve& m_rational_bezier_curve;

      enum Vao
  {
    SEGMENTS,
    NumberOfVaos
  };

  enum Buffer
  {
    B_SEGMENTS,
    NumberOfBuffers
  };

    mutable QOpenGLShaderProgram* m_program;
    mutable std::vector<float> segments;
    mutable bool initialized;
    CGAL::Three::Scene_item::Bbox bbox;
    Scene_rational_bezier_curve_item* item;
};

Scene_rational_bezier_curve_item::Scene_rational_bezier_curve_item(const dtkRationalBezierCurve& dtk_rational_bezier_curve)
  :CGAL::Three::Scene_item(D::NumberOfBuffers, D::NumberOfVaos)
{
  d = new Scene_rational_bezier_curve_item_priv(dtk_rational_bezier_curve, this);
}

Scene_rational_bezier_curve_item::~Scene_rational_bezier_curve_item() { if(d) delete d; }

void Scene_rational_bezier_curve_item::draw(CGAL::Three::Viewer_interface* viewer)const
{
    if(!d->initialized)
        d->initializeBuffers(viewer);

    attribBuffers(viewer, PROGRAM_NO_SELECTION);
    d->m_program = getShaderProgram(PROGRAM_NO_SELECTION, viewer);
    d->m_program->bind();
    vaos[D::SEGMENTS]->bind();
    d->m_program->setAttributeValue("colors", QColor(Qt::red));
    viewer->glDrawArrays(GL_LINES, 0, d->segments.size()/3);
    vaos[D::SEGMENTS]->release();
    d->m_program->release();
}

void Scene_rational_bezier_curve_item::compute_bbox() const
{
  _bbox = d->bbox;
}

bool Scene_rational_bezier_curve_item::isEmpty() const
{
  return d->m_elements.empty();
}

QMenu* Scene_rational_bezier_curve_item::contextMenu()
{
  const char* prop_name = "Menu modified by Scene_rational_bezier_curve_item.";

  QMenu* menu = Scene_item::contextMenu();

  // Use dynamic properties:
  // http://doc.qt.io/qt-5/qobject.html#property
  bool menuChanged = menu->property(prop_name).toBool();

  if (!menuChanged) {
    menu->setProperty(prop_name, true);
  }
  return menu;
}
