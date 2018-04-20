#undef QT_NO_KEYWORDS
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
        dtkLogger::instance().attachConsole();
        dtkLogger::instance().setLevel(dtkLog::Trace);
      initialized = false;

      dtkContinuousGeometryPrimitives::Point_3 p(0., 0., 0.);

      polyline.reserve(101);
      for(float f = 0.; f <= 1.; f += 0.01) {
          m_rational_bezier_curve.evaluatePoint(f, p.data());
          for(int i = 0; i < 3; ++i) {
              polyline.push_back(p[i]);
          }
      }
      dtkContinuousGeometryPrimitives::AABB_3 aabb(0., 0., 0., 0. ,0., 0.);
      m_rational_bezier_curve.aabb(aabb.data());
      bbox = CGAL::Bbox_3(aabb[0], aabb[1], aabb[2], aabb[3], aabb[4], aabb[5]);
    }

  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const
  {
    dtkDebug() << "Initializing buffer...";

    // ///////////////////////////////////////////////////////////////////
    // Creates VBO EBO and VAO
    // ///////////////////////////////////////////////////////////////////

    m_program = item->getShaderProgram(Scene_rational_bezier_curve_item::PROGRAM_NO_SELECTION, viewer);
    m_program->bind();
    item->vaos[POLYLINE]->bind();
    item->buffers[B_POLYLINE].bind();

    item->buffers[B_POLYLINE].allocate(polyline.data(), static_cast<int>(polyline.size() * sizeof(float)));
    m_program->enableAttributeArray("vertex");
    m_program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
    m_program->disableAttributeArray("colors");

    item->buffers[B_POLYLINE].release();
    item->vaos[POLYLINE]->release();
    m_program->release();

    initialized = true;

    dtkDebug() << "Buffers initialized...";
  }
  const dtkRationalBezierCurve& m_rational_bezier_curve;

      enum Vao
  {
    POLYLINE,
    NumberOfVaos
  };

  enum Buffer
  {
    B_POLYLINE,
    NumberOfBuffers
  };

    mutable QOpenGLShaderProgram* m_program;
    mutable std::vector<float> polyline;
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
    vaos[D::POLYLINE]->bind();
    d->m_program->setAttributeValue("colors", QColor(Qt::red));
    viewer->glDrawArrays(GL_LINE_STRIP, 0, d->polyline.size() / 3);
  _bbox = d->bbox;
}

bool Scene_rational_bezier_curve_item::isEmpty() const
{
  return d->polyline.empty();
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
