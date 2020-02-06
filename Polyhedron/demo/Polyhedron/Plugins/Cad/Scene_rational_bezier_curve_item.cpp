#undef QT_NO_KEYWORDS
#include "Scene_rational_bezier_curve_item.h"
#define foreach Q_FOREACH

#include <CGAL/Three/Edge_container.h>

#include <dtkLog>

#include <dtkRationalBezierCurve>
#include <dtkContinuousGeometryUtils>
#include <algorithm>
#include <QMenu>

typedef Scene_rational_bezier_curve_item_priv D;
typedef CGAL::Three::Edge_container Ec;
typedef CGAL::Three::Viewer_interface Vi;
//Vertex source code
struct Scene_rational_bezier_curve_item_priv{

  Scene_rational_bezier_curve_item_priv(const dtkRationalBezierCurve& dtk_rational_bezier_curve,
                        Scene_rational_bezier_curve_item* parent)
    : m_rational_bezier_curve(dtk_rational_bezier_curve), item(parent)

    {
        dtkLogger::instance().attachConsole();
        dtkLogger::instance().setLevel(dtkLog::Trace);

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


  const dtkRationalBezierCurve& m_rational_bezier_curve;

    mutable QOpenGLShaderProgram* m_program;
    mutable std::vector<float> polyline;
    CGAL::Three::Scene_item::Bbox bbox;
    Scene_rational_bezier_curve_item* item;
};

Scene_rational_bezier_curve_item::Scene_rational_bezier_curve_item(const dtkRationalBezierCurve& dtk_rational_bezier_curve)
  :CGAL::Three::Scene_item_rendering_helper()
{
  setEdgeContainer(0, new Ec( Vi::PROGRAM_NO_SELECTION, false));

  d = new Scene_rational_bezier_curve_item_priv(dtk_rational_bezier_curve, this);
}

Scene_rational_bezier_curve_item::~Scene_rational_bezier_curve_item() { if(d) delete d; }

void Scene_rational_bezier_curve_item::draw(CGAL::Three::Viewer_interface* viewer)const
{
  if(!isInit(viewer))
    initGL(viewer);
  if ( getBuffersFilled() &&
       ! getBuffersInit(viewer))
  {
    initializeBuffers(viewer);
    setBuffersInit(viewer, true);
  }
  if(!getBuffersFilled())
  {
    computeElements();
    initializeBuffers(viewer);
  }
  getEdgeContainer(0)->setColor(QColor(Qt::red));
  if(viewer->isOpenGL_4_3())
  {
    QVector2D vp(viewer->width(), viewer->height());
    getEdgeContainer(0)->setViewport(vp);
    getEdgeContainer(0)->setWidth(5);
  }
  getEdgeContainer(0)->draw(viewer,true);
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

void Scene_rational_bezier_curve_item::initializeBuffers(Viewer_interface * viewer) const
{
  getEdgeContainer(0)->initializeBuffers(viewer);
  getEdgeContainer(0)->setFlatDataSize(d->polyline.size());
}

void Scene_rational_bezier_curve_item::computeElements() const
{
  getEdgeContainer(1)->allocate(
        Ec::Vertices, d->polyline.data(),
        static_cast<int>(d->polyline.size() * sizeof(float)));
}
