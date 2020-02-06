#undef QT_NO_KEYWORDS
#include "Scene_rational_bezier_surface_item.h"
#define foreach Q_FOREACH

#include <dtkLog>

#include <dtkRationalBezierSurface>
#include <dtkContinuousGeometryUtils>
#include <algorithm>
#include <QMenu>
#include <CGAL/Three/Triangle_container.h>
#include <CGAL/Three/Three.h>

using namespace CGAL::Three;
typedef Triangle_container Tc;
typedef Viewer_interface Vi;
//Vertex source code
struct Scene_rational_bezier_surface_item_priv{

  Scene_rational_bezier_surface_item_priv(const dtkRationalBezierSurface& dtk_rational_bezier_surface,
                        Scene_rational_bezier_surface_item* parent)
    : m_rational_bezier_surface(dtk_rational_bezier_surface), item(parent)
  {
    dtkLogger::instance().attachConsole();
    dtkLogger::instance().setLevel(dtkLog::Error);

    trimmed_shown = false;
    float min_box[3];
    float max_box[3];
    std::fill_n(min_box, 3, std::numeric_limits<float>::infinity());
    std::fill_n(max_box, 3, -std::numeric_limits<float>::infinity());
    // ///////////////////////////////////////////////////////////////////
    // Sampling corresponds to the number of triangles edges along a RATIONAL_BEZIER_SURFACE edge
    // ///////////////////////////////////////////////////////////////////
    std::size_t u_sampling = 50;
    std::size_t v_sampling = 40;
    m_nb_vertices = (u_sampling + 1) * (v_sampling + 1);
    m_nb_elements = u_sampling * v_sampling * 2;
    // ///////////////////////////////////////////////////////////////////
    // Builds up the array of vertices by sampling the surface
    // ///////////////////////////////////////////////////////////////////
    vertices.resize(m_nb_vertices * 3 * 3);

    dtkContinuousGeometryPrimitives::Point_3 point(0., 0., 0.);
    dtkContinuousGeometryPrimitives::Vector_3 normal(0., 0., 0.);
    for (std::size_t i = 0.; i <= u_sampling; ++i) {
      for (std::size_t j = 0.; j <= v_sampling; ++j) {
        m_rational_bezier_surface.evaluatePoint(
                                                double(i)/ u_sampling,
                                                double(j)/ v_sampling,
                                                point.data());
        for(int foo=0; foo<3; ++foo)
            {
                if(point[foo] < min_box[foo])
                    min_box[foo]=point[foo];
                if(point[foo] > max_box[foo])
                    max_box[foo]=point[foo];
            }
        m_rational_bezier_surface.evaluateNormal(
          double(i) / u_sampling,
          double(j) / v_sampling,
          normal.data());
        int index = i * (v_sampling + 1) + j;
        vertices[6 * index]     = point[0];
        vertices[6 * index + 1] = point[1];
        vertices[6 * index + 2] = point[2];

        vertices[6 * index + 3] = normal[0];
        vertices[6 * index + 4] = normal[1];
        vertices[6 * index + 5] = normal[2];
      }
    }
    bbox = CGAL::Three::Scene_item::Bbox(min_box[0], min_box[1], min_box[2],
        max_box[0], max_box[1], max_box[2]);

    m_elements.resize(m_nb_elements * 3);
    std::size_t cntr = 0;
    for (GLuint i = 0; i < (u_sampling); ++i) {
      for (GLuint j = 0; j < (v_sampling); ++j, cntr+=6) {
        m_elements[cntr + 0] = i * (v_sampling + 1) + j;
        m_elements[cntr + 1] = i * (v_sampling + 1) + j + 1;
        m_elements[cntr + 2] = i * (v_sampling + 1) + j + (v_sampling + 1);

        m_elements[cntr + 3] = i * (v_sampling + 1) + j + 1;
        m_elements[cntr + 4] = i * (v_sampling + 1) + j + (v_sampling + 1) + 1;
        m_elements[cntr + 5] = i * (v_sampling + 1) + j + (v_sampling + 1);
      }
    }
  }

  const dtkRationalBezierSurface& m_rational_bezier_surface;

  mutable QOpenGLShaderProgram* m_program;
  mutable std::vector<GLuint> m_elements;
  mutable std::vector<float> vertices;
  mutable std::size_t m_nb_vertices;
  mutable std::size_t m_nb_elements;
  mutable bool trimmed_shown;
  CGAL::Three::Scene_item::Bbox bbox;
  Scene_rational_bezier_surface_item* item;
};

Scene_rational_bezier_surface_item::Scene_rational_bezier_surface_item(const dtkRationalBezierSurface& dtk_rational_bezier_surface)
{

  setTriangleContainer(0,
                    new Tc(Vi::PROGRAM_WITH_LIGHT, true));
  d = new Scene_rational_bezier_surface_item_priv(dtk_rational_bezier_surface, this);
  invalidateOpenGLBuffers();
}

Scene_rational_bezier_surface_item::~Scene_rational_bezier_surface_item() { if(d) delete d; }

QString Scene_rational_bezier_surface_item::toolTip() const {
    return tr("<p><b>Rational Bezier Surface</b></p>"
              "<p> Degree in U direction: %1<br />"
              "Degree in V direction: %2</p>%3")
        .arg(d->m_rational_bezier_surface.uDegree())
        .arg(d->m_rational_bezier_surface.vDegree())
        .arg(property("toolTip").toString());
}

void Scene_rational_bezier_surface_item::initializeBuffers(Viewer_interface *viewer) const
{
  getTriangleContainer(0)->initializeBuffers(viewer);
  getTriangleContainer(0)->setIdxSize(d->m_nb_elements*3);
}


void Scene_rational_bezier_surface_item::draw(CGAL::Three::Viewer_interface* viewer)const
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
  getTriangleContainer(0)->setColor(color());
  getTriangleContainer(0)->draw( viewer,true);
}

void Scene_rational_bezier_surface_item::compute_bbox() const
{
  _bbox = d->bbox;
}

bool Scene_rational_bezier_surface_item::isEmpty() const
{
  return d->m_elements.empty();
}

QMenu* Scene_rational_bezier_surface_item::contextMenu()
{
  const char* prop_name = "Menu modified by Scene_rational_bezier_surface_item.";

  QMenu* menu = Scene_item::contextMenu();

  bool menuChanged = menu->property(prop_name).toBool();

  if (!menuChanged) {
    menu->setProperty(prop_name, true);
  }
  return menu;
}

void Scene_rational_bezier_surface_item::computeElements() const
{

  getTriangleContainer(0)->allocate(Tc::Vertex_indices, d->m_elements.data(),
                                    static_cast<int>(d->m_elements.size() *
                                                     sizeof(unsigned int)));
  getTriangleContainer(0)->allocate(Tc::Smooth_vertices, d->vertices.data(),
                              static_cast<int>(d->m_nb_vertices * 6 *
                                               sizeof(float)));
  getTriangleContainer(0)->setOffset(Tc::Smooth_vertices, 0);

  getTriangleContainer(0)->setStride(Tc::Smooth_vertices, 6*sizeof(float));

  getTriangleContainer(0)->allocate(Tc::Smooth_normals, d->vertices.data(),
                                    static_cast<int>(d->m_nb_vertices * 6 *
                                                     sizeof(float)));
  getTriangleContainer(0)->setOffset(Tc::Smooth_normals, 3 * sizeof(float));
  getTriangleContainer(0)->setStride(Tc::Smooth_normals, 6 * sizeof(float));

}
