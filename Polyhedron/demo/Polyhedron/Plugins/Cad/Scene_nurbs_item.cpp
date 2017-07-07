#include "Scene_nurbs_item.h"
#define foreach Q_FOREACH
#include <dtkNurbsSurface>
#include <dtkContinuousGeometryUtils>
#include <algorithm>
typedef Scene_nurbs_item_priv D;
//Vertex source code
struct Scene_nurbs_item_priv{

  Scene_nurbs_item_priv(const dtkNurbsSurface& dtk_nurbs_surface,
                        Scene_nurbs_item* parent)
    : m_nurbs_surface(dtk_nurbs_surface), item(parent)

  {
    float min_box[3];
    float max_box[3];
    std::fill_n(min_box, 3, std::numeric_limits<float>::infinity());
    std::fill_n(max_box, 3, -std::numeric_limits<float>::infinity());
    initialized = false;
    // ///////////////////////////////////////////////////////////////////
    // Sampling corresponds to the number of triangles edges along a NURBS edge
    // ///////////////////////////////////////////////////////////////////
    std::size_t u_sampling = 50;
    std::size_t v_sampling = 40;
    m_nb_vertices = (u_sampling + 1) * (v_sampling + 1);
    m_nb_elements = u_sampling * v_sampling * 2;
    // ///////////////////////////////////////////////////////////////////
    // Builds up the array of vertices by sampling the surface
    // ///////////////////////////////////////////////////////////////////
    m_vertices.resize(m_nb_vertices*3*3);

    double u_knots[dtk_nurbs_surface.uNbCps() + dtk_nurbs_surface.uDegree() - 1];
    dtk_nurbs_surface.uKnots(u_knots);
    double v_knots[dtk_nurbs_surface.vNbCps() + dtk_nurbs_surface.vDegree() - 1];
    dtk_nurbs_surface.vKnots(v_knots);

    dtkContinuousGeometryPrimitives::Point_3 point(0., 0., 0.);
    dtkContinuousGeometryPrimitives::Vector_3 normal(0., 0., 0.);
    for (std::size_t i = 0.; i <= u_sampling; ++i) {
      for (std::size_t j = 0.; j <= v_sampling; ++j) {
        m_nurbs_surface.evaluatePoint(
              u_knots[0] + double(i) / u_sampling * (u_knots[dtk_nurbs_surface.uNbCps() + dtk_nurbs_surface.uDegree() - 2] - u_knots[0]),
            v_knots[0] + double(j) / v_sampling * (v_knots[dtk_nurbs_surface.vNbCps() + dtk_nurbs_surface.vDegree() - 2] - v_knots[0]),
            point.data());
        for(int foo=0; foo<3; ++foo)
        {
         if(point[foo] < min_box[foo])
           min_box[foo]=point[foo];
         if(point[foo] > max_box[foo])
           max_box[foo]=point[foo];
        }
        m_nurbs_surface.evaluateNormal(
              u_knots[0] + double(i) / u_sampling * (u_knots[dtk_nurbs_surface.uNbCps() + dtk_nurbs_surface.uDegree() - 2] - u_knots[0]),
            v_knots[0] + double(j) / v_sampling * (v_knots[dtk_nurbs_surface.vNbCps() + dtk_nurbs_surface.vDegree() - 2] - v_knots[0]),
            normal.data());
        int index = i * (v_sampling + 1) + j;
        m_vertices[6 * index]     = point[0];
        m_vertices[6 * index + 1] = point[1];
        m_vertices[6 * index + 2] = point[2];

        m_vertices[6 * index + 3] = normal[0];
        m_vertices[6 * index + 4] = normal[1];
        m_vertices[6 * index + 5] = normal[2];
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
  void computeElements() const
  {
  }
  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const
  {

    // ///////////////////////////////////////////////////////////////////
    // Creates VBO EBO and VAO
    // ///////////////////////////////////////////////////////////////////
    m_program = item->getShaderProgram(Scene_nurbs_item::PROGRAM_WITH_LIGHT, viewer);
    m_program->bind();
    item->vaos[FACES]->bind();
    item->buffers[B_FACES].bind();

    item->buffers[B_FACES].allocate(m_vertices.data(), static_cast<int>(m_nb_vertices * 6 * sizeof(float)));
    m_program->enableAttributeArray("vertex");
    m_program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 6 * sizeof(float));
    m_program->enableAttributeArray("normals");
    m_program->setAttributeBuffer("normals", GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
    m_program->disableAttributeArray("colors");

    item->buffers[B_FACES].release();
    m_program->release();
    item->vaos[FACES]->release();

    initialized = true;
  }
  const dtkNurbsSurface& m_nurbs_surface;

  enum Buffer
  {
      B_FACES=0,
      NumberOfBuffers
  };
  enum Vao
  {
      FACES=0,
      NumberOfVaos
  };

  mutable QOpenGLShaderProgram* m_program;
  mutable std::vector<unsigned int> m_elements;
  mutable std::vector<float> m_vertices;
  mutable std::size_t m_nb_vertices;
  mutable std::size_t m_nb_elements;
  mutable bool initialized;
  CGAL::Three::Scene_item::Bbox bbox;
  Scene_nurbs_item* item;
};

Scene_nurbs_item::Scene_nurbs_item(const dtkNurbsSurface& dtk_nurbs_surface)
  :CGAL::Three::Scene_item(D::NumberOfBuffers, D::NumberOfVaos)
{
  d = new Scene_nurbs_item_priv(dtk_nurbs_surface, this);
}

Scene_nurbs_item::~Scene_nurbs_item() { if(d) delete d; }



void Scene_nurbs_item::draw(CGAL::Three::Viewer_interface* viewer)const
{
  if(!d->initialized)
    d->initializeBuffers(viewer);

  attribBuffers(viewer, PROGRAM_WITH_LIGHT);
  d->m_program = getShaderProgram(PROGRAM_WITH_LIGHT, viewer);
  d->m_program->bind();
  vaos[D::FACES]->bind();
  d->m_program->setAttributeValue("colors", this->color());
  viewer->glDrawElements(GL_TRIANGLES, static_cast<GLuint>(d->m_elements.size()), GL_UNSIGNED_INT, d->m_elements.data());
  vaos[D::FACES]->release();
  d->m_program->release();
}

void Scene_nurbs_item::compute_bbox() const
{
  _bbox = d->bbox;
}

bool Scene_nurbs_item::isEmpty() const
{
  bool res = d->m_elements.empty();
  return res;
}
