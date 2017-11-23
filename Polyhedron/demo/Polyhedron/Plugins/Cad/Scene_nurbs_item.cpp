#include "Scene_nurbs_item.h"
#define foreach Q_FOREACH

#include <dtkNurbsSurface>
#include <dtkNurbsPolyhedralSurface>
#include <dtkContinuousGeometryUtils>
#include <dtkNurbsCurve2D>
#include <dtkTrim>
#include <dtkTrimLoop>
#include <algorithm>
#include <QMenu>

typedef Scene_nurbs_item_priv D;
//Vertex source code
struct Scene_nurbs_item_priv{

  Scene_nurbs_item_priv(const dtkNurbsSurface& dtk_nurbs_surface,
                        Scene_nurbs_item* parent)
    : m_nurbs_surface(dtk_nurbs_surface), item(parent)

  {
    dtkLogger::instance().attachConsole();
    dtkLogger::instance().setLevel(dtkLog::Info);

    trimmed_shown = false;
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
    m_nb_untrimmed_vertices = (u_sampling + 1) * (v_sampling + 1);
    m_nb_untrimmed_elements = u_sampling * v_sampling * 2;
    // ///////////////////////////////////////////////////////////////////
    // Builds up the array of vertices by sampling the surface
    // ///////////////////////////////////////////////////////////////////
    untrimmed_vertices.resize(m_nb_untrimmed_vertices * 3 * 3);

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
        untrimmed_vertices[6 * index]     = point[0];
        untrimmed_vertices[6 * index + 1] = point[1];
        untrimmed_vertices[6 * index + 2] = point[2];

        untrimmed_vertices[6 * index + 3] = normal[0];
        untrimmed_vertices[6 * index + 4] = normal[1];
        untrimmed_vertices[6 * index + 5] = normal[2];
      }
    }
    bbox = CGAL::Three::Scene_item::Bbox(min_box[0], min_box[1], min_box[2],
        max_box[0], max_box[1], max_box[2]);

    m_untrimmed_elements.resize(m_nb_untrimmed_elements * 3);
    std::size_t cntr = 0;
    for (GLuint i = 0; i < (u_sampling); ++i) {
      for (GLuint j = 0; j < (v_sampling); ++j, cntr+=6) {
        m_untrimmed_elements[cntr + 0] = i * (v_sampling + 1) + j;
        m_untrimmed_elements[cntr + 1] = i * (v_sampling + 1) + j + 1;
        m_untrimmed_elements[cntr + 2] = i * (v_sampling + 1) + j + (v_sampling + 1);

        m_untrimmed_elements[cntr + 3] = i * (v_sampling + 1) + j + 1;
        m_untrimmed_elements[cntr + 4] = i * (v_sampling + 1) + j + (v_sampling + 1) + 1;
        m_untrimmed_elements[cntr + 5] = i * (v_sampling + 1) + j + (v_sampling + 1);
      }
    }


    std::vector< dtkContinuousGeometryPrimitives::Point_3 > points;

    dtkNurbsPolyhedralSurface *polyhedral_surface = dtkContinuousGeometry::nurbsPolyhedralSurface::pluginFactory().create("dtkNurbsPolyhedralSurfaceCgal");
    if (polyhedral_surface == nullptr) {
        dtkFatal() << "The dtkAbstractNurbsPolyhedralSurfaceData could not be loaded by the factory under the cgal implementation";
    }
    dtkDebug() << "Initialization of polyhedral NURBS surface...";
    double approximation = 1e-4 * std::sqrt((bbox.xmax() - bbox.xmin()) * (bbox.xmax() - bbox.xmin()) + (bbox.ymax() - bbox.ymin()) * (bbox.ymax() - bbox.ymin()) + (bbox.zmax() - bbox.zmin()) * (bbox.zmax() - bbox.zmin()));
    std::cerr << "approximation :" << approximation << std::endl;
    polyhedral_surface->initialize(const_cast<dtkNurbsSurface*>(&m_nurbs_surface), approximation);
    dtkDebug() << "Polyhedral NURBS surface initialized...";

    dtkDebug() << "Recovering points and triangles...";

    std::vector< std::size_t > triangles;
    std::vector< dtkContinuousGeometryPrimitives::Vector_3 > normals;
    polyhedral_surface->pointsTrianglesAndNormals(points, triangles, normals);
    for(auto t : triangles) { m_trimmed_elements.push_back(GLuint(t));}

    dtkDebug() << "Points and triangles recovered";

    dtkDebug() << "Filling up trimmed_vertices...";
    dtkDebug() << "Nb of triangles (*3) : " << m_trimmed_elements.size();

    trimmed_vertices.resize(points.size() * 6);
    for (std::size_t i = 0; i < points.size(); ++i) {
      trimmed_vertices[6 * i + 0] = points[i][0];
      trimmed_vertices[6 * i + 1] = points[i][1];
      trimmed_vertices[6 * i + 2] = points[i][2];
      trimmed_vertices[6 * i + 3] = normals[i][0];
      trimmed_vertices[6 * i + 4] = normals[i][1];
      trimmed_vertices[6 * i + 5] = normals[i][2];
    }

    m_nb_trimmed_vertices = points.size();
    m_nb_trimmed_elements = m_trimmed_elements.size() / 3.;

    dtkDebug() << "trimmed_vertices filled up";

    dtkDebug() << "Generating intersection lines...";
    for (auto trim_loop = m_nurbs_surface.trimLoops().begin(); trim_loop != m_nurbs_surface.trimLoops().end(); ++trim_loop) {
      for (auto trim = (*trim_loop)->trims().begin(); trim != (*trim_loop)->trims().end(); ++trim)
      {
        std::size_t length = (*trim)->curve2D().nbCps() + (*trim)->curve2D().degree() - 1;
        double knots[length];
        (*trim)->curve2D().knots(knots);
        dtkContinuousGeometryPrimitives::Point_2 p(0,0);
        dtkContinuousGeometryPrimitives::Point_3 p3D(0,0,0);
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
  void computeElements() const
  {
  }

  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const
  {
    dtkDebug() << "Initializing buffer...";
    // ///////////////////////////////////////////////////////////////////
    // Creates VBO EBO and VAO
    // ///////////////////////////////////////////////////////////////////
    m_program = item->getShaderProgram(Scene_nurbs_item::PROGRAM_WITH_LIGHT, viewer);
    m_program->bind();
    item->vaos[UNTRIMMED_FACES]->bind();
    item->buffers[UNTRIMMED_FACES_BUFFER].bind();

    item->buffers[UNTRIMMED_FACES_BUFFER].allocate(untrimmed_vertices.data(), static_cast<int>(m_nb_untrimmed_vertices * 6 * sizeof(float)));
    m_program->enableAttributeArray("vertex");
    m_program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 6 * sizeof(float));
    m_program->enableAttributeArray("normals");
    m_program->setAttributeBuffer("normals", GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
    m_program->disableAttributeArray("colors");

    item->buffers[UNTRIMMED_FACES_BUFFER].release();
    item->vaos[UNTRIMMED_FACES]->release();

    item->vaos[TRIMMED_FACES]->bind();
    item->buffers[TRIMMED_FACES_BUFFER].bind();

    item->buffers[TRIMMED_FACES_BUFFER].allocate(trimmed_vertices.data(), static_cast<int>(m_nb_trimmed_vertices * 6 * sizeof(float)));
    m_program->enableAttributeArray("vertex");
    m_program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 6 * sizeof(float));
    m_program->enableAttributeArray("normals");
    m_program->setAttributeBuffer("normals", GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
    m_program->disableAttributeArray("colors");

    item->buffers[TRIMMED_FACES_BUFFER].release();
    item->vaos[TRIMMED_FACES]->release();

    m_program->release();

    m_program = item->getShaderProgram(Scene_nurbs_item::PROGRAM_NO_SELECTION, viewer);
    m_program->bind();
    item->vaos[INTERSECTIONS]->bind();
    item->buffers[B_INTERSECTIONS].bind();

    item->buffers[B_INTERSECTIONS].allocate(intersection.data(), static_cast<int>(intersection.size() * sizeof(float)));
    m_program->enableAttributeArray("vertex");
    m_program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
    m_program->disableAttributeArray("colors");

    item->buffers[B_INTERSECTIONS].release();
    item->vaos[INTERSECTIONS]->release();
    m_program->release();

    initialized = true;
    dtkDebug() << "Buffers initialized...";
  }
  const dtkNurbsSurface& m_nurbs_surface;

  enum Vao
  {
    UNTRIMMED_FACES=0,
    TRIMMED_FACES,
    INTERSECTIONS,
    NumberOfVaos
  };

  enum Buffer
  {
    UNTRIMMED_FACES_BUFFER=0,
    TRIMMED_FACES_BUFFER,
    B_INTERSECTIONS,
    NumberOfBuffers
  };

  mutable QOpenGLShaderProgram* m_program;
  mutable std::vector<GLuint> m_trimmed_elements;
  mutable std::vector<unsigned int> m_untrimmed_elements;
  mutable std::vector<float> untrimmed_vertices;
  mutable std::vector<float> trimmed_vertices;

  mutable std::vector<float> intersection;

  mutable std::size_t m_nb_trimmed_vertices;
  mutable std::size_t m_nb_trimmed_elements;
  mutable std::size_t m_nb_untrimmed_vertices;
  mutable std::size_t m_nb_untrimmed_elements;
  mutable bool initialized;
  mutable bool trimmed_shown;
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
  if(d->trimmed_shown)
  {
    vaos[D::TRIMMED_FACES]->bind();
    d->m_program->setAttributeValue("colors", this->color());
    viewer->glDrawElements(GL_TRIANGLES, static_cast<GLuint>(d->m_trimmed_elements.size()), GL_UNSIGNED_INT, d->m_trimmed_elements.data());
    vaos[D::TRIMMED_FACES]->release();
    d->m_program->release();
  }
  else
  {
    vaos[D::UNTRIMMED_FACES]->bind();
    d->m_program->setAttributeValue("colors", this->color());
    viewer->glDrawElements(GL_TRIANGLES, static_cast<GLuint>(d->m_untrimmed_elements.size()), GL_UNSIGNED_INT, d->m_untrimmed_elements.data());
    vaos[D::UNTRIMMED_FACES]->release();
    d->m_program->release();
  }

}

void Scene_nurbs_item::drawEdges(CGAL::Three::Viewer_interface * viewer) const
{
  if(!d->initialized)
    d->initializeBuffers(viewer);

  attribBuffers(viewer, PROGRAM_NO_SELECTION);
  d->m_program = getShaderProgram(PROGRAM_NO_SELECTION, viewer);
  d->m_program->bind();
  vaos[D::INTERSECTIONS]->bind();
  d->m_program->setAttributeValue("colors", QColor(Qt::red));
  viewer->glDrawArrays(GL_LINES, 0, d->intersection.size()/3);
  vaos[D::INTERSECTIONS]->release();
  d->m_program->release();
}

void Scene_nurbs_item::compute_bbox() const
{
  _bbox = d->bbox;
}

bool Scene_nurbs_item::isEmpty() const
{
  return d->m_untrimmed_elements.empty()
      && d->m_trimmed_elements.empty();
}

void Scene_nurbs_item::show_trimmed(bool b)
{
  d->trimmed_shown = b;
  QAction* actionShowTrimmed = contextMenu()->findChild<QAction*>("actionShowTrimmed");
  if(!actionShowTrimmed)
    return;
  actionShowTrimmed->setChecked(b);
  itemChanged();
}

QMenu* Scene_nurbs_item::contextMenu()
{
  const char* prop_name = "Menu modified by Scene_nurbs_item.";

  QMenu* menu = Scene_item::contextMenu();

  // Use dynamic properties:
  // http://doc.qt.io/qt-5/qobject.html#property
  bool menuChanged = menu->property(prop_name).toBool();

  if (!menuChanged) {
    QAction* actionShowTrimmed=
      menu->addAction(tr("Show Trimmed"));
    actionShowTrimmed->setCheckable(true);
    actionShowTrimmed->setChecked(false);
    actionShowTrimmed->setObjectName("actionShowTrimmed");
    connect(actionShowTrimmed, SIGNAL(toggled(bool)),
            this, SLOT(show_trimmed(bool)));

    menu->setProperty(prop_name, true);
  }
  return menu;
}

void Scene_nurbs_item::highlight(const dtkTopoTrim *)
{
    // ///////////////////////////////////////////////////////////////////
    // Removes old colors
    // ///////////////////////////////////////////////////////////////////

}
