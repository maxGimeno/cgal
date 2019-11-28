#include "Scene_nurbs_item.h"
#include "Scene_spheres_item.h"
#include <CGAL/Three/Triangle_container.h>
#include <CGAL/Three/Edge_container.h>
#include <CGAL/Three/Three.h>

#include <dtkNurbsSurface>
#include <dtkNurbsPolyhedralSurface>
#include <dtkContinuousGeometryUtils>
#include <dtkNurbsCurve2D>
#include <dtkTrim>
#include <dtkTrimLoop>
#include <algorithm>
#include <QMenu>

typedef CGAL::Three::Triangle_container Tc;
typedef CGAL::Three::Edge_container Ec;
typedef CGAL::Three::Viewer_interface Vi;
class Scene_control_net_item : public Scene_spheres_item
{
public:
  Scene_control_net_item(Scene_group_item* parent, std::size_t max_index = 0, bool planed = false)
    :Scene_spheres_item(parent, max_index, planed)
  {
    vao_lines = new QOpenGLVertexArrayObject();
    vao_lines->create();
    vbo_lines = new QOpenGLBuffer();
    vbo_lines->create();
    is_initialized = false;
  }
  ~Scene_control_net_item()
  {
    vao_lines->destroy();
    delete vao_lines;
    vbo_lines->destroy();
    delete vbo_lines;
  }
  void initializeBuffers(Viewer_interface* viewer)const
  {
    QOpenGLShaderProgram* program = getShaderProgram(Scene_nurbs_item::PROGRAM_NO_SELECTION, viewer);
    program->bind();
    vao_lines->bind();
    vbo_lines->bind();

    vbo_lines->allocate(control_net.data(), static_cast<int>(control_net.size() * sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
    program->disableAttributeArray("colors");

    vbo_lines->release();
    vao_lines->release();
    program->release();
    is_initialized = true;
  }
  void draw_control_edges(Viewer_interface *viewer) const
  {
    if(!is_initialized)
      initializeBuffers(viewer);
    viewer->glEnable(GL_LINE_STIPPLE);
    attribBuffers(viewer, PROGRAM_NO_SELECTION);
    QOpenGLShaderProgram* program = getShaderProgram(PROGRAM_NO_SELECTION, viewer);
    program->bind();
    vao_lines->bind();
    program->setAttributeValue("colors", QColor(Qt::black));
    getEdgeContainer(0)->setWidth(5);
    viewer->glDrawArrays(GL_LINES, 0, control_net.size()/3);

    vao_lines->release();
    program->release();
    viewer->glDisable(GL_LINE_STIPPLE);
  }

  void draw(Viewer_interface *viewer) const
  {
    Scene_spheres_item::draw(viewer);
    draw_control_edges(viewer);

  }
  void drawEdges(Viewer_interface *viewer) const
  {
    Scene_spheres_item::drawEdges(viewer);
    if(renderingMode() == Wireframe)
      draw_control_edges(viewer);
  }
  void set_control_edges(const std::vector<float>& edges)
  {
    control_net = edges;
    is_initialized = false;
  }
private:
  mutable std::vector<float> control_net;
  QOpenGLVertexArrayObject *vao_lines;
  QOpenGLBuffer* vbo_lines;
  mutable bool is_initialized;


};
//Vertex source code
struct Scene_nurbs_item_priv{
  Scene_nurbs_item_priv(const dtkNurbsSurface& dtk_nurbs_surface,
                        Scene_nurbs_item* parent)
    : m_nurbs_surface(dtk_nurbs_surface), item(parent)

  {
    dtkLogger::instance().attachConsole();
    dtkLogger::instance().setLevel(dtkLog::Trace);
    spheres_item = NULL;
    trimmed_shown = false;
    cp_shown = false;
    float min_box[3];
    float max_box[3];
    std::fill_n(min_box, 3, std::numeric_limits<float>::infinity());
    std::fill_n(max_box, 3, -std::numeric_limits<float>::infinity());
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

    item->getTriangleContainer(1)->allocate(Tc::Smooth_vertices, untrimmed_vertices.data(),
                                      static_cast<int>(m_nb_untrimmed_vertices * 6 *
                                                       sizeof(float)));
    item->getTriangleContainer(1)->setOffset(Tc::Smooth_vertices, 0);
    item->getTriangleContainer(1)->setTupleSize(3);
    item->getTriangleContainer(1)->setStride(Tc::Smooth_vertices, 6*sizeof(float));

    item->getTriangleContainer(1)->allocate(Tc::Smooth_normals, untrimmed_vertices.data(),
                                            static_cast<int>(m_nb_untrimmed_vertices * 6 *
                                                             sizeof(float)));
    item->getTriangleContainer(1)->setOffset(Tc::Smooth_normals, 3 * sizeof(float));
    item->getTriangleContainer(1)->setStride(Tc::Smooth_normals, 6 * sizeof(float));

    item->getTriangleContainer(0)->allocate(Tc::Smooth_vertices, trimmed_vertices.data(),
                                      static_cast<int>(m_nb_trimmed_vertices * 6 *
                                                       sizeof(float)));
    item->getTriangleContainer(0)->setOffset(Tc::Smooth_vertices, 0);
    item->getTriangleContainer(0)->setTupleSize(3);
    item->getTriangleContainer(0)->setStride(Tc::Smooth_vertices, 6*sizeof(float));

    item->getTriangleContainer(0)->allocate(Tc::Smooth_normals, trimmed_vertices.data(),
                                            static_cast<int>(m_nb_trimmed_vertices * 6 *
                                                             sizeof(float)));
    item->getTriangleContainer(0)->setOffset(Tc::Smooth_normals, 3 * sizeof(float));
    item->getTriangleContainer(0)->setStride(Tc::Smooth_normals, 6 * sizeof(float));

    item->getEdgeContainer(0)->allocate(Ec::Vertices, intersection.data(),
                                            static_cast<int>(intersection.size() *
                                                             sizeof(float)));

  }

  void compute_spheres()
  {
    if(!spheres_item)
      return;
    //control points
    dtkContinuousGeometryPrimitives::Point_3 dtk_point(0., 0., .0);

    for(std::size_t i = 0; i < m_nurbs_surface.uNbCps(); ++i) {
      for(std::size_t j = 0; j < m_nurbs_surface.vNbCps(); ++j) {
        m_nurbs_surface.controlPoint(i, j, dtk_point.data());
        Scene_spheres_item::Kernel::Sphere_3 sphere(Scene_spheres_item::Kernel::Point_3(dtk_point[0],
                                                    dtk_point[1],
                                       dtk_point[2]), item->diagonalBbox()/280.0f);
        spheres_item->add_sphere(sphere,i*m_nurbs_surface.vNbCps()+j, CGAL::Color(120,120,25));
      }
    }
    for(std::size_t i = 0; i < m_nurbs_surface.uNbCps(); ++i) {
      for(std::size_t j = 0; j < m_nurbs_surface.vNbCps()-1; ++j) {
        m_nurbs_surface.controlPoint(i, j, dtk_point.data());
        control_net.push_back(dtk_point[0]);
        control_net.push_back(dtk_point[1]);
        control_net.push_back(dtk_point[2]);
        m_nurbs_surface.controlPoint(i, j+1, dtk_point.data());
        control_net.push_back(dtk_point[0]);
        control_net.push_back(dtk_point[1]);
        control_net.push_back(dtk_point[2]);
      }}
    for(std::size_t j = 0; j < m_nurbs_surface.vNbCps(); ++j) {
      for(std::size_t i = 0; i < m_nurbs_surface.uNbCps()-1; ++i) {
        m_nurbs_surface.controlPoint(i, j, dtk_point.data());
        control_net.push_back(dtk_point[0]);
        control_net.push_back(dtk_point[1]);
        control_net.push_back(dtk_point[2]);
        m_nurbs_surface.controlPoint(i+1, j, dtk_point.data());
        control_net.push_back(dtk_point[0]);
        control_net.push_back(dtk_point[1]);
        control_net.push_back(dtk_point[2]);
      }}
    spheres_item->setName(QString("Control Points"));
  }
  ~Scene_nurbs_item_priv()
  {
   if(spheres_item)
     spheres_item->deleteLater();
  }
  void computeElements() const
  {
  }


  const dtkNurbsSurface& m_nurbs_surface;

  enum Vao
  {
    UNTRIMMED_FACES=0,
    TRIMMED_FACES,
    INTERSECTIONS,
    CONTROL_NET,
    NumberOfVaos
  };

  enum Buffer
  {
    UNTRIMMED_FACES_BUFFER=0,
    TRIMMED_FACES_BUFFER,
    B_INTERSECTIONS,
    B_CONTROL_NET,
    NumberOfBuffers
  };

  mutable QOpenGLShaderProgram* m_program;
  mutable std::vector<GLuint> m_trimmed_elements;
  mutable std::vector<unsigned int> m_untrimmed_elements;
  mutable std::vector<float> untrimmed_vertices;
  mutable std::vector<float> trimmed_vertices;
  mutable std::vector<float> control_net;

  mutable std::vector<float> intersection;

  mutable std::size_t m_nb_trimmed_vertices;
  mutable std::size_t m_nb_trimmed_elements;
  mutable std::size_t m_nb_untrimmed_vertices;
  mutable std::size_t m_nb_untrimmed_elements;

  mutable bool trimmed_shown;
  mutable bool cp_shown;
  CGAL::Three::Scene_item::Bbox bbox;
  Scene_nurbs_item* item;
  Scene_control_net_item* spheres_item;
};

Scene_nurbs_item::Scene_nurbs_item(const dtkNurbsSurface& dtk_nurbs_surface)
  :CGAL::Three::Scene_group_item(QString("nurbs"))
{
  this->scene = CGAL::Three::Three::scene();

  setTriangleContainer(1,
                       new Tc(Vi::PROGRAM_WITH_LIGHT
                              ,true));

  setTriangleContainer(0,
                       new Tc(Vi::PROGRAM_WITH_LIGHT
                              ,true));

  setEdgeContainer(0,
                   new Ec( Vi::PROGRAM_NO_SELECTION
                           ,false));
  d = new Scene_nurbs_item_priv(dtk_nurbs_surface, this);
}

Scene_nurbs_item::~Scene_nurbs_item() { if(d) delete d; }



void Scene_nurbs_item::draw(CGAL::Three::Viewer_interface* viewer)const
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
    initializeBuffers(viewer);
    setBuffersFilled(true);
    setBuffersInit(viewer, true);
  }

  if(d->trimmed_shown)
  {
    getTriangleContainer(0)->setColor(this->color());
    getTriangleContainer(0)->draw(viewer, true);
  }
  else
  {
    getTriangleContainer(1)->setColor(this->color());
    getTriangleContainer(1)->draw(viewer, true);
    Scene_group_item::draw(viewer);
  }
}

void Scene_nurbs_item::drawEdges(CGAL::Three::Viewer_interface * viewer) const
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
    initializeBuffers(viewer);
    setBuffersFilled(true);
    setBuffersInit(viewer, true);
  }

    getEdgeContainer(0)->setColor(QColor(Qt::red));
    getEdgeContainer(0)->draw(viewer, true);
  
  Scene_group_item::drawEdges(viewer);
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

void Scene_nurbs_item::show_control_points(bool b)
{

  d->cp_shown = b;
  if(b)
  {
    if(d->spheres_item)
      return;
    d->spheres_item = new Scene_control_net_item(this, d->m_nurbs_surface.uNbCps() * d->m_nurbs_surface.vNbCps());
    d->compute_spheres();
    d->spheres_item->set_control_edges(d->control_net);
    scene->addItem(d->spheres_item);
    addChild(d->spheres_item);
    lockChild(d->spheres_item);
    scene->changeGroup(d->spheres_item, this);
  }
  else
  {
    if(!d->spheres_item)
      return;
    unlockChild(d->spheres_item);
    removeChild(d->spheres_item);
    scene->erase(scene->item_id(d->spheres_item));
    d->spheres_item = NULL;
  }
  QAction* actionShowCPs = contextMenu()->findChild<QAction*>("actionShowCPs");
  if(!actionShowCPs)
    return;
  actionShowCPs->setChecked(b);
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
    QAction* actionShowCPs=
      menu->addAction(tr("Show Control Points"));
    actionShowCPs->setCheckable(true);
    actionShowCPs->setChecked(false);
    actionShowCPs->setObjectName("actionShowCPs");
    connect(actionShowCPs, SIGNAL(toggled(bool)),
            this, SLOT(show_control_points(bool)));
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

void Scene_nurbs_item::initializeBuffers(Viewer_interface * viewer) const
{
  getTriangleContainer(0)->initializeBuffers(viewer);
  getTriangleContainer(0)->setIdxSize(d->m_nb_trimmed_elements);

  getTriangleContainer(0)->initializeBuffers(viewer);
  getTriangleContainer(0)->setIdxSize(d->m_nb_untrimmed_elements);


  d->trimmed_vertices.clear();
  d->trimmed_vertices.shrink_to_fit();
  d->untrimmed_vertices.clear();
  d->untrimmed_vertices.shrink_to_fit();

  getEdgeContainer(0)->initializeBuffers(viewer);
  getEdgeContainer(0)->setIdxSize(d->intersection.size()/3);

  d->intersection.clear();
  d->intersection.shrink_to_fit();
}
