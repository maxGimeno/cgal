#include "config.h"
#include "Scene_spheres_item.h"
#include "Scene_c3t3_item.h"

#include <QVector>
#include <QColor>
#include <QPixmap>
#include <QApplication>
#include <QPainter>
#include <QtCore/qglobal.h>
#include <QGuiApplication>
#include <QSlider>
#include <QWidgetAction>
#include <QKeyEvent>

#include <map>
#include <vector>
#include <CGAL/Three/Scene_interface.h>
#include <CGAL/Real_timer.h>

#include <QGLViewer/manipulatedFrame.h>
#include <QGLViewer/qglviewer.h>

#include <boost/function_output_iterator.hpp>
#include <boost/foreach.hpp>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangulation_3_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Three/Triangle_container.h>
#include <CGAL/Three/Edge_container.h>

#include <CGAL/Primitives_filler.h>

typedef CGAL::AABB_triangulation_3_triangle_primitive<Kernel,C3t3> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;
typedef Viewer_interface VI;
typedef CGAL::Three::Triangle_container Tri;
typedef CGAL::Three::Edge_container Ed;

typedef Scene_c3t3_item_priv Priv;


// The special Scene_item only for triangles
class Scene_intersection_item : public CGAL::Three::Scene_item_rendering_helper
{
  Q_OBJECT
public :
  Scene_intersection_item(Scene_c3t3_item* parent)
  :CGAL::Three::Scene_item_rendering_helper(),
    is_fast(false)
  {
    setParent(parent);
    setTriangleContainer(0,
          new Triangle_container(VI::PROGRAM_C3T3, false));
    setEdgeContainer(0,
          new Edge_container(VI::PROGRAM_C3T3_EDGES, false));
  }
  void init_vectors(
      std::vector<float> *p_vertices,
      std::vector<float> *p_normals,
      std::vector<float> *p_edges,
      std::vector<float> *p_colors)
  {
    vertices = p_vertices;
    normals = p_normals;
    edges = p_edges;
    colors = p_colors;
  }

  void allocate()
   {
    if(!isInit())
      initGL();

    getTriangleContainer(0)->reset_vbos(ALL);
    getTriangleContainer(0)->allocate(Tri::Flat_vertices, vertices->data(),
                                      static_cast<int>(vertices->size()*sizeof(float)));
    getTriangleContainer(0)->allocate(Tri::Flat_normals, normals->data(),
                                      static_cast<int>(normals->size()*sizeof(float)));
    getTriangleContainer(0)->allocate(Tri::FColors, colors->data(),
                                      static_cast<int>(colors->size()*sizeof(float)));
    getTriangleContainer(0)->setFlatDataSize( vertices->size());

    getEdgeContainer(0)->reset_vbos(ALL);
    getEdgeContainer(0)->allocate(Ed::Vertices, edges->data(),
                                                     static_cast<int>(edges->size()*sizeof(float)));
    getEdgeContainer(0)->setFlatDataSize(edges->size());

   }

  void setColor(QColor c)
  {
    qobject_cast<Scene_c3t3_item*>(this->parent())->setColor(c);
    Scene_item::setColor(c);
  }
  // Indicates if rendering mode is supported
  bool supportsRenderingMode(RenderingMode m) const {
    return (m != Gouraud && m != PointsPlusNormals && m != Points && m != ShadedPoints);
  }
  void initializeBuffers(CGAL::Three::Viewer_interface *viewer) Q_DECL_OVERRIDE
  {
    getTriangleContainer(0)->initializeBuffers(viewer);
    getEdgeContainer(0)->initializeBuffers(viewer);
  }
  //Displays the item
  void draw(Viewer_interface *viewer, int pass, bool is_writing, QOpenGLFramebufferObject *fbo)
  {
    if(is_fast)
      return;

    if(!isWriting() && !isInit())
    {
      initGL();
    }

    if (!isWriting() && getBuffersFilled() &&
        ! getBuffersInit(viewer))
    {
      initializeBuffers(viewer);
      setBuffersInit(viewer, true);
    }
    float near(viewer->camera()->zNear()), far(viewer->camera()->zFar());
    const Kernel::Plane_3& plane = qobject_cast<Scene_c3t3_item*>(this->parent())->plane();
    QVector4D cp(-plane.a(), -plane.b(), -plane.c(), -plane.d());
    getTriangleContainer(0)->setPlane(cp);
    getTriangleContainer(0)->setShrinkFactor(
        qobject_cast<Scene_c3t3_item*>(this->parent())->getShrinkFactor());
    getTriangleContainer(0)->setComparing(pass > 0);
    getTriangleContainer(0)->setWidth(viewer->width()*1.0f);
    getTriangleContainer(0)->setHeight(viewer->height()*1.0f);
    getTriangleContainer(0)->setNear(near);
    getTriangleContainer(0)->setFar(far);
    getTriangleContainer(0)->setDepthWriting(is_writing);
    getTriangleContainer(0)->setColor(isSelected() ? selectionColor() : color());
    getTriangleContainer(0)->setSelected(isSelected());
    getTriangleContainer(0)->setAlpha(alpha());
    getTriangleContainer(0)->draw(viewer, false, fbo);
  }
  void drawEdges(CGAL::Three::Viewer_interface* viewer)
  {
    if(is_fast)
      return;
    if(!isWriting() && !isInit())
    {
      initGL();
    }

    if (!isWriting() && getBuffersFilled() &&
        ! getBuffersInit(viewer))
    {
      initializeBuffers(viewer);
      setBuffersInit(viewer, true);
    }
    const Kernel::Plane_3& plane = qobject_cast<Scene_c3t3_item*>(this->parent())->plane();
    QVector4D cp(-plane.a(), -plane.b(), -plane.c(), -plane.d());
    getEdgeContainer(0)->setPlane(cp);
    getEdgeContainer(0)->setColor(
        QColor(Qt::black));
    getEdgeContainer(0)->draw(viewer, true);
  }

  void setFast(bool b)
  {
    is_fast = b;
  }

  void addTriangle(const Tr::Bare_point& pa, const Tr::Bare_point& pb,
                   const Tr::Bare_point& pc, const CGAL::Color color)
  {
    const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
    Geom_traits::Vector_3 n = cross_product(pb - pa, pc - pa);
    n = n / CGAL::sqrt(n*n);

    for (int i = 0; i<3; i++)
    {
      normals->push_back(n.x());
      normals->push_back(n.y());
      normals->push_back(n.z());
    }

    vertices->push_back(pa.x()+offset.x);
    vertices->push_back(pa.y()+offset.y);
    vertices->push_back(pa.z()+offset.z);

    vertices->push_back(pb.x()+offset.x);
    vertices->push_back(pb.y()+offset.y);
    vertices->push_back(pb.z()+offset.z);

    vertices->push_back(pc.x()+offset.x);
    vertices->push_back(pc.y()+offset.y);
    vertices->push_back(pc.z()+offset.z);

    edges->push_back(pa.x()+offset.x);
    edges->push_back(pa.y()+offset.y);
    edges->push_back(pa.z()+offset.z);

    edges->push_back(pb.x()+offset.x);
    edges->push_back(pb.y()+offset.y);
    edges->push_back(pb.z()+offset.z);

    edges->push_back(pb.x()+offset.x);
    edges->push_back(pb.y()+offset.y);
    edges->push_back(pb.z()+offset.z);

    edges->push_back(pc.x()+offset.x);
    edges->push_back(pc.y()+offset.y);
    edges->push_back(pc.z()+offset.z);

    edges->push_back(pc.x()+offset.x);
    edges->push_back(pc.y()+offset.y);
    edges->push_back(pc.z()+offset.z);

    edges->push_back(pa.x()+offset.x);
    edges->push_back(pa.y()+offset.y);
    edges->push_back(pa.z()+offset.z);

    for(int i=0; i<3; i++)
    {
      colors->push_back((float)color.red()/255);
      colors->push_back((float)color.green()/255);
      colors->push_back((float)color.blue()/255);
    }
  }
  void compute_bbox() const{}

  Scene_item* clone() const {return 0;}
  QString toolTip() const {return QString();}
private:


  //contains the data
  mutable std::vector<float> *vertices;
  mutable std::vector<float> *normals;
  mutable std::vector<float> *edges;
  mutable std::vector<float> *colors;
  mutable QOpenGLShaderProgram *program;
  mutable bool is_fast;
}; //end of class Scene_triangle_item


struct Scene_c3t3_item_priv {
  typedef qglviewer::ManipulatedFrame ManipulatedFrame;
  Scene_c3t3_item_priv(Scene_c3t3_item* item)
    : item(item), c3t3()
    , frame(new ManipulatedFrame())
    , data_item_(NULL)
    , histogram_()
    , surface_patch_indices_()
    , subdomain_indices_()
    , is_valid(true)
  {
    init_default_values();
    tet_Slider = NULL;
    invalidate_stats();
  }
  Scene_c3t3_item_priv(const C3t3& c3t3_, Scene_c3t3_item* item)
    : item(item), c3t3(c3t3_)
    , frame(new ManipulatedFrame())
    , data_item_(NULL)
    , histogram_()
    , surface_patch_indices_()
    , subdomain_indices_()
    , is_valid(true)
  {
    init_default_values();
    tet_Slider = NULL;
    invalidate_stats();
  }
  ~Scene_c3t3_item_priv()
  {
    c3t3.clear();
    tree.clear();
    if(frame)
    {
      static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->setManipulatedFrame(0);
      delete frame;
      frame = NULL;
      if(tet_Slider)
        delete tet_Slider;
    }
  }

  void init_default_values() {
    positions_lines.resize(0);
    positions_poly.resize(0);
    normals.resize(0);
    s_vertex.resize(0);
    s_normals.resize(0);
    ws_vertex.resize(0);
    need_changed = false;
    spheres = NULL;
    intersection = NULL;
    spheres_are_shown = false;
    cnc_are_shown = false;
    show_tetrahedra = false;
    is_aabb_tree_built = false;
    is_grid_shown = true;
  }
  void computeIntersection(const Primitive& facet);
  void computeElements(Scene_item::Gl_data_names name);
  void fill_aabb_tree() {
    if(item->isEmpty()) return;
    QGuiApplication::setOverrideCursor(Qt::WaitCursor);
    CGAL::Real_timer timer;
    timer.start();
    tree.clear();
    for (Tr::Finite_facets_iterator
           fit = c3t3.triangulation().finite_facets_begin(),
           end = c3t3.triangulation().finite_facets_end();
         fit != end; ++fit)
    {
      Tr::Cell_handle ch = fit->first, nh =ch->neighbor(fit->second);

      if( (!c3t3.is_in_complex(ch)) &&  (!c3t3.is_in_complex(nh)) )
        continue;

      if(c3t3.is_in_complex(ch)){
        tree.insert(Primitive(fit));
      } else{
        int ni = nh->index(ch);
        tree.insert(Primitive(Tr::Facet(nh,ni)));
      }
    }
    tree.build();
    std::cerr << "C3t3 facets AABB tree built in " << timer.time()
              << " wall-clock seconds\n";

    is_aabb_tree_built = true;
    QGuiApplication::restoreOverrideCursor();
  }
  void reset_cut_plane();

  void draw_triangle_edges_cnc(const Tr::Bare_point& pa,
                               const Tr::Bare_point& pb,
                               const Tr::Bare_point& pc) const;
  double complex_diag() const;
  void compute_color_map(const QColor& c);
  void initialize_intersection_buffers(CGAL::Three::Viewer_interface *viewer);
  void computeSpheres();
  void computeIntersections();

  void invalidate_stats()
  {
    min_edges_length = std::numeric_limits<float>::max();
    max_edges_length = 0;
    mean_edges_length = 0;
    min_dihedral_angle = std::numeric_limits<float>::max();
    max_dihedral_angle = 0;
    mean_dihedral_angle = 0;
    nb_subdomains = 0;
    nb_spheres = 0;
    nb_cnc = 0;
    nb_vertices = 0;
    nb_tets = 0;
    smallest_radius_radius = std::numeric_limits<float>::max();
    smallest_edge_radius = std::numeric_limits<float>::max();
    biggest_v_sma_cube = 0;
    computed_stats = false;
  }




  enum STATS {
    MIN_EDGES_LENGTH = 0,
    MAX_EDGES_LENGTH,
    MEAN_EDGES_LENGTH,
    MIN_DIHEDRAL_ANGLE,
    MAX_DIHEDRAL_ANGLE,
    MEAN_DIHEDRAL_ANGLE,
    NB_SPHERES,
    NB_CNC,
    NB_VERTICES,
    NB_TETS,
    SMALLEST_RAD_RAD,
    SMALLEST_EDGE_RAD,
    BIGGEST_VL3_CUBE,
    NB_SUBDOMAINS
  };
  Scene_c3t3_item* item;
  C3t3 c3t3;
  bool is_grid_shown;
  qglviewer::ManipulatedFrame* frame;
  bool need_changed;
  Scene_spheres_item *spheres;
  Scene_intersection_item *intersection;
  bool spheres_are_shown;
  const Scene_item* data_item_;
  QPixmap histogram_;
  typedef std::set<int> Indices;
  Indices surface_patch_indices_;
  Indices subdomain_indices_;
  std::set<Tr::Cell_handle> intersected_cells;
  QSlider* tet_Slider;

  //!Allows OpenGL 2.1 context to get access to glDrawArraysInstanced.
  typedef void (APIENTRYP PFNGLDRAWARRAYSINSTANCEDARBPROC) (GLenum mode, GLint first, GLsizei count, GLsizei primcount);
  //!Allows OpenGL 2.1 context to get access to glVertexAttribDivisor.
  typedef void (APIENTRYP PFNGLVERTEXATTRIBDIVISORARBPROC) (GLuint index, GLuint divisor);
  //!Allows OpenGL 2.1 context to get access to gkFrameBufferTexture2D.
  PFNGLDRAWARRAYSINSTANCEDARBPROC glDrawArraysInstanced;
  //!Allows OpenGL 2.1 context to get access to glVertexAttribDivisor.
  PFNGLVERTEXATTRIBDIVISORARBPROC glVertexAttribDivisor;

  mutable std::size_t poly_size;
  mutable std::size_t lines_size;
  mutable std::size_t positions_lines_not_in_complex_size;
  mutable std::vector<float> positions_lines;
  mutable std::vector<float> positions_lines_not_in_complex;
  mutable std::vector<float> positions_grid;
  mutable std::size_t grid_size;
  mutable std::vector<float> positions_poly;

  mutable std::vector<float> normals;
  mutable std::vector<float> f_colors;
  mutable std::vector<float> s_normals;
  mutable std::vector<float> s_colors;
  mutable std::vector<float> s_vertex;
  mutable std::vector<float> ws_vertex;
  mutable std::vector<float> s_radius;
  mutable std::vector<float> s_center;
  mutable QOpenGLShaderProgram *program;
  mutable bool computed_stats;
  mutable float max_edges_length;
  mutable float min_edges_length;
  mutable float mean_edges_length;
  mutable float min_dihedral_angle;
  mutable float max_dihedral_angle;
  mutable float mean_dihedral_angle;
  mutable std::size_t nb_spheres;
  mutable std::size_t nb_cnc;
  mutable std::size_t nb_subdomains;
  mutable std::size_t nb_vertices;
  mutable std::size_t nb_tets;
  mutable float smallest_radius_radius;
  mutable float smallest_edge_radius;
  mutable float biggest_v_sma_cube;


  Tree tree;
  QVector<QColor> colors;
  QVector<QColor> colors_subdomains;
  bool show_tetrahedra;
  bool is_aabb_tree_built;
  bool cnc_are_shown;
  bool is_valid;
};

struct Set_show_tetrahedra {
  Scene_c3t3_item_priv* priv;
  Set_show_tetrahedra(Scene_c3t3_item_priv* priv) : priv(priv) {}
  void operator()(bool b) {
    priv->show_tetrahedra = b;
    priv->item->show_intersection(b);
  }
};

Scene_c3t3_item::Scene_c3t3_item(CGAL::Three::Scene_interface* scene)
  : Scene_group_item("unnamed", scene)
  , d(new Scene_c3t3_item_priv(this))

{
  compute_bbox();
  connect(d->frame, SIGNAL(modified()), this, SLOT(changed()));
  c3t3_changed();
  setRenderingMode(FlatPlusEdges);
  create_flat_and_wire_sphere(1.0f,d->s_vertex,d->s_normals, d->ws_vertex);
  setTriangleContainer(0,
        new Triangle_container(VI::PROGRAM_C3T3,
                                        false));
  setEdgeContainer(Edges,
                   new Edge_container(VI::PROGRAM_C3T3_EDGES,
                           false));

  setEdgeContainer(Grid,
        new Edge_container(VI::PROGRAM_WITHOUT_LIGHT,
                           false));
}

Scene_c3t3_item::Scene_c3t3_item(CGAL::Three::Scene_interface* scene, const C3t3& c3t3)
  : Scene_group_item("unnamed", scene)
  , d(new Scene_c3t3_item_priv(c3t3, this))
{
  compute_bbox();
  connect(d->frame, SIGNAL(modified()), this, SLOT(changed()));
  d->reset_cut_plane();
  c3t3_changed();
  setRenderingMode(FlatPlusEdges);
  create_flat_and_wire_sphere(1.0f,d->s_vertex,d->s_normals, d->ws_vertex);
  setTriangleContainer(0,
        new Triangle_container(VI::PROGRAM_C3T3,
                                        false));
  setEdgeContainer(Edges,
                   new Edge_container(VI::PROGRAM_C3T3_EDGES,
                           false));

  setEdgeContainer(Grid,
        new Edge_container(VI::PROGRAM_WITHOUT_LIGHT,
                           false));
}

Scene_c3t3_item::~Scene_c3t3_item()
{
  if(d)
  {
    delete d;
    d = NULL;
  }
}



const Scene_item*
Scene_c3t3_item::data_item() const
{
  return d->data_item_;
}

void
Scene_c3t3_item::set_data_item(const Scene_item* data_item)
{
  d->data_item_ = data_item;
  if (NULL != data_item)
  {
    connect(d->data_item_, SIGNAL(aboutToBeDestroyed()),
      this, SLOT(data_item_destroyed()));
  }
}

void
Scene_c3t3_item::data_item_destroyed()
{
  set_data_item(NULL);
}

const C3t3&
Scene_c3t3_item::c3t3() const {
  return d->c3t3;
}

C3t3&
Scene_c3t3_item::c3t3()
{
  return d->c3t3;
}

void
Scene_c3t3_item::changed()
{
  if(!d)
    return;
  d->need_changed = true;
  moveToThread(qApp->thread());
  QTimer::singleShot(0,this, SLOT(updateCutPlane()));
}

void Scene_c3t3_item::updateCutPlane()
{ // just handle deformation - paint like selection is handled in eventFilter()
  if(!d)
    return;
  if(d->need_changed) {
    if(d->intersection)
      d->intersection->setBuffersFilled(false);
    d->need_changed = false;
  }
}

void
Scene_c3t3_item::c3t3_changed()
{
  // Update colors
  // Fill indices map and get max subdomain value
  d->surface_patch_indices_.clear();
  d->subdomain_indices_.clear();

  int max = 0;
  for (C3t3::Cells_in_complex_iterator cit = this->c3t3().cells_in_complex_begin(),
    end = this->c3t3().cells_in_complex_end(); cit != end; ++cit)
  {
    max = (std::max)(max, cit->subdomain_index());
    d->subdomain_indices_.insert(cit->subdomain_index());
  }
  const int max_subdomain_index = max;
  for (C3t3::Facets_in_complex_iterator fit = this->c3t3().facets_in_complex_begin(),
    end = this->c3t3().facets_in_complex_end(); fit != end; ++fit)
  {
    max = (std::max)(max, fit->first->surface_patch_index(fit->second));
    d->surface_patch_indices_.insert(fit->first->surface_patch_index(fit->second));
  }

  d->colors.resize(max + 1);
  d->colors_subdomains.resize(max_subdomain_index + 1);
  d->compute_color_map(color());

  // Rebuild histogram
  build_histogram();

  d->tree.clear();
  d->is_aabb_tree_built = false;
}

QPixmap
Scene_c3t3_item::graphicalToolTip() const
{
  if (!d->histogram_.isNull())
  {
    return d->histogram_;
  }
  const_cast<Scene_c3t3_item&>(*this).build_histogram();
  return d->histogram_;
}

std::vector<int>
create_histogram(const C3t3& c3t3, double& min_value, double& max_value)
{
  Geom_traits::Compute_approximate_dihedral_angle_3 approx_dihedral_angle
    = c3t3.triangulation().geom_traits().compute_approximate_dihedral_angle_3_object();
  Geom_traits::Construct_point_3 wp2p
    = c3t3.triangulation().geom_traits().construct_point_3_object();

  std::vector<int> histo(181, 0);

  min_value = 180.;
  max_value = 0.;

  for (C3t3::Cells_in_complex_iterator cit = c3t3.cells_in_complex_begin();
    cit != c3t3.cells_in_complex_end();
    ++cit)
  {
    if (!c3t3.is_in_complex(cit))
      continue;

#ifdef CGAL_MESH_3_DEMO_DONT_COUNT_TETS_ADJACENT_TO_SHARP_FEATURES_FOR_HISTOGRAM
    if (c3t3.in_dimension(cit->vertex(0)) <= 1
      || c3t3.in_dimension(cit->vertex(1)) <= 1
      || c3t3.in_dimension(cit->vertex(2)) <= 1
      || c3t3.in_dimension(cit->vertex(3)) <= 1)
      continue;
#endif //CGAL_MESH_3_DEMO_DONT_COUNT_TETS_ADJACENT_TO_SHARP_FEATURES_FOR_HISTOGRAM

    const Tr::Bare_point& p0 = wp2p(cit->vertex(0)->point());
    const Tr::Bare_point& p1 = wp2p(cit->vertex(1)->point());
    const Tr::Bare_point& p2 = wp2p(cit->vertex(2)->point());
    const Tr::Bare_point& p3 = wp2p(cit->vertex(3)->point());

    double a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p0, p1, p2, p3)));
    histo[static_cast<int>(std::floor(a))] += 1;
    min_value = (std::min)(min_value, a);
    max_value = (std::max)(max_value, a);

    a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p0, p2, p1, p3)));
    histo[static_cast<int>(std::floor(a))] += 1;
    min_value = (std::min)(min_value, a);
    max_value = (std::max)(max_value, a);

    a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p0, p3, p1, p2)));
    histo[static_cast<int>(std::floor(a))] += 1;
    min_value = (std::min)(min_value, a);
    max_value = (std::max)(max_value, a);

    a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p1, p2, p0, p3)));
    histo[static_cast<int>(std::floor(a))] += 1;
    min_value = (std::min)(min_value, a);
    max_value = (std::max)(max_value, a);

    a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p1, p3, p0, p2)));
    histo[static_cast<int>(std::floor(a))] += 1;
    min_value = (std::min)(min_value, a);
    max_value = (std::max)(max_value, a);

    a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p2, p3, p0, p1)));
    histo[static_cast<int>(std::floor(a))] += 1;
    min_value = (std::min)(min_value, a);
    max_value = (std::max)(max_value, a);

  }

  return histo;
}

void
Scene_c3t3_item::build_histogram()
{
#ifdef CGAL_MESH_3_DEMO_BIGGER_HISTOGRAM_WITH_WHITE_BACKGROUNG
  // Create an histogram_ and display it
  const int height = 280;
  const int top_margin = 5;
  const int left_margin = 20;
  const int drawing_height = height - top_margin * 2;
  const int width = 804;
  const int cell_width = 4;
  const int text_margin = 3;
  const int text_height = 34;

  histogram_ = QPixmap(width, height + text_height);
  histogram_.fill(QColor(255, 255, 255));
#else
  // Create an histogram_ and display it
  const int height = 140;
  const int top_margin = 5;
  const int left_margin = 20;
  const int drawing_height = height - top_margin * 2;
  const int width = 402;
  const int cell_width = 2;
  const int text_margin = 3;
  const int text_height = 20;

  d->histogram_ = QPixmap(width, height + text_height);
  d->histogram_.fill(QColor(192, 192, 192));
#endif

  QPainter painter(&d->histogram_);
  painter.setPen(Qt::black);
  painter.setBrush(QColor(128, 128, 128));
  //painter.setFont(QFont("Arial", 30));

  // Build histogram_ data
  double min_value, max_value;
  std::vector<int> histo_data = create_histogram(c3t3(), min_value, max_value);

  // Get maximum value (to normalize)
  int max_size = 0;
  for (std::vector<int>::iterator it = histo_data.begin(), end = histo_data.end();
    it != end; ++it)
  {
    max_size = (std::max)(max_size, *it);
  }

  // colored histogram
  int j = 0;

  // draw
  int i = left_margin;
  for (std::vector<int>::iterator it = histo_data.begin(), end = histo_data.end();
    it != end; ++it, i += cell_width)
  {
    int line_height = static_cast<int>(std::ceil(static_cast<double>(drawing_height)*
      static_cast<double>(*it) / static_cast<double>(max_size)) + .5);

    painter.fillRect(i,
      drawing_height + top_margin - line_height,
      cell_width,
      line_height,
      get_histogram_color(j++));
  }

  // draw bottom horizontal line
  painter.setPen(Qt::blue);

  painter.drawLine(QPoint(left_margin, drawing_height + top_margin),
    QPoint(left_margin + static_cast<int>(histo_data.size())*cell_width,
    drawing_height + top_margin));


  // draw min value and max value
  const int min_tr_width = static_cast<int>(2 * (std::floor(min_value)*cell_width + left_margin));
  const int max_tr_width = static_cast<int>(
    2 * ((histo_data.size() - std::floor(max_value))*cell_width + left_margin));
  const int tr_y = drawing_height + top_margin + text_margin;

  painter.setPen(get_histogram_color(min_value));
  QRect min_text_rect(0, tr_y, min_tr_width, text_height);
  painter.drawText(min_text_rect, Qt::AlignCenter, tr("%1").arg(min_value, 0, 'f', 1));

  painter.setPen(get_histogram_color(max_value));
  QRect max_text_rect(width - max_tr_width, tr_y, max_tr_width, text_height);
  painter.drawText(max_text_rect, Qt::AlignCenter, tr("%1").arg(max_value, 0, 'f', 1));
}

QColor
Scene_c3t3_item::get_histogram_color(const double v) const
{
  if (v < 5)            { return Qt::red; }
  else if (v < 10)      { return QColor(215, 108, 0); }
  else if (v < 15)      { return QColor(138, 139, 0); }
  else if (v < 165)     { return QColor(60, 136, 64); }
  else if (v < 170)     { return QColor(138, 139, 1); }
  else if (v < 175)     { return QColor(215, 108, 0); }
  else /* 175<v<=180 */   { return Qt::red; }
}

void
Scene_c3t3_item::update_histogram()
{
  build_histogram();
}

void
Scene_c3t3_item_priv::compute_color_map(const QColor& c)
{
  typedef Indices::size_type size_type;

  const size_type nb_domains = subdomain_indices_.size();
  size_type i = 0;
  for (Indices::iterator it = subdomain_indices_.begin(),
         end = subdomain_indices_.end(); it != end; ++it, ++i)
  {
    double hue = c.hueF() + 1. / nb_domains * i;
    if (hue > 1) { hue -= 1.; }
    colors_subdomains[*it] = QColor::fromHsvF(hue, c.saturationF(), c.valueF());
  }
  const size_type nb_patch_indices = surface_patch_indices_.size();
  i = 0;
  for (Indices::iterator it = surface_patch_indices_.begin(),
         end = surface_patch_indices_.end(); it != end; ++it, ++i)
  {
    double hue = c.hueF() + 1. / nb_patch_indices * i;
    if (hue > 1) { hue -= 1.; }
    colors[*it] = QColor::fromHsvF(hue, c.saturationF(), c.valueF());
  }
}

Geom_traits::Plane_3 Scene_c3t3_item::plane(qglviewer::Vec offset) const
{
  const qglviewer::Vec& pos = d->frame->position() - offset;
  const qglviewer::Vec& n =
    d->frame->inverseTransformOf(qglviewer::Vec(0.f, 0.f, 1.f));
  return Geom_traits::Plane_3(n[0], n[1], n[2], -n * pos);
}

void Scene_c3t3_item::compute_bbox() const {
  Scene_c3t3_item* ncthis = const_cast<Scene_c3t3_item*>(this);
  if (isEmpty())
    ncthis->setBbox( Bbox());
  else {
    bool bbox_init = false;
    CGAL::Bbox_3 result;
    for (Tr::Finite_vertices_iterator
           vit = c3t3().triangulation().finite_vertices_begin(),
           end = c3t3().triangulation().finite_vertices_end();
         vit != end; ++vit)
    {
      if(vit->in_dimension() == -1) continue;
      if (bbox_init)
        result = result + vit->point().bbox();
      else
      {
        result = vit->point().bbox();
        bbox_init = true;
      }
    }
    ncthis->setBbox(Bbox(result.xmin(), result.ymin(), result.zmin(),
                 result.xmax(), result.ymax(), result.zmax()));
  }
}

QString Scene_c3t3_item::toolTip() const {
  return tr("<p><b>3D complex in a 3D triangulation</b></p>"
    "<p>Number of vertices: %1<br />"
    "Number of surface facets: %2<br />"
    "Number of volume tetrahedra: %3</p>%4")
    .arg(c3t3().triangulation().number_of_vertices())
    .arg(c3t3().number_of_facets_in_complex())
    .arg(c3t3().number_of_cells_in_complex())
    .arg(property("toolTip").toString());
}

void Scene_c3t3_item::draw(Viewer_interface *viewer , int pass, bool is_writing, QOpenGLFramebufferObject *fbo){
  if(!(visible() &&
     (renderingMode() == Flat ||
      renderingMode() == FlatPlusEdges ||
      renderingMode() == Gouraud)))
    return;

  if(!isWriting() && !isInit())
  {
    initGL();
    if(!d->tet_Slider)
    {
      d->tet_Slider = new QSlider(Qt::Horizontal);
      d->tet_Slider->setMinimum(0);
      d->tet_Slider->setMaximum(100);
      d->tet_Slider->setValue(100);
    }
  }
  if (!isWriting() && getBuffersFilled() &&
     ! getBuffersInit(viewer))
  {
    initializeBuffers(viewer);
    setBuffersInit(viewer, true);
  }

  float near(viewer->camera()->zNear()), far(viewer->camera()->zFar());
  QVector4D cp(this->plane().a(),this->plane().b(),this->plane().c(),this->plane().d());
  getTriangleContainer(0)->setPlane(cp);
  getTriangleContainer(0)->setShrinkFactor(getShrinkFactor());  
  getTriangleContainer(0)->setComparing(pass > 0);
  getTriangleContainer(0)->setWidth(viewer->width()*1.0f);
  getTriangleContainer(0)->setHeight(viewer->height()*1.0f);
  getTriangleContainer(0)->setNear(near);
  getTriangleContainer(0)->setFar(far);
  getTriangleContainer(0)->setDepthWriting(is_writing);
  getTriangleContainer(0)->setColor(isSelected() ? selectionColor() : color());
  getTriangleContainer(0)->setSelected(isSelected());
  getTriangleContainer(0)->setAlpha(alpha());
  getTriangleContainer(0)->draw(viewer, false, fbo);

  if(d->show_tetrahedra){
      if(!d->frame->isManipulated())
      {
        d->intersection->setFast(false);
        redraw();
      }
      else
        d->intersection->setFast(true);

      if(!d->frame->isManipulated() && !d->intersection->getBuffersFilled())
      {
        d->computeIntersections();
        d->intersection->initializeBuffers(viewer);;
      }
    }

    if(d->spheres_are_shown)
    {
      d->spheres->setPlane(this->plane());
    }

    if(d->is_grid_shown)
    {
      QMatrix4x4 f_mat;
      for (int i = 0; i<16; i++)
        f_mat.data()[i] = d->frame->matrix()[i];
      getEdgeContainer(Grid)->setFrameMatrix(f_mat);
      getEdgeContainer(Grid)->setColor(QColor(Qt::black));
      getEdgeContainer(Grid)->draw(viewer, true);
    }
}

void Scene_c3t3_item::drawEdges(CGAL::Three::Viewer_interface* viewer) {
  if(!(visible() &&
      (renderingMode() == FlatPlusEdges
       || renderingMode() == Wireframe
       || renderingMode() == PointsPlusNormals)))
    return;


  if(!isWriting() && !isInit())
  {
    GLint renderMode;
    viewer->glGetIntegerv(GL_RENDER_MODE, &renderMode);
    if(renderMode == GL_SELECT) return;
    initGL();
    if(!d->tet_Slider)
    {
      d->tet_Slider = new QSlider(Qt::Horizontal);
      d->tet_Slider->setMinimum(0);
      d->tet_Slider->setMaximum(100);
      d->tet_Slider->setValue(100);
    }
  }
  if (!isWriting() && getBuffersFilled() &&
     ! getBuffersInit(viewer))
  {
    initializeBuffers(viewer);
    setBuffersInit(viewer, true);
  }

  if(renderingMode() == Wireframe && d->is_grid_shown)
  {
    QMatrix4x4 f_mat;
    for (int i = 0; i<16; i++)
      f_mat.data()[i] = d->frame->matrix()[i];
    getEdgeContainer(Grid)->setFrameMatrix(f_mat);
    getEdgeContainer(Grid)->setColor(QColor(Qt::black));
    getEdgeContainer(Grid)->draw( viewer, true);
  }
  getEdgeContainer(Edges)->setPlane(QVector4D(this->plane().a(),this->plane().b(),this->plane().c(),this->plane().d()));
  getEdgeContainer(Edges)->setColor(QColor(Qt::black));
  getEdgeContainer(Edges)->draw(viewer, true);

  if(d->show_tetrahedra){
    if(!d->frame->isManipulated())
      d->intersection->setFast(false);
    else
      d->intersection->setFast(true);
    if(!d->frame->isManipulated() && !d->intersection->getBuffersFilled())
    {
      d->computeIntersections();
      d->intersection->initializeBuffers(viewer);
      d->intersection->setBuffersFilled(true);
    }
  }
  if(d->spheres_are_shown)
  {
      d->spheres->setPlane(this->plane());
  }
  Scene_group_item::drawEdges(viewer);
  //if(d->cnc_are_shown)
  //{
  //  vaos[Scene_c3t3_item_priv::CNC]->bind();
  //  d->program = getShaderProgram(PROGRAM_NO_SELECTION);
  //  attribBuffers(viewer, PROGRAM_NO_SELECTION);
  //  d->program->bind();
  //  d->program->setAttributeValue("colors", QColor(Qt::black));
  //  viewer->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(d->positions_lines_not_in_complex_size / 3));
  //  d->program->release();
  //  vaos[Scene_c3t3_item_priv::CNC]->release();
  //}

}

void Scene_c3t3_item::drawPoints(CGAL::Three::Viewer_interface * viewer)
{
  if(!(visible() &&
       (renderingMode() == Points  ||
        (renderingMode() == PointsPlusNormals)  ||
        (renderingMode() == ShadedPoints))))
    return;
//  Scene_c3t3_item* ncthis = const_cast<Scene_c3t3_item*>(this);
//  if (!are_buffers_filled)
//  {
//    ncthis->d->computeElements();
//    ncthis->d->initializeBuffers(viewer);
//  }
//  vaos[Scene_c3t3_item_priv::Edges]->bind();
//  d->program = getShaderProgram(PROGRAM_C3T3_EDGES);
//  attribBuffers(viewer, PROGRAM_C3T3_EDGES);
//  d->program->bind();
//  QVector4D cp(this->plane().a(),this->plane().b(),this->plane().c(),this->plane().d());
//  d->program->setUniformValue("cutplane", cp);
//  d->program->setAttributeValue("colors", this->color());
//  viewer->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(d->positions_lines.size() / 3));
//  vaos[Scene_c3t3_item_priv::Edges]->release();
//  d->program->release();

//  if(d->is_grid_shown)
//  {
//    vaos[Scene_c3t3_item_priv::Grid]->bind();
//    d->program = getShaderProgram(PROGRAM_NO_SELECTION);
//    attribBuffers(viewer, PROGRAM_NO_SELECTION);
//    d->program->bind();
//    d->program->setAttributeValue("colors", this->color());
//    QMatrix4x4 f_mat;
//    for (int i = 0; i<16; i++)
//      f_mat.data()[i] = d->frame->matrix()[i];
//    d->program->setUniformValue("f_matrix", f_mat);
//    viewer->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(d->positions_grid.size() / 3));
//    d->program->release();
//    vaos[Scene_c3t3_item_priv::Grid]->release();
//  }
//  if(d->spheres_are_shown)
//  {
//    d->spheres->setPlane(this->plane());
//  }
//  Scene_group_item::drawEdges(viewer);

}



void Scene_c3t3_item_priv::draw_triangle_edges_cnc(const Tr::Bare_point& pa,
                                                   const Tr::Bare_point& pb,
                                                   const Tr::Bare_point& pc) const
{
#undef darker
  const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
  positions_lines_not_in_complex.push_back(pa.x()+offset.x);
  positions_lines_not_in_complex.push_back(pa.y()+offset.y);
  positions_lines_not_in_complex.push_back(pa.z()+offset.z);

  positions_lines_not_in_complex.push_back(pb.x()+offset.x);
  positions_lines_not_in_complex.push_back(pb.y()+offset.y);
  positions_lines_not_in_complex.push_back(pb.z()+offset.z);

  positions_lines_not_in_complex.push_back(pb.x()+offset.x);
  positions_lines_not_in_complex.push_back(pb.y()+offset.y);
  positions_lines_not_in_complex.push_back(pb.z()+offset.z);

  positions_lines_not_in_complex.push_back(pc.x()+offset.x);
  positions_lines_not_in_complex.push_back(pc.y()+offset.y);
  positions_lines_not_in_complex.push_back(pc.z()+offset.z);

  positions_lines_not_in_complex.push_back(pc.x()+offset.x);
  positions_lines_not_in_complex.push_back(pc.y()+offset.y);
  positions_lines_not_in_complex.push_back(pc.z()+offset.z);

  positions_lines_not_in_complex.push_back(pa.x()+offset.x);
  positions_lines_not_in_complex.push_back(pa.y()+offset.y);
  positions_lines_not_in_complex.push_back(pa.z()+offset.z);

}

double Scene_c3t3_item_priv::complex_diag() const {
  const CGAL::Three::Scene_item::Bbox& bbox = item->bbox();
  const double& xdelta = bbox.xmax() - bbox.xmin();
  const double& ydelta = bbox.ymax() - bbox.ymin();
  const double& zdelta = bbox.zmax() - bbox.zmin();
  const double diag = std::sqrt(xdelta*xdelta +
    ydelta*ydelta +
    zdelta*zdelta);
  return diag * 0.7;
}

void Scene_c3t3_item::export_facets_in_complex()
{
  std::set<C3t3::Vertex_handle> vertex_set;
  for (C3t3::Facets_in_complex_iterator fit = c3t3().facets_in_complex_begin();
       fit != c3t3().facets_in_complex_end();
       ++fit)
  {
    vertex_set.insert(fit->first->vertex((fit->second + 1) % 4));
    vertex_set.insert(fit->first->vertex((fit->second + 2) % 4));
    vertex_set.insert(fit->first->vertex((fit->second + 3) % 4));
  }

  std::map<C3t3::Vertex_handle, std::size_t> indices;
  std::vector<Tr::Bare_point> points(vertex_set.size());
  std::vector<std::vector<std::size_t> > polygons(c3t3().number_of_facets_in_complex());

  std::size_t index = 0;
  Geom_traits::Construct_point_3 wp2p
    = c3t3().triangulation().geom_traits().construct_point_3_object();

  BOOST_FOREACH(C3t3::Vertex_handle v, vertex_set)
  {
    points[index] = wp2p(v->point());
    indices.insert(std::make_pair(v, index));
    index++;
  }

  index = 0;
  for (C3t3::Facets_in_complex_iterator fit = c3t3().facets_in_complex_begin();
       fit != c3t3().facets_in_complex_end();
       ++fit, ++index)
  {
    std::vector<std::size_t> facet(3);
    facet[0] = indices.at(fit->first->vertex((fit->second + 1) % 4));
    facet[1] = indices.at(fit->first->vertex((fit->second + 2) % 4));
    facet[2] = indices.at(fit->first->vertex((fit->second + 3) % 4));
    polygons[index] = facet;
  }

  namespace PMP = CGAL::Polygon_mesh_processing;
  Polyhedron outmesh;

  if (PMP::is_polygon_soup_a_polygon_mesh(polygons))
  {
    CGAL_assertion_code(bool orientable = )
    PMP::orient_polygon_soup(points, polygons);
    CGAL_assertion(orientable);

    PMP::polygon_soup_to_polygon_mesh(points, polygons, outmesh);
    Scene_polyhedron_item* item = new Scene_polyhedron_item(outmesh);
    item->setName(QString("%1_%2").arg(this->name()).arg("facets"));
    scene->addItem(item);
  }
  else
  {
    Scene_polygon_soup_item* soup_item = new Scene_polygon_soup_item;
    soup_item->load(points, polygons);
    soup_item->setName(QString("%1_%2").arg(this->name()).arg("facets"));
    scene->addItem(soup_item);
  }
  this->setVisible(false);
}

QMenu* Scene_c3t3_item::contextMenu()
{
  const char* prop_name = "Menu modified by Scene_c3t3_item.";

  QMenu* menu = Scene_item_rendering_helper::contextMenu();

  // Use dynamic properties:
  // http://doc.qt.io/qt-5/qobject.html#property
  bool menuChanged = menu->property(prop_name).toBool();

  if (!menuChanged) {

    QMenu *container = new QMenu(tr("Tetrahedra's Shrink Factor"));
    QWidgetAction *sliderAction = new QWidgetAction(0);
    connect(d->tet_Slider, &QSlider::valueChanged, this, &Scene_c3t3_item::itemChanged);

    sliderAction->setDefaultWidget(d->tet_Slider);
    container->addAction(sliderAction);
    menu->addMenu(container);
    QAction* actionExportFacetsInComplex =
      menu->addAction(tr("Export facets in complex"));
    actionExportFacetsInComplex->setObjectName("actionExportFacetsInComplex");
    connect(actionExportFacetsInComplex,
      SIGNAL(triggered()), this,
      SLOT(export_facets_in_complex()));

    if(is_valid())
    {
      QAction* actionShowSpheres =
          menu->addAction(tr("Show protecting &spheres"));
      actionShowSpheres->setCheckable(true);
      actionShowSpheres->setObjectName("actionShowSpheres");
      connect(actionShowSpheres, SIGNAL(toggled(bool)),
              this, SLOT(show_spheres(bool)));

      QAction* actionShowCNC =
          menu->addAction(tr("Show cells not in complex"));
      actionShowCNC->setCheckable(true);
      actionShowCNC->setObjectName("actionShowCNC");
      connect(actionShowCNC, SIGNAL(toggled(bool)),
              this, SLOT(show_cnc(bool)));
    }
    QAction* actionShowTets =
      menu->addAction(tr("Show &tetrahedra"));
    actionShowTets->setCheckable(true);
    actionShowTets->setObjectName("actionShowTets");
    connect(actionShowTets, SIGNAL(toggled(bool)),
            this, SLOT(show_intersection(bool)));

    QAction* actionShowGrid=
      menu->addAction(tr("Show &grid"));
    actionShowGrid->setCheckable(true);
    actionShowGrid->setChecked(true);
    actionShowGrid->setObjectName("actionShowGrid");
    connect(actionShowGrid, SIGNAL(toggled(bool)),
            this, SLOT(show_grid(bool)));

    menu->setProperty(prop_name, true);
  }
  return menu;
}


void Scene_c3t3_item::initializeBuffers(CGAL::Three::Viewer_interface *viewer)
{
  getTriangleContainer(0)->initializeBuffers(viewer);
  getTriangleContainer(0)->setFlatDataSize(d->poly_size);
  getEdgeContainer(Grid)->initializeBuffers(viewer);
  getEdgeContainer(Grid)->setFlatDataSize(d->grid_size);
  getEdgeContainer(Edges)->initializeBuffers(viewer);
  getEdgeContainer(Edges)->setFlatDataSize(d->lines_size);

  d->positions_poly.clear();
  d->positions_poly.shrink_to_fit();
  d->normals.clear();
  d->normals.shrink_to_fit();
  d->f_colors.clear();
  d->f_colors.shrink_to_fit();
}



void Scene_c3t3_item_priv::computeIntersection(const Primitive& facet)
{
  Geom_traits::Construct_point_3 wp2p
    = c3t3.triangulation().geom_traits().construct_point_3_object();

  Tr::Cell_handle ch = facet.id().first;
  if(intersected_cells.find(ch) == intersected_cells.end())
  {
    QColor c = this->colors_subdomains[ch->subdomain_index()].light(50);

    const Tr::Bare_point& pa = wp2p(ch->vertex(0)->point());
    const Tr::Bare_point& pb = wp2p(ch->vertex(1)->point());
    const Tr::Bare_point& pc = wp2p(ch->vertex(2)->point());
    const Tr::Bare_point& pd = wp2p(ch->vertex(3)->point());

    CGAL::Color color(c.red(), c.green(), c.blue());

    intersection->addTriangle(pb, pa, pc, color);
    intersection->addTriangle(pa, pb, pd, color);
    intersection->addTriangle(pa, pd, pc, color);
    intersection->addTriangle(pb, pc, pd, color);
    intersected_cells.insert(ch);
  }
  {
    Tr::Cell_handle nh = ch->neighbor(facet.id().second);
    if(c3t3.is_in_complex(nh)){
      if(intersected_cells.find(nh) == intersected_cells.end())
      {
        const Tr::Bare_point& pa = wp2p(nh->vertex(0)->point());
        const Tr::Bare_point& pb = wp2p(nh->vertex(1)->point());
        const Tr::Bare_point& pc = wp2p(nh->vertex(2)->point());
        const Tr::Bare_point& pd = wp2p(nh->vertex(3)->point());

        QColor c = this->colors_subdomains[nh->subdomain_index()].light(50);

        CGAL::Color color(c.red(), c.green(), c.blue());

        intersection->addTriangle(pb, pa, pc, color);
        intersection->addTriangle(pa, pb, pd, color);
        intersection->addTriangle(pa, pd, pc, color);
        intersection->addTriangle(pb, pc, pd, color);
        intersected_cells.insert(nh);
      }
    }
  }
  intersection->setBuffersFilled(true);
}

struct ComputeIntersection {
  Scene_c3t3_item_priv& item_priv;

  ComputeIntersection(Scene_c3t3_item_priv& item_priv)
    : item_priv(item_priv)
  {}

  void operator()(const Primitive& facet) const
  {
    item_priv.computeIntersection(facet);
  }
};

void Scene_c3t3_item_priv::computeIntersections()
{
  const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
  if(!is_aabb_tree_built) fill_aabb_tree();

  positions_poly.clear();
  normals.clear();
  f_colors.clear();
  positions_lines.clear();
  const Geom_traits::Plane_3& plane = item->plane(offset);
  tree.all_intersected_primitives(plane,
        boost::make_function_output_iterator(ComputeIntersection(*this)));
  intersection->allocate();
  intersected_cells.clear();
}

void Scene_c3t3_item_priv::computeSpheres()
{
  Geom_traits::Construct_point_3 wp2p
    = c3t3.triangulation().geom_traits().construct_point_3_object();

  if(!spheres)
    return;

  for(Tr::Finite_vertices_iterator
      vit = c3t3.triangulation().finite_vertices_begin(),
      end =  c3t3.triangulation().finite_vertices_end();
      vit != end; ++vit)
  {
    if(vit->point().weight()==0) continue;

    typedef Tr::Vertex_handle Vertex_handle;
    std::vector<Vertex_handle> incident_vertices;
    c3t3.triangulation().incident_vertices(vit, std::back_inserter(incident_vertices));
    bool red = vit->is_special();
    for(std::vector<Vertex_handle>::const_iterator
        vvit = incident_vertices.begin(), end = incident_vertices.end();
        vvit != end; ++vvit)
    {
      if(c3t3.triangulation().is_infinite(*vvit)) continue;
      if(Geom_traits::Sphere_3(wp2p(vit->point()),
                               vit->point().weight()).bounded_side(wp2p((*vvit)->point()))
         == CGAL::ON_BOUNDED_SIDE)
        red = true;
    }

    QColor c;
    if(red)
      c = QColor(Qt::red);
    else
      c = spheres->color();

    switch(vit->in_dimension())
    {
    case 0:
      c = QColor::fromHsv((c.hue()+120)%360, c.saturation(),c.lightness(), c.alpha());
      break;
    case 1:
      break;
    default:
      c.setRgb(50,50,50,255);
    }

    const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
    Tr::Bare_point center(wp2p(vit->point()).x() + offset.x,
                          wp2p(vit->point()).y() + offset.y,
                          wp2p(vit->point()).z() + offset.z);
    float radius = vit->point().weight() ;
    spheres->add_sphere(Geom_traits::Sphere_3(center, radius),
                        CGAL::Color(c.red(), c.green(), c.blue()));
  }
  spheres->invalidate(Scene_item::ALL);
  spheres->computeElements(Scene_item::ALL);
}

void Scene_c3t3_item_priv::computeElements(Scene_c3t3_item::Gl_data_names name)
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  positions_poly.clear();
  normals.clear();
  f_colors.clear();
  positions_lines.clear();
  positions_lines_not_in_complex.clear();
  s_colors.resize(0);
  s_center.resize(0);
  s_radius.resize(0);

  //The grid
  {

    float x = (2 * (float)complex_diag()) / 10.0;
    float y = (2 * (float)complex_diag()) / 10.0;
    for (int u = 0; u < 11; u++)
    {

      positions_grid.push_back(-(float)complex_diag() + x* u);
      positions_grid.push_back(-(float)complex_diag());
      positions_grid.push_back(0.0);

      positions_grid.push_back(-(float)complex_diag() + x* u);
      positions_grid.push_back((float)complex_diag());
      positions_grid.push_back(0.0);
    }
    for (int v = 0; v<11; v++)
    {

      positions_grid.push_back(-(float)complex_diag());
      positions_grid.push_back(-(float)complex_diag() + v * y);
      positions_grid.push_back(0.0);

      positions_grid.push_back((float)complex_diag());
      positions_grid.push_back(-(float)complex_diag() + v * y);
      positions_grid.push_back(0.0);
    }
  }

  const qglviewer::Vec o= static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
  Geom_traits::Vector_3 offset(o.x, o.y, o.z);
  //The facets
  {
    Geom_traits::Construct_point_3 wp2p
      = c3t3.triangulation().geom_traits().construct_point_3_object();
    CGAL::Bbox_3 bbox;
    Buffer_for_vao<float, unsigned int> face_filler(
          &positions_poly,  //pos
          NULL,              //indices
          &bbox,   //bbox
          &f_colors,          //color
          &normals,              //flat normal
          NULL);        //gouraud normal
    Buffer_for_vao<float, unsigned int> edge_filler(
          &positions_lines,  //pos
          NULL,              //indices
          NULL,   //bbox
          NULL,          //color
          NULL,              //flat normal
          NULL);        //gouraud normal

    Buffer_for_vao<float, unsigned int> cnc_filler(
          &positions_lines_not_in_complex,  //pos
          NULL,              //indices
          NULL,   //bbox
          NULL,          //color
          NULL,              //flat normal
          NULL);        //gouraud normal

    for (C3t3::Facet_iterator
      fit = c3t3.facets_begin(),
      end = c3t3.facets_end();
    fit != end; ++fit)
    {
      const Tr::Cell_handle& cell = fit->first;
      const int& index = fit->second;
      const Tr::Bare_point& pa = wp2p(cell->vertex((index + 1) & 3)->point()) + offset;
      const Tr::Bare_point& pb = wp2p(cell->vertex((index + 2) & 3)->point()) + offset;
      const Tr::Bare_point& pc = wp2p(cell->vertex((index + 3) & 3)->point()) + offset;
      QColor c= colors[cell->surface_patch_index(index)];
      CGAL::Color color(c.red(), c.green(), c.blue());
      Geom_traits::Vector_3 n = cross_product(pb - pa, pc - pa);
      n = n / CGAL::sqrt(n*n);
      if ((index % 2 == 1) == c3t3.is_in_complex(cell))
      {
        face_filler.face_begin(color, -n);
      }
      else
      {
        face_filler.face_begin(color, n);
      }
      face_filler.add_point_in_face(pa);
      face_filler.add_point_in_face(pb);
      face_filler.add_point_in_face(pc);
      face_filler.face_end();
      edge_filler.add_segment(pa, pb);
      edge_filler.add_segment(pb, pc);
      edge_filler.add_segment(pc, pa);
    }
    //the cells not in the complex
    for(C3t3::Triangulation::Cell_iterator
        cit = c3t3.triangulation().finite_cells_begin(),
        end = c3t3.triangulation().finite_cells_end();
        cit != end; ++cit)
    {
      if(!c3t3.is_in_complex(cit))
      {

        bool has_far_point = false;
        for(int i=0; i<4; i++)
          if(c3t3.in_dimension(cit->vertex(i)) == -1)
          {
            has_far_point = true;
            break;
          }
        if(!has_far_point)
        {
          const Tr::Bare_point& p1 = wp2p(cit->vertex(0)->point()) + offset;
          const Tr::Bare_point& p2 = wp2p(cit->vertex(1)->point()) + offset;
          const Tr::Bare_point& p3 = wp2p(cit->vertex(2)->point()) + offset;
          const Tr::Bare_point& p4 = wp2p(cit->vertex(3)->point()) + offset;
          cnc_filler.add_segment(p1, p2);
          cnc_filler.add_segment(p2, p4);
          cnc_filler.add_segment(p4, p1);

          cnc_filler.add_segment(p1, p3);
          cnc_filler.add_segment(p3, p4);
          cnc_filler.add_segment(p2, p3);
        }
      }
    }
  }

  item->getTriangleContainer(0)->allocate(Tri::Flat_vertices, positions_poly.data(),
                                          static_cast<int>(positions_poly.size()*sizeof(float)));
  item->getTriangleContainer(0)->allocate(Tri::Flat_normals, normals.data(),
                                          static_cast<int>(normals.size()*sizeof(float)));
  item->getTriangleContainer(0)->allocate(Tri::FColors, f_colors.data(),
                                          static_cast<int>(f_colors.size()*sizeof(float)));
  item->getEdgeContainer(Scene_c3t3_item::Grid)->allocate(Ed::Vertices, positions_grid.data(),
                                                          static_cast<int>(positions_grid.size()*sizeof(float)));
  item->getEdgeContainer(Scene_c3t3_item::Edges)->allocate(Ed::Vertices, positions_lines.data(),
                                                           static_cast<int>(positions_lines.size()*sizeof(float)));
  poly_size = positions_poly.size();
  lines_size = positions_lines.size();
  grid_size = positions_grid.size();

  QApplication::restoreOverrideCursor();
}

void Scene_c3t3_item::computeElements(Gl_data_names name)
{
  getTriangleContainer(0)->reset_vbos(name);
  getEdgeContainer(Grid)->reset_vbos(name);
  getEdgeContainer(Edges)->reset_vbos(name);
  d->computeElements(name);
  setBuffersFilled(true);
}
bool Scene_c3t3_item::load_binary(std::istream& is)
{
  if(!CGAL::Mesh_3::load_binary_file(is, c3t3())) return false;
  if(is && d->frame == 0) {
    d->frame = new qglviewer::ManipulatedFrame();
  }
  d->reset_cut_plane();
  if(is.good()) {
    c3t3_changed();
    changed();
    return true;
  }
  else
    return false;
}

void
Scene_c3t3_item_priv::reset_cut_plane() {
  const CGAL::Three::Scene_item::Bbox& bbox = item->bbox();
  const float xcenter = static_cast<float>((bbox.xmax()+bbox.xmin())/2.);
  const float ycenter = static_cast<float>((bbox.ymax()+bbox.ymin())/2.);
  const float zcenter = static_cast<float>((bbox.zmax()+bbox.zmin())/2.);
 const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
  frame->setPosition(qglviewer::Vec(xcenter+offset.x, ycenter+offset.y, zcenter+offset.z));
}

void
Scene_c3t3_item::setColor(QColor c)
{
  Scene_item_rendering_helper::setColor(c);
  d->compute_color_map(c);
  invalidate(COLORS);
  d->invalidate_stats();
  if(d->intersection)
    d->intersection->setBuffersFilled(false);
}

void Scene_c3t3_item::show_grid(bool b)
{
  d->is_grid_shown = b;
  contextMenu()->findChild<QAction*>("actionShowGrid")->setChecked(b);
  itemChanged();
}
void Scene_c3t3_item::show_spheres(bool b)
{
  if(is_valid())
  {
    d->spheres_are_shown = b;
    contextMenu()->findChild<QAction*>("actionShowSpheres")->setChecked(b);
    if(b && !d->spheres)
    {
      d->spheres = new Scene_spheres_item(this, true);
      d->spheres->setName("Protecting spheres");
      d->spheres->setRenderingMode(Gouraud);
      connect(d->spheres, SIGNAL(destroyed()), this, SLOT(reset_spheres()));
      connect(d->spheres, SIGNAL(on_color_changed()), this, SLOT(on_spheres_color_changed()));
      scene->addItem(d->spheres);
      scene->changeGroup(d->spheres, this);
      lockChild(d->spheres);
      d->computeSpheres();
    }
    else if (!b && d->spheres!=NULL)
    {
      unlockChild(d->spheres);
      scene->erase(scene->item_id(d->spheres));
    }
    Q_EMIT redraw();
  }

}
void Scene_c3t3_item::show_intersection(bool b)
{
  d->show_tetrahedra = b;
  contextMenu()->findChild<QAction*>("actionShowTets")->setChecked(b);
  if(b && !d->intersection)
  {
    d->intersection = new Scene_intersection_item(this);
    scene->addItem(d->intersection);
    scene->changeGroup(d->intersection, this);
    lockChild(d->intersection);
    d->intersection->init_vectors(&d->positions_poly,
                                  &d->normals,
                                  &d->positions_lines,
                                  &d->f_colors);
    d->intersection->setName("Intersection tetrahedra");
    d->intersection->setRenderingMode(renderingMode());
    connect(d->intersection, SIGNAL(destroyed()), this, SLOT(reset_intersection_item()));
    d->intersection->setBuffersFilled(false);
  }
  else if (!b && d->intersection!=NULL)
  {
    unlockChild(d->intersection);
    scene->erase(scene->item_id(d->intersection));
  }
  Q_EMIT redraw();

}
void Scene_c3t3_item::show_cnc(bool b)
{
  if(is_valid())
  {
    d->cnc_are_shown = b;
    contextMenu()->findChild<QAction*>("actionShowCNC")->setChecked(b);
    Q_EMIT redraw();
  }
}

void Scene_c3t3_item::reset_intersection_item()
{
  d->intersection = NULL;
}

void Scene_c3t3_item::reset_spheres()
{
  d->spheres = NULL;
}
CGAL::Three::Scene_item::ManipulatedFrame* Scene_c3t3_item::manipulatedFrame() {
  if(d)
    return d->frame;
  else
    return NULL;
}

void Scene_c3t3_item::setPosition(float x, float y, float z) {
   const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
  d->frame->setPosition(x+offset.x, y+offset.y, z+offset.z);
}

bool Scene_c3t3_item::has_spheres()const { return d->spheres_are_shown;}

bool Scene_c3t3_item::has_grid()const { return d->is_grid_shown;}

bool Scene_c3t3_item::has_cnc()const { return d->cnc_are_shown;}

bool Scene_c3t3_item::has_tets()const { return d->intersection; }

void Scene_c3t3_item::setNormal(float x, float y, float z) {
  d->frame->setOrientation(x, y, z, 0.f);
}

void Scene_c3t3_item::copyProperties(Scene_item *item)
{
  Scene_c3t3_item* c3t3_item = qobject_cast<Scene_c3t3_item*>(item);
  if(!c3t3_item)
    return;
   const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
  d->frame->setPositionAndOrientation(c3t3_item->manipulatedFrame()->position() - offset,
                                      c3t3_item->manipulatedFrame()->orientation());

  show_intersection(c3t3_item->has_tets());

  show_spheres(c3t3_item->has_spheres());

  show_cnc(c3t3_item->has_cnc());

  show_grid(c3t3_item->has_grid());
}

bool Scene_c3t3_item::is_valid() const
{
  return d->is_valid;
}
void Scene_c3t3_item::set_valid(bool b)
{
  d->is_valid = b;
}
float Scene_c3t3_item::getShrinkFactor() const
{
 return d->tet_Slider->value()/100.0f;
}
bool Scene_c3t3_item::keyPressEvent(QKeyEvent *event)
{
 if(event->key() == Qt::Key_Plus)
 {
   d->tet_Slider->setValue(d->tet_Slider->value() + 5);
   itemChanged();
 }
 else if(event->key() == Qt::Key_Minus)
 {
   d->tet_Slider->setValue(d->tet_Slider->value() -5);
   itemChanged();
 }
 return true;
}

QString Scene_c3t3_item::computeStats(int type)
{
  Geom_traits::Construct_point_3 wp2p
    = d->c3t3.triangulation().geom_traits().construct_point_3_object();

  if(!d->computed_stats)
  {
    float nb_edges = 0;
    float total_edges = 0;
    float nb_angle = 0;
    float total_angle = 0;

    for (C3t3::Facet_iterator
      fit = d->c3t3.facets_begin(),
      end = d->c3t3.facets_end();
    fit != end; ++fit)
    {
      const Tr::Cell_handle& cell = fit->first;
      const int& index = fit->second;
      const Tr::Bare_point& pa = wp2p(cell->vertex((index + 1) & 3)->point());
      const Tr::Bare_point& pb = wp2p(cell->vertex((index + 2) & 3)->point());
      const Tr::Bare_point& pc = wp2p(cell->vertex((index + 3) & 3)->point());
      float edges[3];
      edges[0]=(std::sqrt(CGAL::squared_distance(pa, pb)));
      edges[1]=(std::sqrt(CGAL::squared_distance(pa, pc)));
      edges[2]=(std::sqrt(CGAL::squared_distance(pb, pc)));
      for(int i=0; i<3; ++i)
      {
        if(edges[i] < d->min_edges_length){ d->min_edges_length = edges[i]; }
        if(edges[i] > d->max_edges_length){ d->max_edges_length = edges[i]; }
        total_edges+=edges[i];
        ++nb_edges;
      }
    }
    d->mean_edges_length = total_edges/(float)nb_edges;
    for(Tr::Finite_vertices_iterator
        vit = d->c3t3.triangulation().finite_vertices_begin(),
        end =  d->c3t3.triangulation().finite_vertices_end();
        vit != end; ++vit)
    {
      if(vit->point().weight()==0) continue;
      ++d->nb_spheres;
    }
    for(C3t3::Triangulation::Cell_iterator
        cit = d->c3t3.triangulation().finite_cells_begin(),
        end = d->c3t3.triangulation().finite_cells_end();
        cit != end; ++cit)
    {
      if(!d->c3t3.is_in_complex(cit))
      {

        bool has_far_point = false;
        for(int i=0; i<4; i++)
          if(d->c3t3.in_dimension(cit->vertex(i)) == -1)
          {
            has_far_point = true;
            break;
          }
        if(!has_far_point)
          ++d->nb_cnc;
      }
    }

    Geom_traits::Compute_approximate_dihedral_angle_3 approx_dihedral_angle
      = d->c3t3.triangulation().geom_traits().compute_approximate_dihedral_angle_3_object();

    QVector<int> sub_ids;
    for (C3t3::Cells_in_complex_iterator cit = d->c3t3.cells_in_complex_begin();
      cit != d->c3t3.cells_in_complex_end();
      ++cit)
    {
      if (!d->c3t3.is_in_complex(cit))
        continue;
      if(!sub_ids.contains(cit->subdomain_index()))
      {
        sub_ids.push_back(cit->subdomain_index());
      }

      const Tr::Bare_point& p0 = wp2p(cit->vertex(0)->point());
      const Tr::Bare_point& p1 = wp2p(cit->vertex(1)->point());
      const Tr::Bare_point& p2 = wp2p(cit->vertex(2)->point());
      const Tr::Bare_point& p3 = wp2p(cit->vertex(3)->point());
      float v = std::abs(CGAL::volume(p0, p1, p2, p3));
      float circumradius = std::sqrt(CGAL::squared_radius(p0, p1, p2, p3));
      //find smallest edge
      float edges[6];
      edges[0] = std::sqrt(CGAL::squared_distance(p0, p1));
      edges[1] = std::sqrt(CGAL::squared_distance(p0, p2));
      edges[2] = std::sqrt(CGAL::squared_distance(p0, p3));
      edges[3] = std::sqrt(CGAL::squared_distance(p2, p1));
      edges[4] = std::sqrt(CGAL::squared_distance(p2, p3));
      edges[5] = std::sqrt(CGAL::squared_distance(p1, p3));

      float min_edge = edges[0];
      for(int i=1; i<6; ++i)
      {
       if(edges[i]<min_edge)
         min_edge=edges[i];
      }
      float sumar = std::sqrt(CGAL::squared_area(p0,p1,p2))+std::sqrt(CGAL::squared_area(p1,p2,p3))+
          std::sqrt(CGAL::squared_area(p2,p3,p0)) + std::sqrt(CGAL::squared_area(p3,p1,p0));
      float inradius = 3*v/sumar;
      float smallest_edge_radius = min_edge/circumradius*std::sqrt(6)/4.0;//*sqrt(6)/4 so that the perfect tet ratio is 1
      float smallest_radius_radius = inradius/circumradius*3; //*3 so that the perfect tet ratio is 1 instead of 1/3
      float biggest_v_sma_cube = v/std::pow(min_edge,3)*6*std::sqrt(2);//*6*sqrt(2) so that the perfect tet ratio is 1 instead

      if(smallest_edge_radius < d->smallest_edge_radius)
        d->smallest_edge_radius = smallest_edge_radius;

      if(smallest_radius_radius < d->smallest_radius_radius)
        d->smallest_radius_radius = smallest_radius_radius;

      if(biggest_v_sma_cube > d->biggest_v_sma_cube)
        d->biggest_v_sma_cube = biggest_v_sma_cube;

      double a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p0, p1, p2, p3)));
      if(a < d->min_dihedral_angle) { d->min_dihedral_angle = a; }
      if(a > d->max_dihedral_angle) { d->max_dihedral_angle = a; }
      total_angle+=a;
      ++nb_angle;
      a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p0, p2, p1, p3)));
      if(a < d->min_dihedral_angle) { d->min_dihedral_angle = a; }
      if(a > d->max_dihedral_angle) { d->max_dihedral_angle = a; }
      total_angle+=a;
      ++nb_angle;
      a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p0, p3, p1, p2)));
      if(a < d->min_dihedral_angle) { d->min_dihedral_angle = a; }
      if(a > d->max_dihedral_angle) { d->max_dihedral_angle = a; }
      total_angle+=a;
      ++nb_angle;
      a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p1, p2, p0, p3)));
      if(a < d->min_dihedral_angle) { d->min_dihedral_angle = a; }
      if(a > d->max_dihedral_angle) { d->max_dihedral_angle = a; }
      total_angle+=a;
      ++nb_angle;
      a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p1, p3, p0, p2)));
      if(a < d->min_dihedral_angle) { d->min_dihedral_angle = a; }
      if(a > d->max_dihedral_angle) { d->max_dihedral_angle = a; }
      total_angle+=a;
      ++nb_angle;
      a = CGAL::to_double(CGAL::abs(approx_dihedral_angle(p2, p3, p0, p1)));
      if(a < d->min_dihedral_angle) { d->min_dihedral_angle = a; }
      if(a > d->max_dihedral_angle) { d->max_dihedral_angle = a; }
      total_angle+=a;
      ++nb_angle;
    }
    d->mean_dihedral_angle = total_angle/(float)nb_angle;
    d->nb_subdomains = sub_ids.size();
    d->nb_vertices = d->c3t3.number_of_vertices_in_complex();
    d->nb_tets = d->c3t3.number_of_cells();
    d->computed_stats = true;
  }

  switch (type)
  {
  case Scene_c3t3_item_priv::MIN_EDGES_LENGTH:
    return QString::number(d->min_edges_length);
  case Scene_c3t3_item_priv::MAX_EDGES_LENGTH:
    return QString::number(d->max_edges_length);
  case Scene_c3t3_item_priv::MEAN_EDGES_LENGTH:
    return QString::number(d->mean_edges_length);
  case Scene_c3t3_item_priv::MIN_DIHEDRAL_ANGLE:
    return QString::number(d->min_dihedral_angle);
  case Scene_c3t3_item_priv::MAX_DIHEDRAL_ANGLE:
    return QString::number(d->max_dihedral_angle);
  case Scene_c3t3_item_priv::MEAN_DIHEDRAL_ANGLE:
    return QString::number(d->mean_dihedral_angle);
  case Scene_c3t3_item_priv::NB_SPHERES:
    return QString::number(d->nb_spheres);
  case Scene_c3t3_item_priv::NB_CNC:
    return QString::number(d->nb_cnc);
  case Scene_c3t3_item_priv::NB_VERTICES:
    return QString::number(d->nb_vertices);
  case Scene_c3t3_item_priv::NB_TETS:
    return QString::number(d->nb_tets);
  case Scene_c3t3_item_priv::SMALLEST_RAD_RAD:
    return QString::number(d->smallest_radius_radius);
  case Scene_c3t3_item_priv::SMALLEST_EDGE_RAD:
    return QString::number(d->smallest_edge_radius);
  case Scene_c3t3_item_priv::BIGGEST_VL3_CUBE:
    return QString::number(d->biggest_v_sma_cube);
  case Scene_c3t3_item_priv::NB_SUBDOMAINS:
    return QString::number(d->nb_subdomains);

  default:
    return QString();
  }
}
CGAL::Three::Scene_item::Header_data Scene_c3t3_item::header() const
{
  CGAL::Three::Scene_item::Header_data data;
  //categories
  data.categories.append(std::pair<QString,int>(QString("Properties"),14));


  //titles
  data.titles.append(QString("Min Edges Length"));
  data.titles.append(QString("Max Edges Length"));
  data.titles.append(QString("Mean Edges Length"));
  data.titles.append(QString("Min Dihedral Angle"));
  data.titles.append(QString("Max Dihedral Angle"));
  data.titles.append(QString("Mean Dihedral Angle"));
  data.titles.append(QString("#Protecting Spheres"));
  data.titles.append(QString("#Cells not in Complex"));
  data.titles.append(QString("#Vertices in Complex"));
  data.titles.append(QString("#Cells"));
  data.titles.append(QString("Smallest Radius-Radius Ratio"));
  data.titles.append(QString("Smallest Edge-Radius Ratio"));
  data.titles.append(QString("Biggest Vl^3"));
  data.titles.append(QString("#Subdomains"));
  return data;
}

void Scene_c3t3_item::invalidate(Gl_data_names )
{
  setBuffersFilled(false);
  compute_bbox();
  d->invalidate_stats();
}
void Scene_c3t3_item::resetCutPlane()
{
  if(!d)
    return;
 d->reset_cut_plane();
}

void Scene_c3t3_item::itemAboutToBeDestroyed(Scene_item *item)
{
  Scene_item::itemAboutToBeDestroyed(item);

  if(d && item == this)
  {
    d->c3t3.clear();
    d->tree.clear();
    if(d->frame)
    {
      static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->setManipulatedFrame(0);
      delete d->frame;
      d->frame = NULL;
      if(d->tet_Slider)
      delete d->tet_Slider;
    }
    delete d;
    d=0;
  }

}
void Scene_c3t3_item::on_spheres_color_changed()
{
  if(!d->spheres)
    return;
  d->spheres->clear_spheres();
  d->computeSpheres();
}

#include "Scene_c3t3_item.moc"

