#include "Scene_spheres_item.h"
#include<fstream>
#include <CGAL/Three/Triangle_container.h>
#include <CGAL/Three/Edge_container.h>
#include <QApplication>

using namespace CGAL::Three;

typedef Viewer_interface Vi;
typedef Triangle_container Tc;
typedef Edge_container Ec;

struct Scene_spheres_item_priv
{
  typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
  typedef CGAL::Sphere_3<Kernel> Sphere;
  typedef std::pair<Sphere, CGAL::Color> Sphere_pair;
  typedef std::vector<std::vector<Sphere_pair> > Spheres_container;

  Scene_spheres_item_priv(bool planed, std::size_t max_index, Scene_spheres_item* parent)
    :precision(36)
    ,has_plane(planed)
  {
    item = parent;
    create_flat_and_wire_sphere(1.0f,vertices,normals, edges);
    colors.clear();
    edges_colors.clear();
    centers.clear();
    radius.clear();
    spheres.resize(max_index + 1);
    nb_centers = 0;
    nb_edges = 0;
    nb_vertices = 0;
    model_sphere_is_up = false;
  }

  ~Scene_spheres_item_priv()
  {
  }

  void pick(int id)const;

  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const;

  int precision;
  mutable CGAL::Plane_3<Kernel> plane;
  bool has_plane;

  mutable std::vector<float> vertices;
  mutable std::vector<float> normals;
  mutable std::vector<float> edges;
  mutable std::vector<float> colors;
  mutable std::vector<float> edges_colors;
  mutable std::vector<float> picking_colors;
  mutable std::vector<float> centers;
  mutable std::vector<float> radius;
  mutable QOpenGLShaderProgram *program;
  mutable std::size_t nb_centers;
  mutable std::size_t nb_edges;
  mutable std::size_t nb_vertices;
  mutable bool model_sphere_is_up;
  Scene_spheres_item* item;
  QString tooltip;
  mutable Spheres_container spheres;

};

void Scene_spheres_item_priv::pick(int id) const
{
  int offset = 0;
  float color[4];
  for(std::size_t i=0; i<spheres.size(); ++i)
  {
   for( std::size_t j = 0; j< spheres[i].size(); ++j)
   {
     if(id == -1 || i != static_cast<std::size_t>(id))
     {
       color[0]=spheres[i][j].second.red()/255.0;
       color[1]=spheres[i][j].second.green()/255.0;
       color[2]=spheres[i][j].second.blue()/255.0;
     }
     else
     {
       color[0]=1.0f;
       color[1]=1.0f;
       color[2]=0.0f;
     }
     Vbo* color_vbo = item->getTriangleContainer(0)->getVbo(Tc::FColors);
     color_vbo->bind();
     color_vbo->vbo.write(offset*3*sizeof(float), color, 3*sizeof(float));
     color_vbo->release();
     ++offset;
   }
  }
}

Scene_spheres_item::Scene_spheres_item(Scene_group_item* parent, std::size_t max_index, bool planed)
{
  setParent(parent);
  d = new Scene_spheres_item_priv(planed, max_index, this);
  setTriangleContainer(0, 
                       new Tc(planed ? Vi::PROGRAM_CUTPLANE_SPHERES
                                     : Vi::PROGRAM_SPHERES
                                       ,false));
  setEdgeContainer(0, 
                   new Ec(planed ? Vi::PROGRAM_CUTPLANE_SPHERES
                                 : Vi::PROGRAM_SPHERES
                                   ,false));
}

Scene_spheres_item::~Scene_spheres_item()
{
  delete d;
}

void Scene_spheres_item_priv::initializeBuffers(CGAL::Three::Viewer_interface *viewer) const
{
  item->getTriangleContainer(0)->initializeBuffers(viewer);
  item->getTriangleContainer(0)->setFlatDataSize(nb_vertices);
  item->getTriangleContainer(0)->setCenterSize(nb_centers);
  vertices.clear();
  vertices.shrink_to_fit();
  item->getEdgeContainer(0)->initializeBuffers(viewer);
  item->getEdgeContainer(0)->setFlatDataSize(nb_edges);
  item->getEdgeContainer(0)->setCenterSize(nb_centers);
  edges.clear();
  edges.shrink_to_fit();

  centers.clear();
  centers.swap(centers);
  colors.clear();
  colors.swap(colors);
  radius.clear();
  radius.swap(radius);
  edges_colors.clear();
  edges_colors.swap(edges_colors);
}

void Scene_spheres_item::draw(Viewer_interface *viewer) const
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
  if(d->has_plane)
  {
    QVector4D cp(d->plane.a(),d->plane.b(),d->plane.c(),d->plane.d());
    getTriangleContainer(0)->setPlane(cp);
  }
  getTriangleContainer(0)->draw(viewer, false);
}

void Scene_spheres_item::drawEdges(Viewer_interface *viewer) const
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
  
  if(d->has_plane)
  {
    QVector4D cp(d->plane.a(),d->plane.b(),d->plane.c(),d->plane.d());
    getEdgeContainer(0)->setPlane(cp);
  }
  getEdgeContainer(0)->draw(viewer, false);
  
}
void Scene_spheres_item::add_sphere(const Sphere &sphere, std::size_t index,  CGAL::Color color)
{
  if(index > d->spheres.size()-1)
    d->spheres.resize(index+1);

  d->spheres[index].push_back(std::make_pair(sphere, color));

  d->colors.push_back((float)color.red()/255);
  d->colors.push_back((float)color.green()/255);
  d->colors.push_back((float)color.blue()/255);
  int R = (index & 0x000000FF) >>  0;
  int G = (index & 0x0000FF00) >>  8;
  int B = (index & 0x00FF0000) >> 16;
  float r= R/255.0;
  float g = G/255.0;
  float b = B/255.0;
  d->picking_colors.push_back(r);
  d->picking_colors.push_back(g);
  d->picking_colors.push_back(b);

    d->edges_colors.push_back((float)color.red()/255);
    d->edges_colors.push_back((float)color.green()/255);
    d->edges_colors.push_back((float)color.blue()/255);

    d->centers.push_back(sphere.center().x());
    d->centers.push_back(sphere.center().y());
    d->centers.push_back(sphere.center().z());

    d->radius.push_back(CGAL::sqrt(sphere.squared_radius()));
}

void Scene_spheres_item::clear_spheres()
{
  d->colors.clear();
  d->edges_colors.clear();
  d->centers.clear();
  d->radius.clear();
}
void Scene_spheres_item::setPrecision(int prec) { d->precision = prec; }
void Scene_spheres_item::setPlane(Kernel::Plane_3 p_plane) { d->plane = p_plane; }
void Scene_spheres_item::invalidateOpenGLBuffers()
{
  setBuffersFilled(false);
  getTriangleContainer(0)->reset_vbos(NOT_INSTANCED);
  getEdgeContainer(0)->reset_vbos(NOT_INSTANCED);
}

QString
Scene_spheres_item::toolTip() const {
    return d->tooltip;
}

void Scene_spheres_item::setToolTip(QString s)
{
  d->tooltip = s;
}

void Scene_spheres_item::setColor(QColor c)
{
  CGAL::Three::Scene_item::setColor(c);
  this->on_color_changed();
}

void Scene_spheres_item::initializeBuffers(Viewer_interface * v) const
{
  d->initializeBuffers(v);
}

void Scene_spheres_item::computeElements() const
{
  if(!d->model_sphere_is_up)
  {
    getTriangleContainer(0)->allocate(Tc::Flat_vertices, d->vertices.data(),
                                      static_cast<int>(d->vertices.size()*sizeof(float)));
    
    getTriangleContainer(0)->allocate(Tc::Flat_normals, d->normals.data(),
                                      static_cast<int>(d->normals.size()*sizeof(float)));
    d->nb_vertices = d->vertices.size();
    
  }
  getTriangleContainer(0)->allocate(Tc::FColors, d->colors.data(),
                                    static_cast<int>(d->colors.size()*sizeof(float)));
  getTriangleContainer(0)->allocate(Tc::Radius, d->radius.data(),
                                    static_cast<int>(d->radius.size()*sizeof(float)));
  
  getTriangleContainer(0)->allocate(Tc::Facet_centers, d->centers.data(),
                                    static_cast<int>(d->centers.size()*sizeof(float)));
  if(!d->model_sphere_is_up)
  {
    getEdgeContainer(0)->allocate(Ec::Vertices, d->edges.data(),
                                  static_cast<int>(d->edges.size()*sizeof(float)));
    
    getEdgeContainer(0)->allocate(Ec::Normals, d->normals.data(), 
                                  static_cast<int>(d->normals.size()*sizeof(float)));
    d->model_sphere_is_up = true;
    d->nb_edges = d->edges.size();
  }
  getEdgeContainer(0)->allocate(Ec::Colors, d->edges_colors.data(),
                                static_cast<int>(d->edges_colors.size()*sizeof(float)));
  
  getEdgeContainer(0)->allocate(Ec::Radius, d->radius.data(),
                                static_cast<int>(d->radius.size()*sizeof(float)));
  
  getEdgeContainer(0)->allocate(Ec::Centers, d->centers.data(),
                                static_cast<int>(d->centers.size()*sizeof(float)));
  
  d->nb_centers = d->centers.size();
  
}

void Scene_spheres_item::gl_initialization(Vi* viewer)
{
  if(!isInit(viewer))
    initGL(viewer);
  if ( getBuffersFilled() &&
       ! getBuffersInit(viewer))
  {
    initializeBuffers(viewer);
    setBuffersInit(viewer, true);
    setBuffersFilled(true);
  }
}

void Scene_spheres_item::compute_bbox() const
{
  Bbox box = Bbox();
  for(std::size_t id=0; id<d->spheres.size(); ++id)
  {
    for(Sphere_pair pair : d->spheres[id])
    {
      box += pair.first.bbox();
    }
  }
  _bbox = box;
}

bool Scene_spheres_item::save(const std::string& file_name)const
{

  std::ofstream out(file_name.c_str());
  if(!out) { return false; }

  std::size_t nb_spheres=0;
  for(std::size_t i = 0; i<d->spheres.size(); ++i)
    nb_spheres += d->spheres[i].size();

  out<<nb_spheres << " " << d->spheres.size() -1<<"\n";

  for(std::size_t i = 0; i<d->spheres.size(); ++i)
  {
    for(const Sphere_pair& pair : d->spheres[i])
    {
      out << i << " " << pair.first.center() << " " << CGAL::sqrt(pair.first.squared_radius())<<"\n";
    }
  }
  out << "\n";
  return true;
}
