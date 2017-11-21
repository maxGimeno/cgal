#include "Scene_spheres_item.h"

#include <QApplication>
#include <QOpenGLFramebufferObject>
#include <fstream>
#include "Messages_interface.h"


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
  }

  ~Scene_spheres_item_priv()
  {
  }

  void pick(int id)const;
  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const;
  void compute_elements()const;
  enum Vbos
  {
    Vertices = 0,
    Edge_vertices,
    Normals,
    Center,
    Radius,
    Color,
    Edge_color,
    Picking_color,
    NbOfVbos
  };
  enum Vaos
  {
    Facets = 0,
    Edges,
    NbOfVaos
  };


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
  mutable int nb_centers;
  Scene_spheres_item* item;
  QString tooltip;
  mutable Spheres_container spheres;
};
Scene_spheres_item::Scene_spheres_item(Scene_group_item* parent, std::size_t max_index, bool planed)
  :CGAL::Three::Scene_item(Scene_spheres_item_priv::NbOfVbos,Scene_spheres_item_priv::NbOfVaos)

{
  setParent(parent);
  d = new Scene_spheres_item_priv(planed, max_index, this);
}

Scene_spheres_item::~Scene_spheres_item()
{
  delete d;
}

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
       //color[3] = 1.0;
     }
     else
     {
       color[0]=1.0f;
       color[1]=1.0f;
       color[2]=0.0f;
       //color[3] = 1.0;
     }
     item->buffers[Color].bind();
     item->buffers[Color].write(offset*3*sizeof(float), color, 3*sizeof(float));
     item->buffers[Color].release();
     ++offset;
   }

  }
  //item->invalidateOpenGLBuffers();
}

void Scene_spheres_item_priv::initializeBuffers(CGAL::Three::Viewer_interface *viewer) const
{
  if(has_plane)
  {
    program = item->getShaderProgram(Scene_spheres_item::PROGRAM_CUTPLANE_SPHERES, viewer);
    item->attribBuffers(viewer, Scene_spheres_item::PROGRAM_CUTPLANE_SPHERES);
  }
  else
  {
    program = item->getShaderProgram(Scene_spheres_item::PROGRAM_SPHERES, viewer);
    item->attribBuffers(viewer, Scene_spheres_item::PROGRAM_SPHERES);
  }

  program->bind();
  item->vaos[Facets]->bind();
  item->buffers[Vertices].bind();
  item->buffers[Vertices].allocate(vertices.data(),
                             static_cast<int>(vertices.size()*sizeof(float)));
  program->enableAttributeArray("vertex");
  program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3);
  item->buffers[Vertices].release();

  item->buffers[Normals].bind();
  item->buffers[Normals].allocate(normals.data(),
                            static_cast<int>(normals.size()*sizeof(float)));
  program->enableAttributeArray("normals");
  program->setAttributeBuffer("normals", GL_FLOAT, 0, 3);
  item->buffers[Normals].release();

  item->buffers[Color].bind();
  item->buffers[Color].allocate(colors.data(),
                          static_cast<int>(colors.size()*sizeof(float)));
  program->enableAttributeArray("colors");
  item->buffers[Color].release();
  if(spheres.size() > 1)
  {
    item->buffers[Picking_color].bind();
    item->buffers[Picking_color].allocate(picking_colors.data(),
                                          static_cast<int>(picking_colors.size()*sizeof(float)));
    item->buffers[Picking_color].release();
  }

  item->buffers[Radius].bind();
  item->buffers[Radius].allocate(radius.data(),
                           static_cast<int>(radius.size()*sizeof(float)));
  program->enableAttributeArray("radius");
  program->setAttributeBuffer("radius", GL_FLOAT, 0, 1);
  item->buffers[Radius].release();

  item->buffers[Center].bind();
  item->buffers[Center].allocate(centers.data(),
                           static_cast<int>(centers.size()*sizeof(float)));
  program->enableAttributeArray("center");
  program->setAttributeBuffer("center", GL_FLOAT, 0, 3);
  item->buffers[Center].release();

  viewer->glVertexAttribDivisor(program->attributeLocation("center"), 1);
  viewer->glVertexAttribDivisor(program->attributeLocation("radius"), 1);
  viewer->glVertexAttribDivisor(program->attributeLocation("colors"), 1);
  item->vaos[Facets]->release();


  item->vaos[Edges]->bind();
  item->buffers[Edge_vertices].bind();
  item->buffers[Edge_vertices].allocate(edges.data(),
                                  static_cast<int>(edges.size()*sizeof(float)));
  program->enableAttributeArray("vertex");
  program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3);
  item->buffers[Edge_vertices].release();

  item->buffers[Normals].bind();
  program->enableAttributeArray("normals");
  program->setAttributeBuffer("normals", GL_FLOAT, 0, 3);
  item->buffers[Normals].release();

  item->buffers[Edge_color].bind();
  item->buffers[Edge_color].allocate(edges_colors.data(),
                               static_cast<int>(edges_colors.size()*sizeof(float)));
  program->enableAttributeArray("colors");
  program->setAttributeBuffer("colors", GL_FLOAT, 0, 3);
  item->buffers[Edge_color].release();

  item->buffers[Radius].bind();
  program->enableAttributeArray("radius");
  program->setAttributeBuffer("radius", GL_FLOAT, 0, 1);
  item->buffers[Radius].release();

  item->buffers[Center].bind();
  program->enableAttributeArray("center");
  program->setAttributeBuffer("center", GL_FLOAT, 0, 3);
  item->buffers[Center].release();

  viewer->glVertexAttribDivisor(program->attributeLocation("center"), 1);
  viewer->glVertexAttribDivisor(program->attributeLocation("radius"), 1);
  viewer->glVertexAttribDivisor(program->attributeLocation("colors"), 1);
  item->vaos[Edges]->release();

  program->release();

  nb_centers = static_cast<int>(centers.size());
  centers.clear();
  centers.swap(centers);
  radius.clear();
  radius.swap(radius);
  edges_colors.clear();
  edges_colors.swap(edges_colors);

  item->are_buffers_filled = true;
}

void Scene_spheres_item_priv::compute_elements() const
{
  picking_colors.clear();
  colors.clear();
  for(std::size_t id=0; id<spheres.size(); ++id)
  {
    Q_FOREACH(Sphere_pair pair, spheres[id])
    {
      if(spheres.size()>1)
      {
        int R = (id & 0x000000FF) >>  0;
        int G = (id & 0x0000FF00) >>  8;
        int B = (id & 0x00FF0000) >> 16;
        float r= R/255.0;
        float g = G/255.0;
        float b = B/255.0;
        picking_colors.push_back(r);
        picking_colors.push_back(g);
        picking_colors.push_back(b);
      }

      colors.push_back((float)pair.second.red()/255);
      colors.push_back((float)pair.second.green()/255);
      colors.push_back((float)pair.second.blue()/255);

      edges_colors.push_back((float)pair.second.red()/255);
      edges_colors.push_back((float)pair.second.green()/255);
      edges_colors.push_back((float)pair.second.blue()/255);

      centers.push_back(pair.first.center().x());
      centers.push_back(pair.first.center().y());
      centers.push_back(pair.first.center().z());

      radius.push_back(CGAL::sqrt(pair.first.squared_radius()));
    }
  }
}

void Scene_spheres_item::draw(Viewer_interface *viewer) const
{
  if (!are_buffers_filled)
  {
    d->compute_elements();
    d->initializeBuffers(viewer);
  }
  int deviceWidth = viewer->camera()->screenWidth();
  int deviceHeight = viewer->camera()->screenHeight();

//  QOpenGLFramebufferObject* fbo = 0;

  if(d->spheres.size() > 1 && viewer->inDrawWithNames())
  {
    vaos[Scene_spheres_item_priv::Facets]->bind();
    buffers[Scene_spheres_item_priv::Picking_color].bind();
    d->program->setAttributeBuffer("colors", GL_FLOAT, 0, 3);
    buffers[Scene_spheres_item_priv::Picking_color].release();
    d->program->disableAttributeArray("normals");
    d->program->setAttributeValue("normals", QVector3D(0,0,0));
  }
  else
  {
    vaos[Scene_spheres_item_priv::Facets]->bind();
    buffers[Scene_spheres_item_priv::Color].bind();
    d->program->setAttributeBuffer("colors", GL_FLOAT, 0, 3);
    buffers[Scene_spheres_item_priv::Color].release();
    d->program->enableAttributeArray("normals");
  }

  if(d->has_plane)
  {
    d->program = getShaderProgram(PROGRAM_CUTPLANE_SPHERES, viewer);
    attribBuffers(viewer, PROGRAM_CUTPLANE_SPHERES);
    d->program->bind();
    QVector4D cp(d->plane.a(),d->plane.b(),d->plane.c(),d->plane.d());
    d->program->setUniformValue("cutplane", cp);

  }
  else
  {
    d->program = getShaderProgram(PROGRAM_SPHERES, viewer);
    attribBuffers(viewer, PROGRAM_SPHERES);
    d->program->bind();
  }
  viewer->glDrawArraysInstanced(GL_TRIANGLES, 0,
                                static_cast<GLsizei>(d->vertices.size()/3),
                                static_cast<GLsizei>(d->nb_centers));
  d->program->release();
  vaos[Scene_spheres_item_priv::Facets]->release();
  if(d->spheres.size() > 1 && viewer->inDrawWithNames())
  {
    int rowLength = deviceWidth * 4; // data asked in RGBA,so 4 bytes.
    const static int dataLength = rowLength * deviceHeight;
    GLubyte* buffer = new GLubyte[dataLength];
    // Qt uses upper corner for its origin while GL uses the lower corner.
    QPoint picking_target = viewer->mapFromGlobal(QCursor::pos());
    viewer->glReadPixels(picking_target.x(), deviceHeight-1-picking_target.y(), 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
    int ID = (buffer[0] + buffer[1] * 256 +buffer[2] * 256*256);
    if(buffer[0]*buffer[1]*buffer[2] < 255*255*255)
    {
      d->pick(ID);
      viewer->displayMessage(QString("Picked spheres index : %1 .").arg(ID), 2000);
    }
    else
      d->pick(-1);
  }
}

void Scene_spheres_item::drawEdges(Viewer_interface *viewer) const
{
  if(viewer->inDrawWithNames())
    return;
  if (!are_buffers_filled)
  {
    d->compute_elements();
    d->initializeBuffers(viewer);
  }
  vaos[Scene_spheres_item_priv::Edges]->bind();
  if(d->has_plane)
  {
    d->program = getShaderProgram(PROGRAM_CUTPLANE_SPHERES, viewer);
    attribBuffers(viewer, PROGRAM_CUTPLANE_SPHERES);
    d->program->bind();
    QVector4D cp(d->plane.a(),d->plane.b(),d->plane.c(),d->plane.d());
    d->program->setUniformValue("cutplane", cp);
  }
  else
  {
    d->program = getShaderProgram(PROGRAM_SPHERES, viewer);
    attribBuffers(viewer, PROGRAM_SPHERES);
    d->program->bind();
  }
  viewer->glDrawArraysInstanced(GL_LINES, 0,
                                static_cast<GLsizei>(d->edges.size()/3),
                                static_cast<GLsizei>(d->nb_centers));
  d->program->release();
  vaos[Scene_spheres_item_priv::Edges]->release();
}

void Scene_spheres_item::add_sphere(const Sphere &sphere, std::size_t index,  CGAL::Color color)
{
  if(index > d->spheres.size()-1)
    d->spheres.resize(index+1);
  d->spheres[index].push_back(std::make_pair(sphere, color));
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

void Scene_spheres_item::invalidateOpenGLBuffers(){are_buffers_filled = false;}

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

void Scene_spheres_item::compute_bbox() const
{
  Bbox box = Bbox();
  for(std::size_t id=0; id<d->spheres.size(); ++id)
  {
    Q_FOREACH(Sphere_pair pair, d->spheres[id])
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
    Q_FOREACH(const Sphere_pair& pair, d->spheres[i])
    {
      out << i << " " << pair.first.center() << " " << CGAL::sqrt(pair.first.squared_radius())<<"\n";
    }
  }
  out << "\n";
  return true;
}
