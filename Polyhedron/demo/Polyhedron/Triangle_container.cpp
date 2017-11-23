#include <CGAL/Three/Triangle_container.h>
#include <QOpenGLFramebufferObject>

typedef Viewer_interface VI;
using namespace CGAL::Three;

struct D{

  D():
    shrink_factor(1.0f),
    comparing(false),
    plane(QVector4D()),
    width(0.0f),
    height(0.0f),
    m_near(0.0f),
    m_far(0.0f),
    writing(false),
    alpha(1.0f)
  {}

  Triangle_container* container;
  float shrink_factor;

  bool comparing;

  QVector4D plane;

  float width;

  float height;

  float m_near;

  float m_far;

  bool writing;
  float alpha;
};

Triangle_container::Triangle_container(int program, bool indexed)
  : Primitive_container(program, indexed),
    d(new D())
{
  std::vector<Vbo*> vbos(NbOfVbos, NULL);
  setVbos(vbos);
}

void Triangle_container::initGL( Viewer_interface* viewer)
{
  viewer->makeCurrent();
  if(!getVao(viewer))
    setVao(viewer, new Vao(viewer->getShaderProgram(getProgram())));
  if(isDataIndexed())
  {
    if(!getVbo(Smooth_vertices))
      setVbo(Smooth_vertices,
             new Vbo("vertex", Vbo::GEOMETRY));
    if(!getVbo(Vertex_indices))
      setVbo(Vertex_indices,
             new Vbo("indices",
                     Vbo::GEOMETRY,
                     QOpenGLBuffer::IndexBuffer));
    getVao(viewer)->addVbo(getVbo(Smooth_vertices));
    getVao(viewer)->addVbo(getVbo(Vertex_indices));

    if(viewer->getShaderProgram(getProgram())->property("hasNormals").toBool())
    {
      if(!getVbo(Smooth_normals))
        setVbo(Smooth_normals,
               new Vbo("normals",
                       Vbo::NORMALS));
      getVao(viewer)->addVbo(getVbo(Smooth_normals));
    }
    if(!getVbo(VColors))
      setVbo(VColors,
             new Vbo("colors",
                     Vbo::COLORS,
                     QOpenGLBuffer::VertexBuffer, GL_FLOAT, 0, 4));
    getVao(viewer)->addVbo(getVbo(VColors));
  }
  else
  {
    if(!getVbo(Flat_vertices))
    {
      setVbo(Flat_vertices,
             new Vbo("vertex",
                     Vbo::GEOMETRY));
    }
    getVao(viewer)->addVbo(getVbo(Flat_vertices));
    if(viewer->getShaderProgram(getProgram())->property("hasNormals").toBool())
    {
    if(!getVbo(Flat_normals))
      setVbo(Flat_normals,
             new Vbo("normals",
                     Vbo::NORMALS));
    getVao(viewer)->addVbo(getVbo(Flat_normals));
    }
    if(!getVbo(FColors))
      setVbo(FColors,
             new Vbo("colors",
                     Vbo::COLORS,
                     QOpenGLBuffer::VertexBuffer, GL_FLOAT, 0, 4));
    getVao(viewer)->addVbo(getVbo(FColors));

    if(viewer->getShaderProgram(getProgram())->property("hasBarycenter").toBool())
    {
      if(!getVbo(Facet_barycenters))
        setVbo(Facet_barycenters,
               new Vbo("barycenter",
                       Vbo::GEOMETRY));
      getVao(viewer)->addVbo(getVbo(Facet_barycenters));
    }

    if(viewer->getShaderProgram(getProgram())->property("hasRadius").toBool())
    {
      if(!getVbo(Radius))
        setVbo(Radius,
            new Vbo("radius",
                    Vbo::GEOMETRY,
                    QOpenGLBuffer::VertexBuffer, GL_FLOAT, 0, 1));
      getVao(viewer)->addVbo(getVbo(Radius));

    }
  }
  setGLInit(viewer, true);
}

void Triangle_container::draw(Viewer_interface* viewer,
                              bool is_color_uniform,
                              QOpenGLFramebufferObject* fbo)
{

  bindUniformValues(viewer);

  if(isDataIndexed())
  {
    getVao(viewer)->bind();
    if(is_color_uniform)
      getVao(viewer)->program->setAttributeValue("colors", getColor());
    getVbo(Vertex_indices)->bind();
    if(getVao(viewer)->program->property("hasTransparency").toBool())
    {
      getVao(viewer)->program->setUniformValue("comparing", d->comparing);
      getVao(viewer)->program->setUniformValue("width", d->width);
      getVao(viewer)->program->setUniformValue("height", d->height);
      getVao(viewer)->program->setUniformValue("near", d->m_near);
      getVao(viewer)->program->setUniformValue("far", d->m_far);
      getVao(viewer)->program->setUniformValue("writing", d->writing);
      getVao(viewer)->program->setUniformValue("alpha", d->alpha);
      if( fbo)
        viewer->glBindTexture(GL_TEXTURE_2D, fbo->texture());
    }
    viewer->glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(getIdxSize()),
                           GL_UNSIGNED_INT, 0 );
    getVbo(Vertex_indices)->release();
    getVao(viewer)->release();
  }
  else
  {
    getVao(viewer)->bind();
    if(getVao(viewer)->program->property("hasCutPlane").toBool())
      getVao(viewer)->program->setUniformValue("cutplane", d->plane);
    if(is_color_uniform)
      getVao(viewer)->program->setAttributeValue("colors", getColor());
    if(getVao(viewer)->program->property("hasTransparency").toBool())
    {
      getVao(viewer)->program->setUniformValue("comparing", d->comparing);
      getVao(viewer)->program->setUniformValue("width", d->width);
      getVao(viewer)->program->setUniformValue("height", d->height);
      getVao(viewer)->program->setUniformValue("near", d->m_near);
      getVao(viewer)->program->setUniformValue("far", d->m_far);
      getVao(viewer)->program->setUniformValue("writing", d->writing);
      getVao(viewer)->program->setUniformValue("alpha", d->alpha);
      if( fbo)
        viewer->glBindTexture(GL_TEXTURE_2D, fbo->texture());
    }
    if(getVao(viewer)->program->property("isInstanced").toBool())
    {
      viewer->glDrawArraysInstanced(GL_TRIANGLES, 0,
                                    static_cast<GLsizei>(getFlatDataSize()/3),
                                    static_cast<GLsizei>(getCenterSize()/3));
    }
    else
    {

      viewer->glDrawArrays(GL_TRIANGLES,0,static_cast<GLsizei>(getFlatDataSize()/3));
    }

    getVao(viewer)->release();
  }
}


void Triangle_container::initializeBuffers(Viewer_interface *viewer)
{
  Primitive_container::initializeBuffers(viewer);
  if(getVao(viewer)->program->property("isInstanced").toBool())
  {
    getVao(viewer)->bind();
    if(getVao(viewer)->program->property("hasBarycenter").toBool())
      viewer->glVertexAttribDivisor(getVao(viewer)->program->attributeLocation("barycenter"), 1);
    if(getVao(viewer)->program->property("hasRadius").toBool())
      viewer->glVertexAttribDivisor(getVao(viewer)->program->attributeLocation("radius"), 1);
    viewer->glVertexAttribDivisor(getVao(viewer)->program->attributeLocation("colors"), 1);
    getVao(viewer)->release();
  }
}

float     Triangle_container::getShrinkFactor() { return d->shrink_factor ; }
bool      Triangle_container::isComparing()     { return d->comparing; }
QVector4D Triangle_container::getPlane()        { return d->plane; }
float     Triangle_container::getWidth()        { return d->width; }
float     Triangle_container::getHeight()       { return d->height; }
float     Triangle_container::getNear()         { return d->m_near; }
float     Triangle_container::getFar()          { return d->m_far; }
bool      Triangle_container::isDepthWriting()  { return d->writing; }
float     Triangle_container::getAlpha()        { return d->alpha; }

void Triangle_container::setShrinkFactor(const float& f) { d->shrink_factor = f; }
void Triangle_container::setComparing   (const bool& b)     { d->comparing = b; }
void Triangle_container::setPlane       (const QVector4D& p)    { d->plane = p; }
void Triangle_container::setWidth       (const float& f)        { d->width = f; }
void Triangle_container::setHeight      (const float& f)       { d->height = f; }
void Triangle_container::setNear        (const float& f)         { d->m_near = f; }
void Triangle_container::setFar         (const float& f)          { d->m_far = f; }
void Triangle_container::setDepthWriting(const bool& b)  { d->writing = b; }
void Triangle_container::setAlpha       (const float& f)        { d->alpha = f ; }
