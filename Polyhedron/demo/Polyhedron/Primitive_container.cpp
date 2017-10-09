#include <CGAL/Three/Primitive_container.h>
using namespace CGAL::Three;
struct D
{
  D(int program, bool indexed)
    :
      program_id(program),
      indexed(indexed),
      is_selected(false),
      flat_size(0),
      idx_size(0),
      color(QColor())
  {
  }
  QMap<CGAL::Three::Viewer_interface*, Vao*> VAOs;
  std::vector<Vbo*> VBOs;
  int program_id;
  bool indexed;
  QMap<CGAL::Three::Viewer_interface*, bool> is_init;
  QMap<CGAL::Three::Viewer_interface*, bool> is_gl_init;
  bool is_selected;
  std::size_t flat_size;
  std::size_t center_size;
  std::size_t idx_size;
  QColor color;
};

Primitive_container::Primitive_container(int program, bool indexed)
  :d(new D(program, indexed))
{}

Primitive_container::~Primitive_container()
{
  Q_FOREACH(Vbo* vbo, d->VBOs)
    if(vbo)
      delete vbo;
  Q_FOREACH(CGAL::Three::Viewer_interface*viewer, d->VAOs.keys())
  {
    removeViewer(viewer);
  }
}

void Primitive_container::bindUniformValues(CGAL::Three::Viewer_interface* viewer) const
{
  viewer->bindUniformValues(d->program_id);
  viewer->getShaderProgram(d->program_id)->bind();
  if(d->is_selected)
    viewer->getShaderProgram(d->program_id)->setUniformValue("is_selected", true);
  else
    viewer->getShaderProgram(d->program_id)->setUniformValue("is_selected", false);

  QColor c = d->color;
  if(d->program_id == Viewer_interface::PROGRAM_WITH_TEXTURE)
  {
    if(d->is_selected) c = c.lighter(120);
    viewer->getShaderProgram(d->program_id)->setAttributeValue
        ("color_facets",
         c.redF(),
         c.greenF(),
         c.blueF());
  }
  else if(d->program_id == Viewer_interface::PROGRAM_WITH_TEXTURED_EDGES)
  {
    if(d->is_selected) c = c.lighter(50);
    viewer->getShaderProgram(d->program_id)->setUniformValue
        ("color_lines",
         QVector3D(c.redF(), c.greenF(), c.blueF()));
  }
  viewer->getShaderProgram(d->program_id)->release();
}

void Primitive_container::initializeBuffers(CGAL::Three::Viewer_interface* viewer) const
{
  if(!d->VAOs[viewer])
    return;
  viewer->makeCurrent();
  d->VAOs[viewer]->bind();
  Q_FOREACH(CGAL::Three::Vbo* vbo, d->VAOs[viewer]->vbos)
  {
    vbo->bind();
    if(vbo->dataSize !=0)
    {
      if(!vbo->allocated)
      {
        if(vbo->vbo_type == QOpenGLBuffer::IndexBuffer)
          vbo->vbo.setUsagePattern(QOpenGLBuffer::StaticDraw);

        vbo->vbo.allocate(vbo->data, vbo->dataSize);
        vbo->allocated = true;
      }
      if(vbo->vbo_type == QOpenGLBuffer::VertexBuffer)
      {
        d->VAOs[viewer]->program->enableAttributeArray(vbo->attribute);
        d->VAOs[viewer]->program->setAttributeBuffer(vbo->attribute, vbo->data_type, vbo->offset, vbo->tupleSize, vbo->stride);
      }
    }
    else if(vbo->vbo_type == QOpenGLBuffer::VertexBuffer)
    {
      d->VAOs[viewer]->program->disableAttributeArray(vbo->attribute);
    }
    vbo->release();
  }
  d->VAOs[viewer]->release();

  d->is_init[viewer] = true;
}

void Primitive_container::removeViewer(CGAL::Three::Viewer_interface* viewer) const
{
  delete d->VAOs[viewer];
  d->VAOs.remove(viewer);
}

void Primitive_container::reset_vbos(Scene_item::Gl_data_names name)
{
  Q_FOREACH(CGAL::Three::Vbo* vbo, d->VBOs)
  {
    if(!vbo)
      continue;
    if(
       (name.testFlag(Scene_item::GEOMETRY) && vbo->flag == Vbo::GEOMETRY)
       || (name.testFlag(Scene_item::NORMALS) && vbo->flag == Vbo::NORMALS)
       || (name.testFlag(Scene_item::COLORS) && vbo->flag == Vbo::COLORS))
       vbo->allocated = false;
  }
}

void Primitive_container::allocate(std::size_t vbo_id, void* data, int datasize)const
{
  d->VBOs[vbo_id]->allocate(data, datasize);
}

void Primitive_container::setVao(Viewer_interface* viewer, Vao* vao)const
{
  d->VAOs[viewer]=vao;
}

void Primitive_container::setVbos(std::vector<Vbo*> vbos)const
{
  d->VBOs = vbos;
}

void Primitive_container::setInit(Viewer_interface* viewer, bool b)const
{
  d->is_init[viewer] = b;
}

void Primitive_container::setGLInit(Viewer_interface* viewer, bool b)const
{
  d->is_gl_init[viewer] = b;
}

void Primitive_container::setSelected(bool b)const
{
  d->is_selected = b;
}

void Primitive_container::setFlatDataSize(std::size_t s)const
{
  d->flat_size = s;
}

void Primitive_container::setCenterSize(std::size_t s)const
{
  d->center_size = s;
}

void Primitive_container::setIdxSize(std::size_t s)const
{
  d->idx_size = s;
}

void Primitive_container::setColor(QColor c)const
{
  d->color = c;
}

Vao* Primitive_container::getVao(Viewer_interface* viewer) const { return d->VAOs[viewer]; }

std::vector<Vbo*>& Primitive_container::getVbos() const { return d->VBOs; }

int Primitive_container::getProgram() const { return d->program_id; }

bool Primitive_container::isDataIndexed() const { return d->indexed; }

bool Primitive_container::isInit(Viewer_interface* viewer) const { return d->is_init[viewer]; }

bool Primitive_container::isGLInit(Viewer_interface* viewer) const { return d->is_gl_init[viewer]; }

bool Primitive_container::isSelected() const { return d->is_selected; }

std::size_t Primitive_container::getFlatDataSize() const { return d->flat_size; }

std::size_t Primitive_container::getCenterSize() const { return d->center_size; }

std::size_t Primitive_container::getIdxSize() const { return d->idx_size; }

QColor Primitive_container::getColor() const { return d->color; }

