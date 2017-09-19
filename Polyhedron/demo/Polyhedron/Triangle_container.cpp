#include <CGAL/Three/Triangle_container.h>
#include <QColor>
typedef CGAL::Three::Viewer_interface VI;
using namespace CGAL::Three;
Triangle_container::Triangle_container(CGAL::Three::Scene_item* item, CGAL::Three::Viewer_interface* viewer, int program, bool indexed)
  :program_id(program),
    indexed(indexed)
{
  nb_flat = 0;
  idx_size = 0;
  VBOs.resize(NbOfVbos);
  if(indexed)
  {
    switch(program_id)
    {
    case VI::PROGRAM_WITH_LIGHT:
    {
      VBOs[Smooth_vertices] =
          new CGAL::Three::Vbo("vertex");
      VBOs[Smooth_normals] =
          new CGAL::Three::Vbo("normals");
      VBOs[VColors] =
          new CGAL::Three::Vbo("colors");
      VBOs[Vertex_indices] =
          new CGAL::Three::Vbo("indices",
                               QOpenGLBuffer::IndexBuffer);
      VAO = new CGAL::Three::Vao(item->getShaderProgram(program_id, viewer));
      VAO->addVbo(VBOs[Smooth_vertices]);
      VAO->addVbo(VBOs[Vertex_indices]);
      VAO->addVbo(VBOs[Smooth_normals]);
      VAO->addVbo(VBOs[VColors]);
    }
      break;
    default:
      break;
    }
  }
  else
  {
    switch(program_id)
    {
    case VI::PROGRAM_WITH_LIGHT:
    case VI::PROGRAM_C3T3:
    case VI::PROGRAM_C3T3_TETS:

    {
      VBOs[Flat_vertices] =
          new CGAL::Three::Vbo("vertex");
      VBOs[Flat_normals] =
          new CGAL::Three::Vbo("normals");
      VBOs[FColors] =
          new CGAL::Three::Vbo("colors");
      VAO = new CGAL::Three::Vao(item->getShaderProgram(program_id, viewer));
      VAO->addVbo(VBOs[Flat_vertices]);
      VAO->addVbo(VBOs[Flat_normals]);
      VAO->addVbo(VBOs[FColors]);

    }
      break;
    }
    switch(program_id)
    {
    case VI::PROGRAM_C3T3:
      case VI::PROGRAM_C3T3_TETS:
    {
      VBOs[Facet_barycenters] =
          new CGAL::Three::Vbo("barycenter");
      VAO->addVbo(VBOs[Facet_barycenters]);
    }
      break;
    default:
      break;
    }
  }
  is_init = false;
}

void Triangle_container::draw(
    const CGAL::Three::Scene_item& item,
    CGAL::Three::Viewer_interface* viewer) const
{
  if(!is_init)
  {
    initializeBuffers();
  }
  item.attribBuffers(viewer, program_id);

  if(indexed)
  {
    VAO->bind();
    if(item.isSelected())
      VAO->program->setAttributeValue("is_selected", true);
    else
      VAO->program->setAttributeValue("is_selected", false);
    if(color == QColor(::Qt::color0))
      VAO->program->setAttributeValue("colors", item.color());
    VBOs[Vertex_indices]->bind();
    viewer->glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(idx_size),
                           GL_UNSIGNED_INT, 0 );
    VBOs[Vertex_indices]->release();
    VAO->release();
  }
  else
  {
    VAO->bind();
    if(program_id == VI::PROGRAM_C3T3
       || program_id == VI::PROGRAM_C3T3_TETS)
      VAO->program->setUniformValue("shrink_factor", shrink_factor);
    if(program_id == VI::PROGRAM_C3T3)
      VAO->program->setUniformValue("cutplane", plane);
    if(item.isSelected())
      VAO->program->setAttributeValue("is_selected", true);
    else
      VAO->program->setAttributeValue("is_selected", false);
    if(color == QColor(::Qt::color0))
      VAO->program->setAttributeValue("colors", color);
    viewer->glDrawArrays(GL_TRIANGLES,0,static_cast<GLsizei>(nb_flat/3));
    VAO->release();
  }
}


void Triangle_container::initializeBuffers()const
{
  //vao containing the data for the flat facets

  if(!VAO)
    return;
  VAO->bind();
  Q_FOREACH(CGAL::Three::Vbo* vbo, VAO->vbos)
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
        VAO->program->enableAttributeArray(vbo->attribute);
        VAO->program->setAttributeBuffer(vbo->attribute, vbo->data_type, vbo->offset, vbo->tupleSize, vbo->stride);
      }
    }
    else if(vbo->vbo_type == QOpenGLBuffer::VertexBuffer)
    {
      VAO->program->disableAttributeArray(vbo->attribute);
    }
    vbo->release();
  }
  VAO->release();

  is_init = true;
}

void Triangle_container::reset_vbos()
{
 Q_FOREACH(CGAL::Three::Vbo* vbo, VBOs)
 {
   if(!vbo)
     continue;
   vbo->allocated = false;
 }
}
