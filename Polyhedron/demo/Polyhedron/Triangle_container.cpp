#include <CGAL/Three/Triangle_container.h>

typedef CGAL::Three::Viewer_interface VI;
using namespace CGAL::Three;
Triangle_container::Triangle_container(CGAL::Three::Scene_item* item, CGAL::Three::Viewer_interface* viewer)
{
  nb_flat = 0;
  idx_size = 0;
  VBOs.resize(NbOfVbos);
  VBOs[Flat_vertices] =
      new CGAL::Three::Vbo("vertex");
  VBOs[Smooth_vertices] =
      new CGAL::Three::Vbo("vertex");
  VBOs[Flat_normals] =
      new CGAL::Three::Vbo("normals");
  VBOs[Smooth_normals] =
      new CGAL::Three::Vbo("normals");
  VBOs[VColors] =
      new CGAL::Three::Vbo("colors");
  VBOs[FColors] =
      new CGAL::Three::Vbo("colors");
  VBOs[Vertex_indices] =
      new CGAL::Three::Vbo("indices",
                           QOpenGLBuffer::IndexBuffer);

  VAOs.resize(NbOfVaos);
  VAOs[Flat_facets] = new CGAL::Three::Vao(item->getShaderProgram(VI::PROGRAM_WITH_LIGHT, viewer));
  VAOs[Flat_facets]->addVbo(VBOs[Flat_vertices]);
  VAOs[Flat_facets]->addVbo(VBOs[Flat_normals]);
  VAOs[Flat_facets]->addVbo(VBOs[FColors]);

  VAOs[Smooth_facets] = new CGAL::Three::Vao(item->getShaderProgram(VI::PROGRAM_WITH_LIGHT, viewer));
  VAOs[Smooth_facets]->addVbo(VBOs[Smooth_vertices]);
  VAOs[Smooth_facets]->addVbo(VBOs[Vertex_indices]);
  VAOs[Smooth_facets]->addVbo(VBOs[Smooth_normals]);
  VAOs[Smooth_facets]->addVbo(VBOs[VColors]);

  //  VAOs[Edges] = new CGAL::Three::Vao(viewer->getShaderProgram(PROGRAM_WITHOUT_LIGHT));
  //  VAOs[Edges]->addVbo(VBOs[Smooth_vertices]);
  is_init = false;
}

void Triangle_container::draw(
    const CGAL::Three::Scene_item& item,
    CGAL::Three::Viewer_interface* viewer,
    bool faces_have_color,
    bool vertices_have_color) const
{
  if(!is_init)
  {
    initializeBuffers();
  }
  item.attribBuffers(viewer, VI::PROGRAM_WITH_LIGHT);

  if(item.renderingMode() == Gouraud)
  {
    CGAL::Three::Vao* vao = VAOs[Smooth_facets];
    vao->bind();
    if(item.isSelected())
      vao->program->setAttributeValue("is_selected", true);
    else
      vao->program->setAttributeValue("is_selected", false);
    if(!vertices_have_color)
      vao->program->setAttributeValue("colors", item.color());

    VBOs[Vertex_indices]->bind();
    viewer->glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(idx_size),
                           GL_UNSIGNED_INT, 0 );
    VBOs[Vertex_indices]->release();
    vao->release();
  }
  else
  {
    CGAL::Three::Vao* vao =VAOs[Flat_facets];
    vao->bind();
    if(item.isSelected())
      vao->program->setAttributeValue("is_selected", true);
    else
      vao->program->setAttributeValue("is_selected", false);
    if(!faces_have_color)
      vao->program->setAttributeValue("colors", item.color());
    viewer->glDrawArrays(GL_TRIANGLES,0,static_cast<GLsizei>(nb_flat/3));
    vao->release();
  }
}

void Triangle_container::initializeBuffers()const
{
  //vao containing the data for the flat facets

  Q_FOREACH(CGAL::Three::Vao* vao, VAOs)
  {
    vao->bind();
    Q_FOREACH(CGAL::Three::Vbo* vbo, vao->vbos)
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
          vao->program->enableAttributeArray(vbo->attribute);
          vao->program->setAttributeBuffer(vbo->attribute, vbo->data_type, vbo->offset, vbo->tupleSize, vbo->stride);
        }
      }
      else if(vbo->vbo_type == QOpenGLBuffer::VertexBuffer)
      {
        vao->program->disableAttributeArray(vbo->attribute);
      }
      vbo->release();
    }
    vao->release();
  }
  is_init = true;
}
