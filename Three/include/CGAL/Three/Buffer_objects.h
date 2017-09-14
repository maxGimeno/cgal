#ifndef BUFFER_OBJECTS_H
#define BUFFER_OBJECTS_H

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <vector>

namespace CGAL {
namespace Three {

struct Vbo;

struct Vao{

  QOpenGLVertexArrayObject* vao;
  QOpenGLShaderProgram* program;
  std::vector<Vbo*> vbos;

  Vao( QOpenGLShaderProgram* program)
    :vao(new QOpenGLVertexArrayObject()),
      program(program)
  {
    vao->create();
  }

  ~Vao()
  {
    delete vao;
  }

  void addVbo(Vbo* vbo)
  {
    vbos.push_back(vbo);
  }

  void bind()
  {
    program->bind();
    vao->bind();
  }

  void release()
  {
    vao->release();
    program->release();
  }
};


struct Vbo
{

  QOpenGLBuffer vbo;
  const char* attribute;
  void* data;
  int dataSize;
  GLenum type;
  int offset;
  int tupleSize;
  int stride;
  bool allocated;

  Vbo(
      const char* attribute,
      GLenum type=GL_FLOAT,
      int offset = 0,
      int tupleSize = 3,
      int stride = 0):
    attribute(attribute),
    type(type),
    offset(offset),
    tupleSize(tupleSize),
    stride(stride),
    allocated(false)
  {
    vbo.create();
  }
  ~Vbo()
  {
    vbo.destroy();
  }

  bool bind()
  {
    return vbo.bind();
  }

  void release()
  {
    vbo.release();
  }

  void allocate(void* data, int datasize)
  {
    this->data = data;
    this->dataSize = datasize;
  }

};

}
}
#endif // BUFFER_OBJECTS_H
