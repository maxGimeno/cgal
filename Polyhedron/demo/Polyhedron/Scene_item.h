#ifndef SCENE_ITEM_H
#define SCENE_ITEM_H
#include "Scene_item_config.h"
#include "Scene_interface.h"
#include <QString>
#include <QPixmap>
#include <QFont>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShader>
#include <QOpenGLVertexArrayObject>
#include <vector>
#include <QMap>
#define PROGRAM_WITH_LIGHT 0
#define PROGRAM_WITHOUT_LIGHT 1
#define PROGRAM_WITH_TEXTURE 2
#define PROGRAM_WITH_TEXTURED_EDGES 3
#define PROGRAM_INSTANCED 4
#define PROGRAM_INSTANCED_WIRE 5
namespace qglviewer {
  class ManipulatedFrame;
}

class QMenu;
class QKeyEvent;
class Viewer_interface;

// This class represents an object in the OpenGL scene
class SCENE_ITEM_EXPORT Scene_item : public QObject {
  Q_OBJECT
  Q_PROPERTY(QColor color READ color WRITE setColor)
  Q_PROPERTY(QString name READ name WRITE setName)
  Q_PROPERTY(bool visible READ visible WRITE setVisible)
  Q_ENUMS(RenderingMode)
  Q_PROPERTY(RenderingMode renderingMode READ renderingMode WRITE setRenderingMode)
public:
  typedef Scene_interface::Bbox Bbox;
  typedef qglviewer::ManipulatedFrame ManipulatedFrame;

  static const QColor defaultColor; // defined in Scene_item.cpp

  Scene_item()
    : name_("unamed"),
      color_(defaultColor),
      visible_(true),
      rendering_mode(FlatPlusEdges),
      defaultContextMenu(0),
      buffersSize(20),
      vaosSize(10),
      are_buffers_filled(false)
  {
      for(int i=0; i<vaosSize; i++)
      {
          addVaos(i);
          vaos[i]->create();
      }

      for(int i=0; i<buffersSize; i++)
      {
          QOpenGLBuffer n_buf;
          buffers.push_back(n_buf);
          buffers[i].create();
      }
  }
  Scene_item(int buffers_size, int vaos_size)
    : name_("unamed"),
      color_(defaultColor),
      visible_(true),
      rendering_mode(FlatPlusEdges),
      defaultContextMenu(0),
      buffersSize(buffers_size),
      vaosSize(vaos_size),
      are_buffers_filled(false)
  {
      areVaosCreated = false;
      nbVaos = 0;
      for(int i=0; i<buffersSize; i++)
      {
          QOpenGLBuffer n_buf;
          buffers.push_back(n_buf);
          buffers[i].create();
      }
  }
  virtual ~Scene_item();
  virtual Scene_item* clone() const = 0;
  mutable  bool areVaosCreated;
  // Indicate if rendering mode is supported
  virtual bool supportsRenderingMode(RenderingMode m) const = 0;
  // Flat/Gouraud OpenGL drawing
  virtual void draw() const {
        if(!areVaosCreated)
        {
            for(int i=0; i<vaosSize; i++)
            { 
                    addVaos(i);
                    vaos[i]->create();
            }
            areVaosCreated = true;
        }

}
  virtual void draw(Viewer_interface*) const  { draw(); }
  // Wireframe OpenGL drawing
  virtual void draw_edges() const { draw(); }
  virtual void draw_edges(Viewer_interface* viewer) const { draw(viewer); }
  // Points OpenGL drawing
  virtual void draw_points() const { draw(); }
  virtual void draw_points(Viewer_interface*) const { draw_points(); }
  // Splats OpenGL drawing
  virtual void draw_splats() const {}
  virtual void draw_splats(Viewer_interface*) const {draw_splats();}
  virtual void selection_changed(bool);

  // Functions for displaying meta-data of the item
  virtual QString toolTip() const = 0;
  virtual QPixmap graphicalToolTip() const { return QPixmap(); }
  virtual QFont font() const { return QFont(); }

  // Functions that help the Scene to compute its bbox
  virtual bool isFinite() const { return true; }
  virtual bool isEmpty() const { return true; }
  virtual Bbox bbox() const { return Bbox(); }

  // Function about manipulation
  virtual bool manipulatable() const { return false; }
  virtual ManipulatedFrame* manipulatedFrame() { return 0; }

  // Getters for the four basic properties
  virtual QColor color() const { return color_; }
  virtual QString name() const { return name_; }
  virtual bool visible() const { return visible_; }
  virtual RenderingMode renderingMode() const { return rendering_mode; }
  virtual QString renderingModeName() const; // Rendering mode as a human
                                             // readable string

  // Context menu
  virtual QMenu* contextMenu();

  // Event handling
  virtual bool keyPressEvent(QKeyEvent*){return false;}
  mutable QMap<int, QOpenGLShaderProgram*> shader_programs;
public slots:
  // Call that once you have finished changing something in the item
  // (either the properties or internal data)
  virtual void changed();
  virtual void contextual_changed(){}

  // Setters for the four basic properties
  virtual void setColor(QColor c) { color_ = c; changed(); }
  void setRbgColor(int r, int g, int b) { setColor(QColor(r, g, b)); changed(); }
  virtual void setName(QString n) { name_ = n; }
  virtual void setVisible(bool b) { visible_ = b; }
  virtual void setRenderingMode(RenderingMode m) { 
    if (supportsRenderingMode(m))
      rendering_mode = m; 
  }
  void setPointsMode() {
    setRenderingMode(Points);
  }

  void setWireframeMode() {
    setRenderingMode(Wireframe);
  }
  void setWireframe() {
    setRenderingMode(Wireframe);
  }

  void setFlat() {
    setRenderingMode(Flat);
  }
  void setFlatMode() {
    setRenderingMode(Flat);
  }

  void setFlatPlusEdgesMode() {
    setRenderingMode(FlatPlusEdges);
  }

  void setGouraudMode() {
    setRenderingMode(Gouraud);
  }

  void setPointsPlusNormalsMode(){
    setRenderingMode(PointsPlusNormals);
  }
  
  void setSplattingMode(){
    setRenderingMode(Splatting);
  }
  
  virtual void itemAboutToBeDestroyed(Scene_item*);

  virtual void select(double orig_x,
                      double orig_y,
                      double orig_z,
                      double dir_x,
                      double dir_y,
                      double dir_z);

signals:
  void itemChanged();
  void aboutToBeDestroyed();

protected:
  // The four basic properties
  QString name_;
  QColor color_;
  bool visible_;
  bool is_selected;
  mutable bool are_buffers_filled;
  RenderingMode rendering_mode;
  QMenu* defaultContextMenu;

  int prev_shading;
  int cur_shading;


  mutable QOpenGLFunctions qFunc;
  int buffersSize;
  int vaosSize;
  mutable std::vector<QOpenGLBuffer> buffers;
  //not allowed to use vectors of VAO for some reason
  //mutable QOpenGLVertexArrayObject vaos[10];
  mutable QMap<int,QOpenGLVertexArrayObject*> vaos;
  mutable int nbVaos;
  void addVaos(int i)const
  {
      QOpenGLVertexArrayObject* n_vao = new QOpenGLVertexArrayObject();
      vaos[i] = n_vao;
      nbVaos ++;
  }



  QOpenGLShaderProgram* getShaderProgram(int , Viewer_interface *viewer = 0) const;

  int vertexLoc;
  int normalLoc;
  int colorLoc;

  virtual void initialize_buffers(){}
  virtual void compute_elements(){}
  virtual void attrib_buffers(Viewer_interface*, int program_name) const;

}; // end class Scene_item


#include <QMetaType>
Q_DECLARE_METATYPE(Scene_item*)

#endif // SCENE_ITEM_H
