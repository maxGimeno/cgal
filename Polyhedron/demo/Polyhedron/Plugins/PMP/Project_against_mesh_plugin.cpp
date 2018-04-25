#include <QApplication>
#include <QObject>
#include <QAction>
#include <QMainWindow>
#include <QInputDialog>
#include <QMessageBox>

#include <QGLViewer/manipulatedFrame.h>

#include <CGAL/Three/Polyhedron_demo_plugin_helper.h>
#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>
#include <CGAL/Three/Viewer_interface.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>

#include <CGAL/boost/graph/Face_filtered_graph.h>

#include "Messages_interface.h"
#include "Scene_surface_mesh_item.h"
#include "create_sphere.h"
#include "Scene.h"

#include "triangulate_primitive.h"

using namespace CGAL::Three;
namespace Euler=CGAL::Euler;
class Scene_create_surface_item : public CGAL::Three::Scene_item
{
  Q_OBJECT
public :
  Scene_create_surface_item(Point_3 p1, Point_3 p2)
    : p1(p1), p2(p2),
      frame(new CGAL::Three::Scene_item::ManipulatedFrame())
  {
    const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
    frame->setPosition(offset);
    _bbox=Bbox(p1.x(), p1.y(), p1.z(),
               p2.x(), p2.y(), p2.z());
    vertex_spheres.resize(0);
    normal_spheres.resize(0);
    create_flat_sphere(1.0f, vertex_spheres, normal_spheres,10);
    mode = 'a';
  }
  CGAL::Three::Scene_item::ManipulatedFrame* manipulatedFrame() Q_DECL_OVERRIDE { return frame; }
  bool manipulatable() const Q_DECL_OVERRIDE { return true; }
  // Indicates if rendering mode is supported
  bool supportsRenderingMode(RenderingMode m) const Q_DECL_OVERRIDE {
    return (m == FlatPlusEdges);
  }
  //Displays the item
  
  void draw(Viewer_interface * viewer) const Q_DECL_OVERRIDE
  {
    return;
    if (!are_buffers_filled)
    {
      computeElements();
      initializeBuffers(viewer);
    }
    vaos[1]->bind();
    GLdouble d_mat[16];
    QMatrix4x4 mvp_mat;
    GLdouble matrix[16];
    QMatrix4x4 f_matrix;
    frame->getMatrix(matrix);
    for (int i=0; i<16; ++i)
      f_matrix.data()[i] = (float)matrix[i];
    viewer->camera()->getModelViewProjectionMatrix(d_mat);
    for (int i=0; i<16; ++i)
      mvp_mat.data()[i] = GLfloat(d_mat[i]);
    mvp_mat = mvp_mat*f_matrix;
    QMatrix4x4 mv_mat;
    viewer->camera()->getModelViewMatrix(d_mat);
    for (int i=0; i<16; ++i)
      mv_mat.data()[i] = GLfloat(d_mat[i]);
    mv_mat = mv_mat*f_matrix;
    attribBuffers(viewer, PROGRAM_SPHERES);
    program = getShaderProgram(PROGRAM_SPHERES, viewer);
    program->bind();
    program->setUniformValue("mvp_matrix", mvp_mat);
    program->setUniformValue("mv_matrix", mv_mat);
    program->setUniformValue("is_clipbox_on", false);
    program->setAttributeValue("radius",0.01*diagonalBbox());
    program->setAttributeValue("colors", QColor(Qt::red));
    viewer->glDrawArraysInstanced(GL_TRIANGLES, 0,
                                  static_cast<GLsizei>(vertex_spheres.size()/3),
                                  static_cast<GLsizei>(4));
    program->release();
    vaos[1]->release();
  }
  void drawEdges(CGAL::Three::Viewer_interface* viewer) const Q_DECL_OVERRIDE
  {
    //The filling of the buffers should be performed in this function, because it needs a valid openGL context, and we are certain to have one in this function.
     if(!are_buffers_filled)
     {
       computeElements();
       initializeBuffers(viewer);
     }
     GLdouble matrix[16];
     QMatrix4x4 f_matrix;
     frame->getMatrix(matrix);
     for (int i=0; i<16; ++i)
       f_matrix.data()[i] = (float)matrix[i];
     vaos[0]->bind();
     program = getShaderProgram(PROGRAM_NO_SELECTION);
     attribBuffers(viewer, PROGRAM_NO_SELECTION);
     program->bind();
     program->setUniformValue("f_matrix", f_matrix);
     program->setAttributeValue("colors", QColor(Qt::green));
     //Draws the items
     viewer->glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(4));
     viewer->glDrawArrays(GL_LINES, 4, static_cast<GLsizei>(2));
     
     //clean up
     vaos[0]->release();
     program->release();
  }
  
  //Specifies that the buffers need to be initialized again.
  //Is mostly called after a change of geometry in the data.
  void invalidateOpenGLBuffers() Q_DECL_OVERRIDE
  {
    are_buffers_filled = false;
    _bbox=Bbox(p1.x(), p1.y(), p1.z(),
               p2.x(), p2.y(), p2.z());
    redraw();
  }
  //fills the std::vector
  void computeElements() const
  {
    vertices.resize(18);
    vertices[0] = p1.x();  vertices[3] = p2.x();    
    vertices[1] = p1.y();  vertices[4] = p1.y();
    vertices[2] = p1.z();  vertices[5] = p1.z();
    
    vertices[6] = p2.x();  vertices[9] = p1.x();    
    vertices[7] = p2.y();  vertices[10] = p2.y();
    vertices[8] = p1.z();  vertices[11] = p1.z();
    
    vertices[12] = (p1.x()+p2.x())/2;  vertices[15] = (p1.x()+p2.x())/2;    
    vertices[13] = (p1.y()+p2.y())/2;  vertices[16] = (p1.y()+p2.y())/2;
    vertices[14] = p1.z();             vertices[17] = p1.z() - 0.1*diagonalBbox();
    
    
   return; center_spheres.resize(12);
    
    center_spheres[0]=p2.x();             center_spheres[3]=(p1.x()+p2.x())/2;      
    center_spheres[1]=(p1.y()+p2.y())/2;  center_spheres[4]=p2.y();
    center_spheres[2]=p1.z();             center_spheres[5]=p1.z();
    
    center_spheres[6]= p1.x();            center_spheres[9] =(p1.y()+p2.y())/2;
    center_spheres[7]= (p1.y()+p2.y())/2; center_spheres[10]=p1.y();;
    center_spheres[8]=  p1.z();           center_spheres[11]=p1.z();
    
    
    
    
  }
  Scene_item* clone() const Q_DECL_OVERRIDE {return 0;}
  QString toolTip() const Q_DECL_OVERRIDE {return QString();}
  bool keyPressEvent(QKeyEvent *e) Q_DECL_OVERRIDE
  {
    if (e->key()==Qt::Key_Plus)
    {
      double p1x=p1.x(), p1y=p1.y(),
          p2x=p2.x(), p2y=p2.y();
      switch(mode)
      {
      case 'a':
      case 'w':
        p1x-=0.01*diagonalBbox();
        p2x+=0.01*diagonalBbox();
        break;
      default:
        break;
      }
      switch(mode)
      {
      case 'a':
      case 'h':
        p1y-=0.01*diagonalBbox();
        p2y+=0.01*diagonalBbox();
        break;
      default:
        break;
      }
      p1 = Point_3(p1x,
                   p1y,
                   p1.z());
      p2 = Point_3(
            p2x,
            p2y,
            p2.z());
      invalidateOpenGLBuffers();
      return true;
    }
    else if (e->key() == Qt::Key_Minus)
    {
      double p1x=p1.x(), p1y=p1.y(),
          p2x=p2.x(), p2y=p2.y();
      switch(mode)
      {
      case 'a':
      case 'w':
        p1x+=0.01*diagonalBbox();
        p2x-=0.01*diagonalBbox();
        break;
      default:
        break;
      }
      switch(mode){
      case 'a':
      case 'h':
        p1y+=0.01*diagonalBbox();
        p2y-=0.01*diagonalBbox();
        break;
      default:
        break;
      }
      p1 = Point_3(p1x,
                   p1y,
                   p1.z());
      p2 = Point_3(
            p2x,
            p2y,
            p2.z());
      invalidateOpenGLBuffers();
      return true;
    }
    else if(e->key() == Qt::Key_A)
    {
      mode = 'a';
      return true;
    }
    else if(e->key() == Qt::Key_W)
    {
      mode='w';
      return true;
    }
    else if(e->key() == Qt::Key_H)
    {
      mode='h';
      return true;
    }
    return false;
  }
  Point_3 getP1() const {return p1; }
  Point_3 getP2() const {return p2; }
private:
  
  //contains the data
  Point_3 p1, p2;
  mutable std::vector<float> vertices;
  mutable std::vector<float> vertex_spheres;
  mutable std::vector<float> normal_spheres;
  mutable std::vector<float> center_spheres;
  char mode;
  CGAL::Three::Scene_item::ManipulatedFrame* frame;
  mutable int nb_pos;
  mutable QOpenGLShaderProgram *program;
  //Fills the buffers with data. The buffers allow us to give data to the shaders.
  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const
  {
    //vao containing the data for the facets
    {
      program = getShaderProgram(PROGRAM_NO_SELECTION, viewer);
      program->bind();
      vaos[0]->bind();
      buffers[0].bind();
      buffers[0].allocate(vertices.data(),
                          static_cast<GLsizei>(vertices.size()*sizeof(float)));
      program->enableAttributeArray("vertex");
      program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
      buffers[0].release();
      vaos[0]->release();
      program->release();
      //once the buffers are fill, we can empty the vectors to optimize memory consumption
      nb_pos = vertices.size();
      vertices.resize(0);
      vertices.shrink_to_fit();
    }
    program = getShaderProgram(PROGRAM_SPHERES, viewer);
    program->bind();
    vaos[1]->bind();
    buffers[1].bind();
    buffers[1].allocate(vertex_spheres.data(),
                              static_cast<int>(vertex_spheres.size()*sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3);
    buffers[1].release();
    
    buffers[2].bind();
    buffers[2].allocate(normal_spheres.data(),
                              static_cast<int>(normal_spheres.size()*sizeof(float)));
    program->enableAttributeArray("normals");
    program->setAttributeBuffer("normals", GL_FLOAT, 0, 3);
    buffers[2].release();
    
    buffers[3].bind();
    buffers[3].allocate(center_spheres.data(),
                              static_cast<int>(center_spheres.size()*sizeof(float)));
    program->enableAttributeArray("center");
    program->setAttributeBuffer("center", GL_FLOAT, 0, 3);
    buffers[3].release();
    program->disableAttributeArray("radius");
    program->disableAttributeArray("colors");
    
    viewer->glVertexAttribDivisor(program->attributeLocation("center"), 1);
    viewer->glVertexAttribDivisor(program->attributeLocation("radius"), 1);
    vaos[1]->release();
    program->release();
    are_buffers_filled = true;
  }
}; //end of class Scene_triangle_item

class Project_against_mesh_plugin :
    public QObject,
    public Polyhedron_demo_plugin_helper
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")
  
public:
  //decides if the plugin's actions will be displayed or not.
  bool applicable(QAction* action) const Q_DECL_OVERRIDE
  {
    return (action==actionCreateSurface && create_surface_item==NULL && scene->numberOfEntries() > 0)
        ||(action==actionProjectSurface &&
           create_surface_item && qobject_cast<Scene_surface_mesh_item*>(scene->item(scene->mainSelectionIndex())));
  }
  
  //the list of the actions of the plugin.
  QList<QAction*> actions() const Q_DECL_OVERRIDE
  {
    return _actions;
  }
  //this acts like a constructor for the plugin. It gets the references to the main window and the scene, and connects the action.
  void init(QMainWindow* mainWindow, CGAL::Three::Scene_interface* sc, Messages_interface* mi) Q_DECL_OVERRIDE
  {
    //gets the reference to the message interface, to display text in the console widget
    this->messageInterface = mi;
    //get the references
    this->scene = sc;
    connect(static_cast<Scene*>(scene), SIGNAL(itemAboutToBeDestroyed(CGAL::Three::Scene_item*)),
            this, SLOT(kill_item(CGAL::Three::Scene_item*)));
    this->mw = mainWindow;
    surface_item = NULL;
    create_surface_item = NULL;
    //creates the action
    actionCreateSurface = new QAction(QString("Create the Surface"), mw);
    actionProjectSurface = new QAction(QString("Project the Surface"), mw);
    //specifies the subMenu
    actionCreateSurface->setProperty("submenuName", "SandBox");
    actionProjectSurface->setProperty("submenuName", "SandBox");
    //links the action
    
    connect(actionCreateSurface, SIGNAL(triggered()),
            this, SLOT(createSurface()));
    connect(actionProjectSurface, SIGNAL(triggered()),
            this, SLOT(projectSurface()));
    _actions << actionCreateSurface << actionProjectSurface;
  }
private Q_SLOTS:
  void kill_item(CGAL::Three::Scene_item* item)
  {
    if(item == create_surface_item)
      create_surface_item = NULL;
    else if(item==surface_item)
      surface_item = NULL;
  }
  void createSurface()
  {
    create_surface_item = new Scene_create_surface_item(Point_3(
                                                   scene->bbox().min(0),
                                                   scene->bbox().min(1),
                                                   (scene->bbox().max(2) + scene->bbox().min(2))/2),
                                                 Point_3(
                                                   scene->bbox().max(0),
                                                   scene->bbox().max(1),
                                                   (scene->bbox().max(2) + scene->bbox().min(2))/2));
    scene->addItem(create_surface_item);
  }
  
  void projectSurface()
  {
    Scene_surface_mesh_item* item =
        qobject_cast<Scene_surface_mesh_item*>(scene->item(scene->mainSelectionIndex()));
    if(!item)
      return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    surface_item = new Scene_surface_mesh_item();
    SMesh& mesh = *surface_item->face_graph();
    typedef boost::graph_traits<SMesh>::vertex_descriptor vertex_descriptor;    
    typename boost::property_map< SMesh,CGAL::vertex_point_t>::type vpmap
        = get(CGAL::vertex_point, mesh);
    SMesh& tmesh =*item->face_graph();
    typename boost::property_map< SMesh,CGAL::vertex_point_t>::type t_vpmap
        = get(CGAL::vertex_point, tmesh);
    typedef boost::graph_traits<SMesh>::vertex_descriptor vertex_descriptor;
    typedef boost::graph_traits<SMesh>::halfedge_descriptor halfedge_descriptor;
    typedef boost::graph_traits<SMesh>::edge_descriptor edge_descriptor;
    typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
    GLdouble matrix[16];
    create_surface_item->manipulatedFrame()->getMatrix(matrix);
    QMatrix4x4 trans_mat;
    for(int i=0; i<16; ++i)
    {
      trans_mat.data()[i] = (float)matrix[i];
    }
    Point_3 points[4];
    Point_3 transformed_points[4];
    vertex_descriptor v[4];
    points[0] = Point_3(create_surface_item->getP1().x(),
                        create_surface_item->getP1().y(),
                        create_surface_item->getP1().z());
    
    points[1] = Point_3(create_surface_item->getP2().x(),
                        create_surface_item->getP1().y(),
                        create_surface_item->getP1().z());
    
    points[2] = Point_3(create_surface_item->getP2().x(),
                        create_surface_item->getP2().y(),
                        create_surface_item->getP1().z());
    
    points[3] = Point_3(create_surface_item->getP1().x(),
                        create_surface_item->getP2().y(),
                        create_surface_item->getP1().z());
    for(int i=0; i<4; ++i){
      QVector3D pos(points[i].x(),
                    points[i].y(),
                    points[i].z());
      QVector3D transformed_pos = trans_mat*pos;
      transformed_points[i] = Point_3(transformed_pos.x(),
                                     transformed_pos.y(),
                                     transformed_pos.z());
      v[i] = mesh.add_vertex(transformed_points[i]);
    }
    QVector3D dir = trans_mat * QVector3D(0,0,-1);
    mesh.add_face(v[0],v[1],v[2]);
    mesh.add_face(v[0],v[2],v[3]);
    surface_item->compute_bbox();
    QApplication::restoreOverrideCursor();
    double el=QInputDialog::getDouble(mw, "Edge length", "Enter Target Edge Length", 0.03*surface_item->diagonalBbox(),
                                      0, 2147483647, 17);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    //get border of the patch and get plane's equation
    halfedge_descriptor border_hd;
    std::vector<halfedge_descriptor> patch_border;
    BOOST_FOREACH(halfedge_descriptor hd, halfedges(mesh))
    {
      if(is_border(hd, mesh))
      {
        border_hd = hd;
        break;
      }
    }
    
    EPICK::Plane_3 proj_plane(transformed_points[0],
                              transformed_points[1],
                              transformed_points[2]);
    std::vector<EPICK::Point_2> proj_border;
    BOOST_FOREACH(halfedge_descriptor hd, halfedges_around_face(border_hd, mesh))
    {
      patch_border.push_back(hd);
      proj_border.push_back(proj_plane.to_2d(proj_plane.projection(get(vpmap, target(hd, mesh)))));
    }
    CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(mesh),
                                                       el,
                                                       mesh,
                                                       CGAL::Polygon_mesh_processing::parameters::all_default());
    typedef CGAL::AABB_face_graph_triangle_primitive<SMesh>  Facet_primitive;
    typedef CGAL::AABB_traits<EPICK, Facet_primitive>        Facet_traits;
    typedef CGAL::AABB_tree<Facet_traits>                    Facet_tree;
    typedef boost::optional<Facet_tree::Intersection_and_primitive_id<EPICK::Ray_3>::Type> Ray_intersection;
    
    Facet_tree tree;
    tree.insert(faces(*item->face_graph()).first,
                faces(*item->face_graph()).second,
                *item->face_graph());
    tree.build();
    //collect faces intersected by patch and remove them:
    //use the technique in selection plugin with lasso:
    //home/gimeno/CGAL/Polyhedron/demo/Polyhedron/Scene_polyhedron_item_k_ring_selection.h : l.257
    std::set<face_descriptor> rm_faces;
    BOOST_FOREACH(face_descriptor f, faces(tmesh))
    {
      bool is_vertex_inside = false;
      BOOST_FOREACH(vertex_descriptor v, CGAL::vertices_around_face(halfedge(f, tmesh), tmesh))
      {
        //project v on da plane
        EPICK::Point_2 proj_p = proj_plane.to_2d(proj_plane.projection(get(t_vpmap, v)));
        //if projected point is inside the patch, add its adjacent faces
        if (CGAL::bounded_side_2(proj_border.begin(),
                                 proj_border.end(),
                                 proj_p,
                                 EPICK())  == CGAL::ON_BOUNDED_SIDE)
        {
          BOOST_FOREACH(face_descriptor fat, faces_around_target(halfedge(v, tmesh), tmesh))
              rm_faces.insert(fat);
          is_vertex_inside = true;
        }
      }
      if(!is_vertex_inside) //else if triangle intersects patch border, remove face
      {
        for(std::size_t id = 0; id <proj_border.size(); ++id)
        {
          EPICK::Segment_2 border_seg(proj_border[id], proj_border[(id+1)%proj_border.size()]);
          halfedge_descriptor bobby = halfedge(f, tmesh);
          EPICK::Triangle_2 tri(
                proj_plane.to_2d(proj_plane.projection(get(t_vpmap, target(bobby, tmesh)))),
                proj_plane.to_2d(proj_plane.projection(get(t_vpmap, target(prev(bobby, tmesh), tmesh)))),
                proj_plane.to_2d(proj_plane.projection(get(t_vpmap, target(prev(prev(bobby, tmesh), tmesh), tmesh)))));
          if(CGAL::do_intersect(tri, border_seg))
          {
            rm_faces.insert(f);
            break;
          }
        }
      }
    }
    //get border edges
    CGAL::Face_filtered_graph<SMesh> patch_zone(tmesh, rm_faces);
    std::vector<edge_descriptor> border_edges;
    BOOST_FOREACH(edge_descriptor ed, edges(patch_zone))
    {
      if(is_border(ed, patch_zone))
        border_edges.push_back(ed);
    }
    
    //project patch
    BOOST_FOREACH(vertex_descriptor vd, vertices(mesh))
    {
      EPICK::Ray_3 ray(mesh.point(vd), Facet_traits::Vector_3(dir.x(), dir.y(), dir.z()));
      Ray_intersection intersection = tree.first_intersection(ray);
      if(intersection){
        if(boost::get<Point_3>(&(intersection->first))){
          const Point_3* p =  boost::get<Point_3>(&(intersection->first) );
          put(vpmap, vd, *p);
        }
      }
    }
    //rm zone faces
    BOOST_FOREACH(face_descriptor f, rm_faces)
      CGAL::Euler::remove_face(halfedge(f, tmesh), tmesh);
    //merge meshes
    //insert patch in mesh: 
    // iterate opposite-border edges and create new face/edges with closest vertex in patch
    
    mesh.collect_garbage();
    tmesh.collect_garbage();
    item->invalidateOpenGLBuffers();
    item->itemChanged();
    scene->addItem(surface_item);
    scene->erase(scene->item_id(create_surface_item));
    QApplication::restoreOverrideCursor();
  }
  
private:
  QList<QAction*> _actions;
  Messages_interface* messageInterface;
  //The reference to the scene
  CGAL::Three::Scene_interface* scene;
  //The reference to the main window
  QMainWindow* mw;
  QAction* actionCreateSurface;
  QAction* actionProjectSurface;
  Scene_create_surface_item* create_surface_item;
  Scene_surface_mesh_item* surface_item;
};
#include "Project_against_mesh_plugin.moc"
