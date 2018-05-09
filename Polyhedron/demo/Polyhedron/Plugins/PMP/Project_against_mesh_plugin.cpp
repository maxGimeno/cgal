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

#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>

#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_2_projection_traits_3.h>

#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>

#include <boost/bimap.hpp>

#include "Messages_interface.h"
#include "Scene_surface_mesh_item.h"
#include "Scene_polylines_item.h"
#include "create_sphere.h"
#include "Scene.h"

#include "triangulate_primitive.h"
using namespace CGAL::Three;
namespace Euler=CGAL::Euler;
typedef boost::graph_traits<SMesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<SMesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<SMesh>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
struct Is_selected_edge_property_map{
  typedef boost::property_map<SMesh,boost::edge_index_t>::type EImap;

  std::vector<bool>* is_selected_ptr;
  EImap* edge_index_map;
  Is_selected_edge_property_map()
    : is_selected_ptr(NULL), edge_index_map(NULL) {}
  Is_selected_edge_property_map(std::vector<bool>& is_selected, EImap* map)
    : is_selected_ptr( &is_selected), edge_index_map(map)
  {}

  std::size_t id(edge_descriptor ed) {
    return get(*edge_index_map, ed);
  }

  friend bool get(Is_selected_edge_property_map map, edge_descriptor ed)
  {
    CGAL_assertion(map.is_selected_ptr!=NULL);
    return (*map.is_selected_ptr)[map.id(ed)];
  }

  friend void put(Is_selected_edge_property_map map, edge_descriptor ed, bool b)
  {
    CGAL_assertion(map.is_selected_ptr!=NULL);
    (*map.is_selected_ptr)[map.id(ed)]=b;
  }
};

class Scene_create_surface_item : public CGAL::Three::Scene_item
{
  Q_OBJECT
  qglviewer::Vec center_;
public :
  Scene_create_surface_item(Point_3 p1, Point_3 p2)
    : p1(p1), p2(p2),
      frame(new CGAL::Three::Scene_item::ManipulatedFrame())
  {
    const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
    _bbox=Bbox(p1.x(), p1.y(), p1.z(),
               p2.x(), p2.y(), p2.z());
    center_ = qglviewer::Vec(
          (p1.x()+p2.x())/2.0,
          (p1.y()+p2.y())/2.0,
          (p1.z()+p2.z())/2.0);
    frame->setPosition( center_+offset);
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
    vertices[0] = p1.x()-center_.x;  vertices[3] = p2.x()-center_.x;    
    vertices[1] = p1.y()-center_.y;  vertices[4] = p1.y()-center_.y;
    vertices[2] = p1.z()-center_.z;  vertices[5] = p1.z()-center_.z;
    
    vertices[6] = p2.x()-center_.x;  vertices[9] =  p1.x()-center_.x;    
    vertices[7] = p2.y()-center_.y;  vertices[10] = p2.y()-center_.y;
    vertices[8] = p1.z()-center_.z;  vertices[11] = p1.z()-center_.z;
    
    vertices[12] = (p1.x()+p2.x())/2-center_.x;  vertices[15] = (p1.x()+p2.x())/2          -center_.x;    
    vertices[13] = (p1.y()+p2.y())/2-center_.y;  vertices[16] = (p1.y()+p2.y())/2          -center_.y;
    vertices[14] = p1.z()           -center_.z;  vertices[17] = p1.z() - 0.1*diagonalBbox()-center_.z;
    
    
    
    
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
  qglviewer::Vec center() const { return center_; }
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
}; //end of class Scene_create_surface_item

class Project_against_mesh_plugin :
    public QObject,
    public Polyhedron_demo_plugin_helper
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")
  
  typedef CGAL::Triangulation_2_projection_traits_3<EPICK>   P_traits;
  
  typedef CGAL::Triangulation_vertex_base_with_info_2<halfedge_descriptor,
  P_traits>        Vb;
  struct Face_info {
    typename boost::graph_traits<SMesh>::halfedge_descriptor e[3];
    bool is_external;
  };
  typedef CGAL::Triangulation_face_base_with_info_2<Face_info,
  P_traits>          Fb1;
  typedef CGAL::Constrained_triangulation_face_base_2<P_traits, Fb1>   Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                  TDS;
  typedef CGAL::Exact_predicates_tag                                   Itag;
  typedef CGAL::Constrained_Delaunay_triangulation_2<P_traits,
                                                      TDS,
                                                      Itag>             CDT;
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
                                                          (scene->bbox().max(2) + scene->bbox().min(2))/2.0),
                                                        Point_3(
                                                          scene->bbox().max(0),
                                                          scene->bbox().max(1),
                                                          (scene->bbox().max(2) + scene->bbox().min(2))/2.0));
    scene->setSelectedItem(
          scene->addItem(create_surface_item)
          );
  }
  void projectSurface()
  {
    
    Scene_surface_mesh_item* item =
        qobject_cast<Scene_surface_mesh_item*>(scene->item(scene->mainSelectionIndex()));
    if(!item)
      return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    SMesh mesh;
    typename boost::property_map< SMesh,CGAL::vertex_point_t>::type vpmap
        = get(CGAL::vertex_point, mesh);
    SMesh& tmesh =*item->face_graph();
    typename boost::property_map< SMesh,CGAL::vertex_point_t>::type t_vpmap
        = get(CGAL::vertex_point, tmesh);
    
    /********************************************************
     *** Collect corners of create_surface_item and plane ***
     ********************************************************/
     
    double matrix[16];
    create_surface_item->manipulatedFrame()->getMatrix(matrix);
    const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
    matrix[12]-=(offset.x);
    matrix[13]-=(offset.y);
    matrix[14]-=(offset.z);
    QMatrix4x4 transform_matrix;
    QMatrix4x4 rotate_matrix;
    QMatrix4x4 compensate;
    compensate.translate(-QVector3D(create_surface_item->center().x,
                         create_surface_item->center().y,
                         create_surface_item->center().z));
    for(int i=0; i<16; ++i)
      transform_matrix.data()[i] = (float)matrix[i];
    rotate_matrix = transform_matrix;
    rotate_matrix.setColumn(3, QVector4D(0,0,0,1));
    
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
      QVector3D transformed_pos = transform_matrix * (compensate*pos);
      transformed_points[i] = Point_3(transformed_pos.x(),
                                     transformed_pos.y(),
                                     transformed_pos.z());
    }
    EPICK::Plane_3 proj_plane(transformed_points[0],
                              transformed_points[1],
                              transformed_points[2]);
    QVector3D dir = rotate_matrix * QVector3D(0,0,-1);
    P_traits cdt_traits(proj_plane.orthogonal_vector());
    CDT cdt(cdt_traits);
    
    
    /***************************
     ** Get faces under patch **
     ***************************/
    
    std::vector<EPICK::Point_2> proj_border;
    for(int i=0; i<4; ++i)
    {
      v[i]=mesh.add_vertex(transformed_points[i]);
      proj_border.push_back(proj_plane.to_2d(transformed_points[i]));
    }
    mesh.add_face(v[0],v[1],v[2]);
    mesh.add_face(v[0],v[2],v[3]);
    
    typedef CGAL::AABB_face_graph_triangle_primitive<SMesh>  Facet_primitive;
    typedef CGAL::AABB_traits<EPICK, Facet_primitive>        Facet_traits;
    typedef CGAL::AABB_tree<Facet_traits>                    Facet_tree;
    typedef boost::optional<Facet_tree::Intersection_and_primitive_id<EPICK::Ray_3>::Type> Ray_intersection;
    
    Facet_tree tree;
    tree.insert(faces(*item->face_graph()).first,
                faces(*item->face_graph()).second,
                *item->face_graph());
    tree.build();
    //project faces on plane. From there, collect faces that are inside the patch for removal, 
    // and collect faces that intersect the patch border for re-triangulation.
    std::set<face_descriptor> rm_faces;
    std::set<face_descriptor> intersecting_faces;
    std::set<face_descriptor> temp_selection;
    std::set<face_descriptor> tmp_rm;
    std::set<face_descriptor> tmp_inter;
    
    //first, find faces of interest
    BOOST_FOREACH(face_descriptor f, faces(tmesh))
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
          temp_selection.insert(f);
          tmp_inter.insert(f);
          break;
        }
      }
      BOOST_FOREACH(vertex_descriptor v, CGAL::vertices_around_face(halfedge(f, tmesh), tmesh))
      {
        //project v on da plane
        EPICK::Point_2 proj_p_2D = proj_plane.to_2d(proj_plane.projection(get(t_vpmap, v)));
        
        if (CGAL::bounded_side_2(proj_border.begin(),
                                 proj_border.end(),
                                 proj_p_2D,
                                 EPICK())  == CGAL::ON_BOUNDED_SIDE)
        {
          temp_selection.insert(f);
          tmp_rm.insert(f);
          break;
        }
      }
    }
    // Un-select faces of the back
    std::vector<halfedge_descriptor> boundary_edges;
    CGAL::Polygon_mesh_processing::border_halfedges(temp_selection, tmesh, std::back_inserter(boundary_edges));
    std::vector<bool> mark(edges(tmesh).size(), false);
    boost::property_map<SMesh, boost::edge_index_t>::type edge_index
        = get(boost::edge_index, tmesh);
    Is_selected_edge_property_map spmap(mark, &edge_index);
    BOOST_FOREACH(halfedge_descriptor h, boundary_edges)
        put(spmap, edge(h, tmesh), true);
    
    boost::vector_property_map<int,
        boost::property_map<SMesh, boost::face_index_t>::type>
        fccmap;
    
    //get connected componant from the picked face
    std::size_t nb_cc = CGAL::Polygon_mesh_processing::connected_components(tmesh
                                                                            , fccmap
                                                                            , CGAL::Polygon_mesh_processing::parameters::edge_is_constrained_map(spmap));
    std::vector<bool> is_cc_done(nb_cc, false);
    
    BOOST_FOREACH(face_descriptor f, temp_selection)
    {
      int cc_id = get(fccmap, f);
      if(is_cc_done[cc_id])
      {
        continue;
      }
      CGAL::Halfedge_around_face_circulator<SMesh> hafc(halfedge(f, tmesh), tmesh);
      CGAL::Halfedge_around_face_circulator<SMesh> end = hafc;
      double x(0), y(0), z(0);
      int total(0);
      CGAL_For_all(hafc, end)
      {
        EPICK::Point_3 p = get(t_vpmap, target(*hafc, tmesh));
        x+=p.x(); y+=p.y(); z+=p.z();
        total++;
      }
      if(total == 0)
        continue;
      
      EPICK::Point_3  center(x/(double)total, y/(double)total, z/(double)total);
      EPICK::Point_3 orig = proj_plane.projection(center);
      EPICK::Vector_3 direction = center - orig;
      //if intersection 
      if(item->intersect_face(orig.x(),
                              orig.y(),
                              orig.z(),
                              direction.x(),
                              direction.y(),
                              direction.z(),
                              f))
      {
        is_cc_done[cc_id] = true;
      }
    }
   
    //end collect faces of interest
    
    /*****************************************
     ** now collect inside faces and border **
     ** intersecting faces                  **
     *****************************************/
    
    BOOST_FOREACH(face_descriptor f, tmp_rm)
    {
      if(is_cc_done[get(fccmap, f)])
        rm_faces.insert(f);
    }
    
    BOOST_FOREACH(face_descriptor f, tmp_inter)
    {
      if(is_cc_done[get(fccmap, f)])
      {
        intersecting_faces.insert(f);
        rm_faces.insert(f);
      }
    }
    
    /********************************************
     * remesh the quad for density and fill CDT *
     ********************************************/
    
    CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(mesh);  
    QApplication::restoreOverrideCursor();
    double el=QInputDialog::getDouble(mw, "Edge length", "Enter Target Edge Length", 0.03*CGAL::sqrt(
                                        (bbox.xmax()-bbox.xmin()) * (bbox.xmax()-bbox.xmin())+
                                        (bbox.ymax()-bbox.ymin()) * (bbox.ymax()-bbox.ymin())+
                                        (bbox.zmax()-bbox.zmin()) * (bbox.zmax()-bbox.zmin())
                                         ),
                                      0, 2147483647, 17);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(mesh),
                                                       el,
                                                       mesh);
    BOOST_FOREACH(vertex_descriptor vd, vertices(mesh))
      cdt.insert(get(vpmap, vd));
    //Add exterior edges of the intersecting faces to the cdt as constrained edges
    //and fill a bimap mesh::vd<->cdt::vh
    typedef boost::bimap<vertex_descriptor, CDT::Vertex_handle>  Vd2vhMap;
    typedef Vd2vhMap::value_type v_pair;
    Vd2vhMap vd2vh;
    Scene_polylines_item* line_item = new Scene_polylines_item();
    BOOST_FOREACH(face_descriptor f, intersecting_faces)
    {
      std::vector<EPICK::Point_3> proj_points;
      std::vector<vertex_descriptor> outside_vertices;
      BOOST_FOREACH(vertex_descriptor v, CGAL::vertices_around_face(halfedge(f, tmesh), tmesh))
      {
        //project v on da plane
         proj_points.push_back(proj_plane.projection(get(t_vpmap, v)));
        EPICK::Point_2 proj_p_2D = proj_plane.to_2d(proj_points.back());
        
        if (CGAL::bounded_side_2(proj_border.begin(),
                                 proj_border.end(),
                                 proj_p_2D,
                                 EPICK())  != CGAL::ON_BOUNDED_SIDE)
        {
          outside_vertices.push_back(v);
        }
        else
          proj_points.pop_back();
      }
      if(outside_vertices.size() > 1)
      {
        std::vector<CDT::Vertex_handle> vs(proj_points.size());
        for(std::size_t id = 0; id < vs.size(); ++id)
        {
          vs[id] = cdt.insert(proj_points[id]);
        }
        
        for(std::size_t id = 0; id < vs.size(); ++id)
        {
          vd2vh.insert(v_pair(outside_vertices[id], vs[id]));
          vd2vh.insert(v_pair(outside_vertices[(id+1)%vs.size()], vs[(id+1)%vs.size()]));
          //not if edge intersects border
          bool intersect_border = false;
          for(std::size_t bid = 0; bid <proj_border.size(); ++bid)
          {
            EPICK::Segment_2 border_seg(proj_border[bid], proj_border[(bid+1)%proj_border.size()]);
            EPICK::Segment_2 test_edge(proj_plane.to_2d(proj_points[id]),
                                       proj_plane.to_2d(proj_points[(id+1)%vs.size()]));
            if(do_intersect(border_seg, test_edge))
            {
              intersect_border = true;
              break;
            }
          }
          if(!intersect_border)
          {
            cdt.insert_constraint(vs[id],
                                  vs[(id+1)%vs.size()]);
            std::vector<EPICK::Point_3> edge;
            edge.push_back(get(t_vpmap, outside_vertices[id]));
            edge.push_back(get(t_vpmap, outside_vertices[(id+1)%vs.size()]));
            line_item->polylines.push_back(edge);
          }
        }
      }
    }
    
    for(typename CDT::All_faces_iterator
        fit2 = cdt.all_faces_begin(),
        end = cdt.all_faces_end();
        fit2 != end; ++fit2)
    {
      fit2->info().is_external = false;
    }
    //check if the facet is external or internal
    std::queue<typename CDT::Face_handle> face_queue;
    face_queue.push(cdt.infinite_vertex()->face());
    while(! face_queue.empty() ) {
      typename CDT::Face_handle fh = face_queue.front();
      face_queue.pop();
      if(fh->info().is_external) continue;
      fh->info().is_external = true;
      for(int i = 0; i <3; ++i) {
        if(!cdt.is_constrained(std::make_pair(fh, i)))
        {
          face_queue.push(fh->neighbor(i));
        }
      }
    }
    
    /****************************************************
     * Inject CDT in mesh and replace faces under patch *
     ****************************************************/

    //iterate CDT : if vh is in map, do nothing. 
    //Else, project point on tmesh, add_vertex(vh->point()) and add new vertex in map.
    for(CDT::Finite_vertices_iterator fit = cdt.finite_vertices_begin();
        fit != cdt.finite_vertices_end();
        ++fit)
    {
      if(vd2vh.right.find(fit) == vd2vh.right.end())
      {
        EPICK::Ray_3 ray(fit->point(), EPICK::Vector_3(dir.x(), dir.y(), dir.z()));
        Ray_intersection intersection = tree.first_intersection(ray);
        if(intersection){
          if(boost::get<EPICK::Point_3>(&(intersection->first))){
            const EPICK::Point_3* p = boost::get<EPICK::Point_3>(&(intersection->first));
            vertex_descriptor new_v = CGAL::add_vertex(tmesh);
            put(t_vpmap, new_v, *p);
            vd2vh.insert(v_pair(new_v,fit));
          }
        }
        else
        {
          QApplication::restoreOverrideCursor();
          QMessageBox::warning(mw, "Error", "Some points projected in the void. Aborting.");
          if(line_item)
            delete line_item;
          return;
        }
      }
    }
    //now, remove faces
    BOOST_FOREACH(face_descriptor f, rm_faces)
    {
      CGAL::Euler::remove_face(halfedge(f, tmesh), tmesh);
    }
    //add faces from CDT if not external
    mesh.clear();
    for(CDT::Finite_faces_iterator
         fit = cdt.finite_faces_begin();
         fit != cdt.finite_faces_end();
         ++fit)
    {
      if(fit->info().is_external)
        continue;
      std::vector<vertex_descriptor> new_face(3);
      for(int i=0; i<3; ++i)
        new_face[i] = mesh.add_vertex(get(t_vpmap, vd2vh.right.at(fit->vertex(i))));
      mesh.add_face(new_face);
    }
    
    /***************************
     * Merge meshes and stitch *
     ***************************/
    
    CGAL::Polygon_mesh_processing::remove_isolated_vertices(tmesh);
    tmesh.collect_garbage();
    CGAL::copy_face_graph(mesh, tmesh);
    tmesh.collect_garbage();
    CGAL::Polygon_mesh_processing::stitch_borders(tmesh);
    item->invalidateOpenGLBuffers();
    item->itemChanged();
    line_item->setName("Constraint edges");
    line_item->setColor(QColor(Qt::red));
    scene->addItem(line_item);
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
