//#define CGAL_SAMPLED
#include <QApplication>
#include <QObject>
#include <QAction>
#include <QMainWindow>
#include <QInputDialog>
#include <QMessageBox>
#include <QMouseEvent>

#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include "CGAL/Three/Scene_interface.h"
#include "CGAL/Three/Polyhedron_demo_plugin_helper.h"
#include "CGAL/Three/Viewer_interface.h"
#include <CGAL/Surface_mesh_deformation.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/squared_distance_3.h>
#include <CGAL/boost/graph/Euler_operations.h>

#include "Messages_interface.h"
#include "Scene_polylines_item.h"
#include "Scene_polyhedron_item.h"
#include "Scene_points_with_normal_item.h"
#include "Plugins/Mesh_3/Volume_plane_interface.h"
#include "ui_Create_surface.h"

typedef Scene_polylines_item::Point_3 Point_3;
typedef Polyhedron::Traits Kernel;
// A modifier creating a Polyhedron with the incremental builder.
template <class HDS>
class Build_polyhedron : public CGAL::Modifier_base<HDS> {
  std::vector<Point_3> points;
  int size_I;
  int size_J;
public:
  Build_polyhedron(const std::vector<Point_3> &p, int sizeI, int sizeJ)
    :points(p),size_I(sizeI), size_J(sizeJ){}
  int vertex(int i, int j)
  {
    return size_I*j+i;
  }
  void operator()( HDS& hds)
  {
    // Postcondition: hds is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
    B.begin_surface( size_I*size_J, (size_I-1)*(size_J-1), 0);
    for(int i = 0; i<size_I*size_J; ++i)
      B.add_vertex(points[i]);
    for(int i = 0; i<size_I-1; ++i)
      for(int j = 0; j<size_J-1; ++j)
      {
        B.begin_facet();
        B.add_vertex_to_facet(vertex(i,j));
        B.add_vertex_to_facet(vertex(i,j+1));
        B.add_vertex_to_facet(vertex(i+1,j));
        B.end_facet();
        B.begin_facet();
        B.add_vertex_to_facet(vertex(i+1, j));
        B.add_vertex_to_facet(vertex(i, j+1));
        B.add_vertex_to_facet(vertex(i+1, j+1));
        B.end_facet();
      }
    B.end_surface();
  }
};

class SurfaceFromPickedPointsPlugin :
    public QObject,
    public CGAL::Three::Polyhedron_demo_plugin_helper
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")

public:
  //decides if the plugin's actions will be displayed or not.
  bool applicable(QAction*) const
  {
    return scene->numberOfEntries() >0;
  }
  //the list of the actions of the plugin.
  QList<QAction*> actions() const
  {
    return _actions;
  }
  //this acts like a constructor for the plugin. It gets the references to the mainwindow and the scene, and connects the action.
  void init(QMainWindow* mainWindow, CGAL::Three::Scene_interface* sc, Messages_interface* mi)
  {
    //gets the reference to the message interface, to display text in the console widget
    this->messageInterface = mi;
    //get the references
    this->scene = sc;
    this->mw = mainWindow;
    //creates the action
    QAction *actionGenerateSurface= new QAction(QString("Generate Surface from Picked Points"), mw);
    //specifies the subMenu
    actionGenerateSurface->setProperty("submenuName", "Triangulated Surface Mesh Deformation");
    //links the action
    connect(actionGenerateSurface, SIGNAL(triggered()),
            this, SLOT(pick()));
    _actions << actionGenerateSurface;
    dock_widget = new QDockWidget("Create and Deform Surface", mw);
    dock_widget->setVisible(false);
    ui_widget.setupUi(dock_widget);
    connect(dock_widget, &QDockWidget::visibilityChanged,
            this, &SurfaceFromPickedPointsPlugin::dockwidgetVisibilityChanged);
    connect(ui_widget.createSurfaceButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::add_surface);
    connect(ui_widget.newPolylineButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::add_polyline);
    connect(ui_widget.finishButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::finish);
    connect(ui_widget.cancelButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::cancel);
    connect(ui_widget.leaderSpinBox, SIGNAL(valueChanged(double)),
            this, SLOT(setDeltaI(double)));
    connect(ui_widget.generatorSpinBox, SIGNAL(valueChanged(double)),
            this, SLOT(setDeltaJ(double)));
    is_active = false;
    mode = IDLE;
    ui_widget.cancelButton->setEnabled(false);
    QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
    viewer->installEventFilter(this);
    mw->installEventFilter(this);
  }
  bool eventFilter(QObject *, QEvent *event);
private Q_SLOTS:
  void pick()
  {
    // show/hide the dock widget
    if(dock_widget->isVisible()) { dock_widget->hide(); }
    else                         { dock_widget->show(); }

  }
  void closure()
  {
    dock_widget->hide();
  }
  void dockwidgetVisibilityChanged(bool b)
  {
    is_active = b;
    if(b)
      leader_is_created = false;
    mode = IDLE;
    ui_widget.cancelButton->setEnabled(false);
    leader_poly = NULL;
    generator_poly = NULL;
    surface = NULL;
  }
  void add_polyline()
  {
    mode = ADD_POLYLINE;
    ui_widget.cancelButton->setEnabled(true);
    if(!leader_is_created)
    {
      if(generator_poly)
      {
        generator_poly = NULL;
      }
      leader_poly = new Scene_polylines_item();
      leader_poly->polylines.push_back( Scene_polylines_item::Polyline() );
      leader_poly->setName("Leader Polyline");
      scene->addItem(leader_poly);
    }
    else
    {
      generator_poly = new Scene_polylines_item();
      generator_poly->polylines.push_back( Scene_polylines_item::Polyline() );
      if(!leader_poly->polylines.back().empty())
        generator_poly->polylines.back().push_back(leader_poly->polylines.back().front());
      generator_poly->setName("Generator Polyline");
      scene->addItem(generator_poly);
    }
    leader_is_created = !leader_is_created;
    min_dist = -1;
  }
  void add_surface()
  {
    if(!(leader_poly && generator_poly))
      return;
    bool plane_found = false;
    for(int i=0; i<scene->numberOfEntries(); ++i)
    {
      //always return true
      Volume_plane_interface* plane = static_cast<Volume_plane_interface*>(scene->item(i));
      if(plane)
      {
        plane_found = true;
        break;
      }
    }
    if(plane_found)
    {
      mode = ADD_POINT_AND_DEFORM;
    }
    else
    {
      mode = IDLE;
    }
    ui_widget.cancelButton->setEnabled(false);
    //create points
    std::vector<Point_3> points;
    std::vector<Point_3>& l_polyline = leader_poly->polylines.back();
    std::vector<Point_3>& g_polyline = generator_poly->polylines.back();
    std::vector<Point_3>::iterator pit1 = l_polyline.begin();
    std::vector<Point_3>::iterator pit2 = pit1+1;
    std::vector<Point_3> control_pos;
#ifdef CGAL_SAMPLED
    //keep the original points in memory
    BOOST_FOREACH(Point_3 p, l_polyline)
    {
      control_pos.push_back(p);
    }
    for(std::size_t i=1; i<g_polyline.size(); ++i)
    {
      control_pos.push_back(g_polyline[i]);
    }
#endif
    //break the polylines into smaller pieces according to the specified deltas
    while(pit2 != l_polyline.end())
    {
      Kernel::Vector_3 v = *pit2 - *pit1;
      double dist = CGAL::sqrt(v.squared_length());
      while(dist > deltaI)
      {
        Point_3 mid_point = *pit1+v/2;
        pit2 = l_polyline.insert(pit2, mid_point);
        pit1 = pit2-1;
        v = *pit2 - *pit1;
        dist = CGAL::sqrt(v.squared_length());
      }
      ++pit1;
      ++pit2;
    }
    leader_poly->invalidateOpenGLBuffers();
    leader_poly->itemChanged();
    pit1 = g_polyline.begin();
    pit2 = pit1+1;
    while(pit2 != g_polyline.end())
    {
      Kernel::Vector_3 v = *pit2 - *pit1;
      double dist = CGAL::sqrt(v.squared_length());
      while(dist > deltaJ)
      {
        Point_3 mid_point = *pit1+v/2;
        pit2 = g_polyline.insert(pit2, mid_point);
        pit1 = pit2-1;
        v = *pit2 - *pit1;
        dist = CGAL::sqrt(v.squared_length());
      }
      ++pit1;
      ++pit2;
    }
    generator_poly->invalidateOpenGLBuffers();
    generator_poly->itemChanged();

#ifndef CGAL_SAMPLED
    //keep the points in memory
    BOOST_FOREACH(Point_3 p, l_polyline)
    {
      control_pos.push_back(p);
    }
    for(std::size_t i=1; i<g_polyline.size(); ++i)
    {
      control_pos.push_back(g_polyline[i]);
    }
#endif
    //compute the points
    for(std::size_t i=0; i< l_polyline.size(); ++i)
    {
      Kernel::Vector_3 offset = l_polyline[i]-l_polyline[0];
      for(std::size_t j=0; j<g_polyline.size(); ++j)
      {
        if(j==0)
          points.push_back(l_polyline[i]);
        else
           points.push_back(g_polyline[j]+offset);
      }
    }
    point_item = new Scene_points_with_normal_item();
    BOOST_FOREACH(Point_3 p, points)
        point_item->point_set()->insert(p);
    point_item->setName("Points");
    scene->addItem(point_item);
    //create Polyhedron item
    Polyhedron *polyhedron = new Polyhedron();
    Build_polyhedron<Polyhedron::HalfedgeDS> builder(points, (int)g_polyline.size(), (int)l_polyline.size());
    polyhedron->delegate(builder);

    //save the points of the initial polylines as control points
    polyhedron->normalize_border();
    Polyhedron::Halfedge_iterator vit = polyhedron->border_halfedges_begin(),
        end = vit;
    do
    {
      int i=0;
      BOOST_FOREACH(Point_3 p, control_pos)
      {
        if(target(vit, *polyhedron)->point() == p)
        {
          control_points.push_back(Polyhedron::Vertex_handle(target(vit, *polyhedron)));
          control_pos.erase(control_pos.begin()+i);
          break;
        }
        ++i;
      }
      if(control_pos.empty())
        break;
      ++vit;
    }while(vit != end);
    surface = new Scene_polyhedron_item(polyhedron);
    surface->setName("Surface");
    scene->addItem(surface);
  }
  void finish()
  {
    mode = IDLE;
    ui_widget.cancelButton->setEnabled(false);
    leader_poly = NULL;
  }
  void cancel()
  {
    if(mode != ADD_POLYLINE)
      return;
    Scene_polylines_item* current_polyline = NULL;
    QDoubleSpinBox* current_spin = NULL;
    if(leader_is_created)
    {
      current_polyline = leader_poly;
      current_spin = ui_widget.leaderSpinBox;
    }
    else
    {
      current_polyline = generator_poly;
      current_spin = ui_widget.generatorSpinBox;
    }
    std::vector<Point_3>& polyline = current_polyline->polylines.back();
    if(polyline.size() >= 2)
    {
      polyline.pop_back();
      current_polyline->invalidateOpenGLBuffers();
      current_polyline->itemChanged();
      min_dist = -1;
      for(std::size_t i=0; i<polyline.size()-1; ++i)
      {
        Kernel::Vector_3 v = polyline[i+1] - polyline[i];
        double dist = CGAL::sqrt(v.squared_length());
        if(min_dist == -1 || dist<min_dist)
        {
          min_dist = dist;
          current_spin->setMaximum(min_dist);
          current_spin->setValue(min_dist/2.0);
          current_spin->setSingleStep(min_dist/100);
        }
      }
    }
  }
  void setDeltaI(double d) { deltaI = d; }
  void setDeltaJ(double d) { deltaJ = d; }
private:
  enum Select_mode{
    ADD_POLYLINE = 0,
    ADD_POINT_AND_DEFORM,
    IDLE
  };
  QList<QAction*> _actions;
  Messages_interface* messageInterface;
  //The reference to the scene
  CGAL::Three::Scene_interface* scene;
  //The reference to the main window
  QMainWindow* mw;
  QDockWidget* dock_widget;
  Ui::Create_surface ui_widget;
  bool is_active;
  Select_mode mode;
  Scene_polylines_item* leader_poly;
  Scene_polylines_item* generator_poly;
  Scene_polyhedron_item* surface;
  Scene_points_with_normal_item* point_item;
  bool leader_is_created;
  double deltaI;
  double deltaJ;
  double min_dist;
  std::vector<Polyhedron::Vertex_handle > control_points;
};


bool SurfaceFromPickedPointsPlugin::eventFilter(QObject *object, QEvent *event)
{
  if (!is_active || mode == IDLE)
    return false;
  if(event->type()==QEvent::MouseButtonPress)
  {
    QMouseEvent* e= static_cast<QMouseEvent*>(event);
    if(e->modifiers() == Qt::ShiftModifier &&
       e->buttons() == Qt::LeftButton)
    {
      QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
      if(object == mw)
      {
        viewer->setFocus();
        return false;
      }
      bool found = false;
      qglviewer::Vec point = viewer->camera()->pointUnderPixel(e->pos(), found);
      switch(mode)
      {
      case ADD_POLYLINE:
      {
        Scene_polylines_item* current_polyline = NULL;
        QDoubleSpinBox* current_spin = NULL;
        if(leader_is_created)
        {
          current_polyline = leader_poly;
          current_spin = ui_widget.leaderSpinBox;
        }
        else
        {
          current_polyline = generator_poly;
          current_spin = ui_widget.generatorSpinBox;
        }
        if(found) {
          std::vector<Point_3>& polyline = current_polyline->polylines.back();
          polyline.push_back(Point_3(point.x, point.y, point.z));
          if(polyline.size() >= 2)
          {
            Kernel::Vector_3 v = polyline[polyline.size()-1] - polyline[polyline.size()-2];
            double dist = CGAL::sqrt(v.squared_length());
            if(min_dist == -1 || dist<min_dist)
            {
              min_dist = dist;
              current_spin->setMaximum(min_dist);
              current_spin->setValue(min_dist/2.0);
              current_spin->setSingleStep(min_dist/100);
            }
          }
          current_polyline->invalidateOpenGLBuffers();
          current_polyline->itemChanged();
        }
        break;
      }
      case ADD_POINT_AND_DEFORM:
      {
        typedef CGAL::Surface_mesh_deformation<Polyhedron>              Surface_mesh_deformation;
        typedef CGAL::AABB_halfedge_graph_segment_primitive<Polyhedron> HGSP;
        typedef CGAL::AABB_traits<Kernel, HGSP>                         AABB_traits;
        typedef CGAL::AABB_tree<AABB_traits>                            AABB_tree;

        Volume_plane_interface* plane_interface = NULL;
        viewer->select(e);
        if(strcmp(scene->item(scene->mainSelectionIndex())->metaObject()->className(),"Volume_plane_interface") == 0)
          plane_interface = static_cast<Volume_plane_interface*>(scene->item(scene->mainSelectionIndex()));
        if(!plane_interface)
          return false;
        qglviewer::AxisPlaneConstraint* constraint = static_cast<qglviewer::AxisPlaneConstraint*>(plane_interface->manipulatedFrame()->constraint());
        if(!constraint)
        {
          return false;
        }
        const qglviewer::Vec& pos = plane_interface->manipulatedFrame()->position();
        const qglviewer::Vec& n =constraint->translationConstraintDirection();

        Kernel::Plane_3 plane(n[0], n[1],  n[2], - n * pos);
        Polyhedron &polyhedron = *surface->polyhedron();
        // Init the indices of the halfedges and the vertices.
        set_halfedgeds_items_id(polyhedron);
        //Slice the Polyhedron along the picked plane
        std::vector<std::vector<Point_3> > slices;
        AABB_tree tree(edges(polyhedron).first, edges(polyhedron).second, polyhedron);
        CGAL::Polygon_mesh_slicer<Polyhedron, Kernel> slicer_aabb(polyhedron, tree);
        slicer_aabb(plane, std::back_inserter(slices));
        if(slices.empty())
          return false;
        //find the closest slice
        boost::tuple<double, int, int> min_squared_dist(-1,-1, -1);
        for(std::size_t i = 0; i<slices.size(); ++i)
        {
          for(std::size_t j = 0; j<slices[i].size()-1; ++j)
          {
            Kernel::Segment_3 segment(slices[i][j], slices[i][j+1]);
            double sq_dist = CGAL::squared_distance(Point_3(point.x, point.y, point.z), segment);
            if(min_squared_dist.get<0>() == -1 ||
               sq_dist < min_squared_dist.get<0>())
            {
              min_squared_dist.get<0>()= sq_dist;
              min_squared_dist.get<1>() = i;
              min_squared_dist.get<2>() = j;
            }
          }
        }
        //find the edges intersected by this slice
        AABB_traits::Primitive::Id pid1 = tree.closest_point_and_primitive(slices[min_squared_dist.get<1>()][min_squared_dist.get<2>()]).second;
        AABB_traits::Primitive::Id pid2 = tree.closest_point_and_primitive(slices[min_squared_dist.get<1>()][min_squared_dist.get<2>()+1]).second;
        Polyhedron::Halfedge_handle h1(halfedge(pid1, polyhedron));
        Polyhedron::Halfedge_handle h2(halfedge(pid2, polyhedron));
        //find the triangle that contains those two edges
        Polyhedron::Facet_handle closest_triangle = h1->facet();
        if(!(closest_triangle == h2->facet()
             || closest_triangle == h2->opposite()->facet()))
        {
          closest_triangle = h1->opposite()->facet();
        }

        // add triangle's center to the mesh
        double x(0), y(0), z(0);
        Polyhedron::Halfedge_around_facet_circulator hafc = closest_triangle->facet_begin();
        Polyhedron::Halfedge_around_facet_circulator end = hafc;
        CGAL_For_all(hafc, end)
        {
          x+=hafc->vertex()->point().x(); y+=hafc->vertex()->point().y(); z+=hafc->vertex()->point().z();
        }
        Polyhedron::Halfedge_handle center = CGAL::Euler::add_center_vertex(closest_triangle->facet_begin(), polyhedron);
        center->vertex()->point() = Point_3(x/3.0, y/3.0, z/3.0);
        // Init the indices of the halfedges and the vertices.
        set_halfedgeds_items_id(polyhedron);
        Surface_mesh_deformation deform_mesh(polyhedron);
        //Define the ROI
        for(Polyhedron::Vertex_iterator vit = polyhedron.vertices_begin();
            vit != polyhedron.vertices_end();
            ++vit)
        {
          deform_mesh.insert_roi_vertex(vit);
        }
        control_points.push_back(target(center, polyhedron));
        //add the control points

        /*BOOST_FOREACH(Polyhedron::Vertex_handle vh, control_points)
        {
          deform_mesh.erase_roi_vertex(vh);
        }*/
        deform_mesh.insert_control_vertices(control_points.begin(), control_points.end());
        //deform
        bool is_matrix_factorization_OK = deform_mesh.preprocess();
        if(!is_matrix_factorization_OK){
          std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
          return false;
        }
        Surface_mesh_deformation::Point constrained_pos(point.x, point.y, point.z);
        deform_mesh.set_target_position(target(center, polyhedron), constrained_pos);
        deform_mesh.deform(10,0.0);
        //update the item
        surface->invalidateOpenGLBuffers();
        surface->itemChanged();
        break;
      }
      default:
        break;
      }
      return true;
    }
  }
  return false;
}
#include "Surface_from_picked_points_plugin.moc"


