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
#include <CGAL/intersections.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>

#include "Messages_interface.h"
#include "Scene_polylines_item.h"
#include "Scene_polyhedron_item.h"
#include "Scene_points_with_normal_item.h"
#include "Plugins/Mesh_3/Volume_plane_interface.h"
#include "ui_Create_surface.h"
typedef Scene_polylines_item::Point_3 Point_3;
typedef Polyhedron::Traits Kernel;
typedef CGAL::Surface_mesh_deformation<Polyhedron> Surface_mesh_deformation;
// A modifier creating a Polyhedron with the incremental builder.
template <class HDS>
class Build_polyhedron : public CGAL::Modifier_base<HDS> {
  std::vector<Point_3> points;
  int size_generator;
  int size_leader;
public:
  Build_polyhedron(const std::vector<Point_3> &p, int sizeI, int sizeJ)
    :points(p),size_generator(sizeI), size_leader(sizeJ){}
  int vertex(int i, int j)
  {
    return size_leader*i+j;
  }
  void operator()( HDS& hds)
  {
    CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
    B.begin_surface( size_generator*size_leader, (size_generator-1)*(size_leader-1), 0);
    for(int i = 0; i<size_generator*size_leader; ++i)
      B.add_vertex(points[i]);
    for(int i = 0; i<size_generator-1; ++i)
      for(int j = 0; j<size_leader-1; ++j)
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
typedef boost::unordered_set<Polyhedron::Vertex_handle, CGAL::Handle_hash_function>    Vertex_set;
struct Is_constrained_map
{
  Vertex_set *m_ptr;

  typedef Vertex_set::key_type               key_type;
  typedef bool                               value_type;
  typedef bool                               reference;
  typedef boost::read_write_property_map_tag category;

  Is_constrained_map()
    : m_ptr(NULL)
  {}
  Is_constrained_map(Vertex_set *vset)
    : m_ptr(vset)
  {}
  friend bool get(const Is_constrained_map& map, const key_type& k)
  {
    CGAL_assertion(map.m_ptr != NULL);
    return map.m_ptr->count(k);
  }
  friend void put(Is_constrained_map& map, const key_type& k, value_type b)
  {
    CGAL_assertion(map.m_ptr != NULL);
    if (b)  map.m_ptr->insert(k);
    else    map.m_ptr->erase(k);
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
    for(int i=0; i<scene->numberOfEntries(); ++i)
      if(strcmp(scene->item(i)->metaObject()->className(),"Volume_plane_interface") == 0)
        return true;
    return false;
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
    mw = mainWindow;
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
    connect(ui_widget.createSurfaceButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::add_surface);
    connect(ui_widget.newPolylineButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::add_polyline);
    connect(ui_widget.finishButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::finish);
    connect(ui_widget.cancelButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::cancel);
    connect(ui_widget.edgeSpinBox, SIGNAL(valueChanged(double)),
            this, SLOT(setEdgeSize(double)));
    mode = IDLE;
    ui_widget.cancelButton->setEnabled(false);
    QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
    viewer->installEventFilter(this);
    mw->installEventFilter(this);
    addDockWidget(dock_widget);
    generator_is_created = false;
    mode = IDLE;
    ui_widget.cancelButton->setEnabled(false);
    generator_poly = NULL;
    control_points_item = NULL;
    g_plane = NULL;
    leader_poly = NULL;
    l_plane = NULL;
    surface = NULL;
  }
  bool eventFilter(QObject *, QEvent *event);
private Q_SLOTS:
  void pick()
  {
    // show/hide the dock widget
    if(dock_widget->isVisible()) { dock_widget->hide(); }
    else                         { dock_widget->show(); }

  }
  void reset_surface() { surface = NULL; control_points.clear();}
  void reset_control_points() { control_points_item = NULL; control_points.clear();}
  void reset_generator() { generator_poly = NULL; generator_is_created = false; g_plane = NULL;}
  void reset_leader() { leader_poly = NULL; l_plane = NULL;}
  void closure()
  {
    dock_widget->hide();
  }
  void add_polyline()
  {
    mode = ADD_POLYLINE;
    Q_FOREACH(int id, hidden_planes)
    {
      if(scene->item(id))
        scene->item(id)->setVisible(true);
    }
    hidden_planes.clear();
    ui_widget.cancelButton->setEnabled(true);
    if(!generator_is_created)
    {
      if(leader_poly)
      {
        disconnect(leader_poly, &Scene_polylines_item::aboutToBeDestroyed,
                this, &SurfaceFromPickedPointsPlugin::reset_leader);
        leader_poly = NULL;
      }
      generator_poly = new Scene_polylines_item();
      connect(generator_poly, &Scene_polylines_item::aboutToBeDestroyed,
              this, &SurfaceFromPickedPointsPlugin::reset_generator);
      generator_poly->polylines.push_back( Scene_polylines_item::Polyline() );
      generator_poly->setName("Generator Polyline");
      generator_poly->setColor(QColor(Qt::green));
      scene->addItem(generator_poly);
      ui_widget.newPolylineButton->setText("New Leader");
      ui_widget.newPolylineButton->setEnabled(false);
    }
    else
    {
      leader_poly = new Scene_polylines_item();
      connect(leader_poly, &Scene_polylines_item::aboutToBeDestroyed,
              this, &SurfaceFromPickedPointsPlugin::reset_leader);
      leader_poly->polylines.push_back( Scene_polylines_item::Polyline() );
      leader_poly->setName("Leader Polyline");
      scene->addItem(leader_poly);
      ui_widget.newPolylineButton->setEnabled(false);
    }
    generator_is_created = !generator_is_created;
    min_dist = -1;
  }
  void add_surface()
  {
    if(!(generator_poly && leader_poly))
      return;
    Q_FOREACH(int id, hidden_planes)
    {
      if(scene->item(id))
        scene->item(id)->setVisible(true);
    }
    hidden_planes.clear();
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
    std::vector<Point_3>& g_polyline = generator_poly->polylines.back();
    std::vector<Point_3>& l_polyline = leader_poly->polylines.back();
    std::vector<Point_3> control_pos;

    //Only work if the polylines intersect the planes at most once.
     boost::optional<boost::variant<Point_3, Kernel::Segment_3, Kernel::Line_3> > o;
    int g_id(-1), l_id(-1);
    for(std::size_t i=0; i< l_polyline.size()-1; ++i)
    {
      if((g_plane->has_on_negative_side(l_polyline[i]) &&
          g_plane->has_on_positive_side(l_polyline[i+1])) ||
         (g_plane->has_on_negative_side(l_polyline[i+1]) &&
          g_plane->has_on_positive_side(l_polyline[i])))
      {
        l_id = i+1;
        break;
      }
    }
    if(l_id == -1)
    {
      Kernel::Vector_3 f_diff = l_polyline.front()-g_polyline.front();
      Kernel::Vector_3 b_diff = l_polyline.back()-g_polyline.back();
      if(f_diff.squared_length()< b_diff.squared_length())
      {
        l_id =0;
      }
      else
      {
        l_id = l_polyline.size()-1;
      }
    }

    for(std::size_t i=0; i< g_polyline.size()-1; ++i)
    {
      if((l_plane->has_on_negative_side(g_polyline[i]) &&
          l_plane->has_on_positive_side(g_polyline[i+1])) ||
         (l_plane->has_on_negative_side(g_polyline[i+1]) &&
          l_plane->has_on_positive_side(g_polyline[i])))
      {
        g_id = i+1;
        o = *intersection(
              Kernel::Segment_3(g_polyline[i], g_polyline[i+1]),*l_plane);
        break;
      }
    }
    if(g_id == -1)
    {
      o =*intersection(
            Kernel::Line_3(g_polyline[0], g_polyline[1])
          ,*l_plane);
      Kernel::Vector_3 diff = boost::get<Point_3>(*o)-g_polyline[0];
      double sq_dist = diff.squared_length();
      o =*intersection(
            Kernel::Line_3(g_polyline[g_polyline.size()-1], g_polyline[g_polyline.size()-2])
          ,*l_plane);
      diff = boost::get<Point_3>(*o)-g_polyline[g_polyline.size()-1];

      if(sq_dist < diff.squared_length())
      {
        g_id =0;
        o =*intersection(
              Kernel::Line_3(g_polyline[0], g_polyline[1])
            ,*l_plane);

      }
      else
      {
        g_id = g_polyline.size();

        o = *intersection(
               Kernel::Line_3(g_polyline[g_id-1], g_polyline[g_id-2])
             ,*l_plane);

      }
    }
    //get the intersection point between the generator and the leader's plane

    if(o==boost::none)
    {
      this->messageInterface->warning("intersection point between generator and leader's plane cannot be found");
      return;
    }
    Point_3 p = boost::get<Point_3>(*o);
    g_polyline.insert(g_polyline.begin()+g_id, p);
    generator_poly->invalidateOpenGLBuffers();
    generator_poly->itemChanged();
    //compute the points
    for(std::size_t i=0; i< g_polyline.size(); ++i)
    {
      Kernel::Vector_3 offset = g_polyline[i]-p;
      for(std::size_t j=0; j<l_polyline.size(); ++j)
      {
        if(j==static_cast<std::size_t>(l_id))
        {
          points.push_back(g_polyline[i]);
          control_pos.push_back(points.back());
        }

        points.push_back(l_polyline[j]+offset);
        if(i == static_cast<std::size_t>(g_id))
          control_pos.push_back(points.back());
      }
    }
    initial_mesh_item = new Scene_points_with_normal_item();
    BOOST_FOREACH(Point_3 p, points)
        initial_mesh_item->point_set()->insert(p);
    initial_mesh_item->setName("Initial mesh");
    initial_mesh_item->setVisible(false);
    scene->addItem(initial_mesh_item);
    //create Polyhedron item
    Polyhedron *polyhedron = new Polyhedron();
    Build_polyhedron<Polyhedron::HalfedgeDS> builder(points, (int)g_polyline.size(), (int)l_polyline.size()+1);
    polyhedron->delegate(builder);
    // Init the indices of the halfedges and the vertices.
    set_halfedgeds_items_id(*polyhedron);


    //save the points of the initial polylines as control points
    for(
    Polyhedron::Vertex_iterator vit = polyhedron->vertices_begin();
        vit != polyhedron->vertices_end();
        ++vit)
    {
      int i=0;
      BOOST_FOREACH(Point_3 p, control_pos)
      {
        if(vit->point() == p)
        {
          control_points.push_back(vit);
          control_pos.erase(control_pos.begin()+i);
          break;
        }
        ++i;
      }
      if(control_pos.empty())
        break;
    }
    surface = new Scene_polyhedron_item(polyhedron);
    surface->setName("Surface");
    scene->addItem(surface);
    connect(surface, &Scene_polyhedron_item::aboutToBeDestroyed,
            this, &SurfaceFromPickedPointsPlugin::reset_surface);
    control_points_item = new Scene_points_with_normal_item();
    Q_FOREACH(Polyhedron::Vertex_iterator vit, control_points)
    {
      control_points_item->point_set()->insert(vit->point());
    }
    control_points_item->setName("Fixed Points");
    control_points_item->setColor(QColor(Qt::red));
    scene->addItem(control_points_item);
    connect(control_points_item, &Scene_polyhedron_item::aboutToBeDestroyed,
            this, &SurfaceFromPickedPointsPlugin::reset_control_points);
    ui_widget.createSurfaceButton->setText("Remesh");
    disconnect(ui_widget.createSurfaceButton, &QPushButton::clicked,
               this, &SurfaceFromPickedPointsPlugin::add_surface);
    connect(ui_widget.createSurfaceButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::remesh);
  }

  void finish()
  {
    mode = IDLE;
    ui_widget.cancelButton->setEnabled(false);
    if(surface)
      disconnect(surface, &Scene_polyhedron_item::aboutToBeDestroyed,
                 this, &SurfaceFromPickedPointsPlugin::reset_surface);
    if(control_points_item)
      disconnect(control_points_item, &Scene_polyhedron_item::aboutToBeDestroyed,
                 this, &SurfaceFromPickedPointsPlugin::reset_control_points);
    if(generator_poly)
      disconnect(generator_poly, &Scene_polyhedron_item::aboutToBeDestroyed,
                 this, &SurfaceFromPickedPointsPlugin::reset_generator);
    if(leader_poly)
      disconnect(leader_poly, &Scene_polyhedron_item::aboutToBeDestroyed,
                 this, &SurfaceFromPickedPointsPlugin::reset_leader);
    reset_leader();
    reset_generator();
    reset_surface();
    reset_control_points();
    ui_widget.createSurfaceButton->setEnabled(false);
    ui_widget.createSurfaceButton->setText("Create Surface");
    disconnect(ui_widget.createSurfaceButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::remesh);
    connect(ui_widget.createSurfaceButton, &QPushButton::clicked,
               this, &SurfaceFromPickedPointsPlugin::add_surface);
    ui_widget.newPolylineButton->setText("New Generator");
    ui_widget.newPolylineButton->setEnabled(true);
    Q_FOREACH(int id, hidden_planes)
    {
      if(scene->item(id))
        scene->item(id)->setVisible(true);
    }
    hidden_planes.clear();
  }
  void cancel()
  {
    if(mode != ADD_POLYLINE)
      return;
    Scene_polylines_item* current_polyline = NULL;
    QDoubleSpinBox* current_spin = ui_widget.edgeSpinBox;
    if(generator_is_created)
    {
      current_polyline = generator_poly;
    }
    else
    {
      current_polyline = leader_poly;
    }

    std::vector<Point_3>& polyline = current_polyline->polylines.back();
    if(polyline.size() >= 1)
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
          current_spin->setValue(min_dist);
        }
      }
    }
  }
  void setEdgeSize(double d) { edgeSize = d; }
  //remesh the polyhedron to optimize the surface
  void remesh()
  {
    if(!surface)
      return;
    Polyhedron* poly = surface->polyhedron();
    Vertex_set is_constrained_set;
    Q_FOREACH(Polyhedron::Vertex_handle vh, control_points)
      is_constrained_set.insert(vh);

    Is_constrained_map vcm(&is_constrained_set);
    CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(*poly),
                                                       edgeSize,
                                                       *poly,
                                                       CGAL::Polygon_mesh_processing::parameters::vertex_is_constrained_map(vcm));
    control_points.clear();
    Q_FOREACH(Polyhedron::Vertex_handle vh, is_constrained_set)
      control_points.push_back(vh);

    surface->invalidateOpenGLBuffers();
    surface->itemChanged();
  }
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
  QDockWidget* dock_widget;
  Ui::Create_surface ui_widget;
  Select_mode mode;
  Scene_polylines_item* generator_poly;
  Scene_polylines_item* leader_poly;
  Scene_polyhedron_item* surface;
  Scene_points_with_normal_item* initial_mesh_item;
  Scene_points_with_normal_item* control_points_item;
  bool generator_is_created;
  double edgeSize;
  double min_dist;
  std::vector<Polyhedron::Vertex_handle > control_points;
  Kernel::Plane_3* l_plane;
  Kernel::Plane_3* g_plane;
  bool find_plane(QMouseEvent* e, Kernel::Plane_3& plane);
  std::vector<int> hidden_planes;
};

bool SurfaceFromPickedPointsPlugin::find_plane(QMouseEvent* e, Kernel::Plane_3& plane)
{
  QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
  Volume_plane_interface* plane_interface = NULL;
  bool found = false;
  viewer->camera()->pointUnderPixel(e->pos(), found);
  if(!found)
  {
    return false;
  }
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

  plane = Kernel::Plane_3(n[0], n[1],  n[2], - n * pos);
  return true;
}

bool SurfaceFromPickedPointsPlugin::eventFilter(QObject *object, QEvent *event)
{
  if (mode == IDLE)
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
        QDoubleSpinBox* current_spin = ui_widget.edgeSpinBox;
        if(generator_is_created)
        {
          current_polyline = generator_poly;
          if(!g_plane)
          {
            g_plane = new Kernel::Plane_3();
            if(!find_plane(e, *g_plane))
            {
              delete g_plane;
              g_plane = NULL;
              return false;
            }
          }
          else
          {
            //project point on plane
            if ( !g_plane->has_on(Point_3(point.x, point.y, point.z)))
            {
              qglviewer::Vec pos = viewer->camera()->position();
              Kernel::Line_3 ray(Point_3(pos.x, pos.y, pos.z), Point_3(point.x, point.y, point.z));
              Point_3 res = boost::get<Point_3>(*intersection(*g_plane, ray));
              point = qglviewer::Vec(res.x(), res.y(), res.z());
            }
          }
          if(generator_poly->polylines.back().size() >=1)
            ui_widget.newPolylineButton->setEnabled(true);
        }
        else if(leader_poly)
        {
          current_polyline = leader_poly;
          if(!l_plane)
          {
            l_plane = new Kernel::Plane_3();
            if(!find_plane(e, *l_plane))
            {
              delete l_plane;
              l_plane = NULL;
              return false;
            }
          }
          else
          {
           //project point on plane
            if ( !l_plane->has_on(Point_3(point.x, point.y, point.z)))
            {
              qglviewer::Vec pos = viewer->camera()->position();
              Kernel::Line_3 ray(Point_3(pos.x, pos.y, pos.z), Point_3(point.x, point.y, point.z));
              Point_3 res = boost::get<Point_3>(*intersection(*l_plane, ray));
              point = qglviewer::Vec(res.x(), res.y(), res.z());
            }
          }
          if(leader_poly->polylines.back().size() >=1)
            ui_widget.createSurfaceButton->setEnabled(true);
        }
        else
          return false;
        for(int i=0; i<scene->numberOfEntries(); ++i)
        {
          if(strcmp(scene->item(i)->metaObject()->className(),"Volume_plane_interface") == 0 &&
             i != scene->mainSelectionIndex())
          {
            scene->item(i)->setVisible(false);
            hidden_planes.push_back(i);
          }

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
              current_spin->setValue(min_dist/2.0);
            }
          }
          current_polyline->invalidateOpenGLBuffers();
          current_polyline->itemChanged();
        }
        break;
      }
      case ADD_POINT_AND_DEFORM:
      {
        typedef CGAL::AABB_halfedge_graph_segment_primitive<Polyhedron> HGSP;
        typedef CGAL::AABB_traits<Kernel, HGSP>                         AABB_traits;
        typedef CGAL::AABB_tree<AABB_traits>                            AABB_tree;
        if(!surface)
          return false;
        Kernel::Plane_3 plane;
        if(!find_plane(e,plane))
          return false;
        Polyhedron &polyhedron = *surface->polyhedron();
        // Init the indices of the halfedges and the vertices.
        set_halfedgeds_items_id(polyhedron);
        //Slice the Polyhedron along the picked plane
        std::vector<std::vector<Point_3> > slices;
        AABB_tree tree(edges(polyhedron).first, edges(polyhedron).second, polyhedron);
        CGAL::Polygon_mesh_slicer<Polyhedron, Kernel> slicer_aabb(polyhedron, tree);
        slicer_aabb(plane, std::back_inserter(slices));
        if(slices.empty())
        {
          messageInterface->warning("The picked plane must intersect the surface.");
          return false;
        }
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
        control_points.push_back(center->vertex());
        //add the control points
        deform_mesh.insert_control_vertices(control_points.begin(), control_points.end());
        //deform
        bool is_matrix_factorization_OK = deform_mesh.preprocess();
        if(!is_matrix_factorization_OK){
          std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
          return false;
        }
        Surface_mesh_deformation::Point constrained_pos(point.x, point.y, point.z);
        deform_mesh.set_target_position(target(center, polyhedron), constrained_pos);
        deform_mesh.deform(3, 1e-4);
        //update the item
        if(control_points_item)
        {
          control_points_item->point_set()->insert(constrained_pos);
          control_points_item->invalidateOpenGLBuffers();
          control_points_item->itemChanged();
        }
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


