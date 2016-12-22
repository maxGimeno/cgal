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
#include "Scene.h"

#include <CGAL/jet_smooth_point_set.h>

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif
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

class SurfaceGroup :
    public Scene_group_item
{
  Q_OBJECT
public:
  SurfaceGroup(std::vector<SurfaceGroup*>* list)
    :Scene_group_item()
  {
    generator_poly = NULL;
    control_points_item = NULL;
    g_plane = NULL;
    leader_poly = NULL;
    l_plane = NULL;
    surface = NULL;
    last_remesh = -1;
    generator_is_created = false;
    surface_groups = list;
    surface_groups->push_back(this);
  }
  Scene_polylines_item* generator_poly;
  Scene_polylines_item* leader_poly;
  Scene_points_with_normal_item* initial_mesh_item;
  Scene_points_with_normal_item* control_points_item;
  Scene_polyhedron_item* surface;
  std::vector<SurfaceGroup*>* surface_groups;
  std::vector<Point_3> points_stack;
  std::vector<Point_3> repoints_stack;
  std::vector<Point_3> edit_points_stack;
  std::vector<Point_3> reedit_points_stack;
  std::vector<Point_3> extend_points_stack;
  std::vector<Kernel::Plane_3> planes_stack;
  std::vector<Kernel::Plane_3> replanes_stack;
  std::vector<Kernel::Plane_3> edit_planes_stack;
  std::vector<Kernel::Plane_3> reedit_planes_stack;
  std::vector<char> operations_done; // 0 =  insertion, 1 = removal, 2 = an edition , 3 = extension
  std::vector<char> operations_redone;// 0 =  insertion, 1 = removal, 2 = an edition, 3 = extension
  bool generator_is_created;
  double edgeSize;
  double min_dist;
  double last_remesh;
  std::vector<Polyhedron::Vertex_handle > control_points;
  Kernel::Plane_3* l_plane;
  Kernel::Plane_3* g_plane;
  int control_limit, g_id, l_id;
  std::vector<Kernel::Plane_3> control_points_planes; //control_points_planes[i] is the plane associated to control_points[i+control_limit]
  std::vector<Point_3> ordered_control_points;
public Q_SLOTS:
  void quitList()
  {
    int i=0;
    Q_FOREACH(SurfaceGroup* suspect, *surface_groups)
    {
      if(suspect == this)
        break;
      ++i;
    }
    surface_groups->erase(surface_groups->begin()+i);
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
    connect(ui_widget.UndoButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::cancel);
    connect(ui_widget.redoButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::redo);
    connect(ui_widget.editButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::edit);
    connect(ui_widget.edgeSpinBox, SIGNAL(valueChanged(double)),
            this, SLOT(setEdgeSize(double)));
    connect(ui_widget.regenButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::minEnergy);
    connect(ui_widget.smoothButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::smooth);
    connect(static_cast<Scene*>(scene), SIGNAL(itemIndexSelected(int)),
            this, SLOT(checkEdit(int)));
    QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
    viewer->installEventFilter(this);
    mw->installEventFilter(this);
    addDockWidget(dock_widget);
    mode = IDLE;
    ui_widget.UndoButton->setEnabled(false);
    ui_widget.redoButton->setEnabled(false);
    ui_widget.edgeSpinBox->setEnabled(false);
    ui_widget.regenButton->setEnabled(false);
    ui_widget.smoothButton->setEnabled(false);
    current_group = NULL;
    is_editing = false;
    is_selecting = false;
  }
  bool eventFilter(QObject *, QEvent *event);
private Q_SLOTS:
  void pick()
  {
    // show/hide the dock widget
    if(dock_widget->isVisible()) { dock_widget->hide(); }
    else                         { dock_widget->show(); }

  }

  void smooth()
  {
    Polyhedron* polyhedron = current_group->surface->polyhedron();
    Vertex_set is_constrained_set;
    Q_FOREACH(Polyhedron::Vertex_handle vh, current_group->control_points)
      is_constrained_set.insert(vh);
    std::vector<Point_3> to_save;
    to_save.reserve(current_group->control_points.size());
    BOOST_FOREACH(Polyhedron::Vertex_handle vh, vertices(*polyhedron))
    {
      if(is_constrained_set.count(vh) == 1)
        to_save.push_back(vh->point());
    }

    CGAL::jet_smooth_point_set<Concurrency_tag>(vertices(*polyhedron).first, vertices(*polyhedron).second,
                                                get(CGAL::vertex_point, *polyhedron),
                                                24, Kernel());
    int id =-1;
    BOOST_FOREACH(Polyhedron::Vertex_handle vh, vertices(*polyhedron))
    {
      if(is_constrained_set.count(vh) == 1)
        vh->point() = to_save[++id];
    }

    current_group->surface->invalidateOpenGLBuffers();
    current_group->surface->itemChanged();
  }

  void closure()
  {
    dock_widget->hide();
  }
  void add_polyline()
  {
    ui_widget.UndoButton->setEnabled(false);
    mode = ADD_POLYLINE;
    Q_FOREACH(int id, hidden_planes)
    {
      if(scene->item(id))
        scene->item(id)->setVisible(true);
    }
    hidden_planes.clear();
    if(!current_group)
    {
      current_group = new SurfaceGroup(&surface_groups);
      connect(current_group, &SurfaceGroup::aboutToBeDestroyed,
              this, &SurfaceFromPickedPointsPlugin::finish);
      connect(current_group, &SurfaceGroup::aboutToBeDestroyed,
              current_group, &SurfaceGroup::quitList);
      static int rift_nb = 0;
      scene->addItem(current_group);
      current_group->setName(QString("Rift #%1").arg(rift_nb++));
    }
    if(!current_group->generator_is_created)
    {
      if(current_group->leader_poly)
      {
        current_group->leader_poly = NULL;
      }
      current_group->generator_poly = new Scene_polylines_item();
      current_group->generator_poly->polylines.push_back( Scene_polylines_item::Polyline() );
      current_group->generator_poly->setName("Generator Polyline");
      current_group->generator_poly->setColor(QColor(Qt::green));
      scene->addItem(current_group->generator_poly);
      scene->changeGroup(current_group->generator_poly, current_group);
      current_group->lockChild(current_group->generator_poly);
      ui_widget.newPolylineButton->setText("New Leader");
      ui_widget.newPolylineButton->setEnabled(false);
    }
    else
    {
      current_group->leader_poly = new Scene_polylines_item();
      current_group->leader_poly->polylines.push_back( Scene_polylines_item::Polyline() );
      current_group->leader_poly->setName("Leader Polyline");
      current_group->leader_poly->setColor(QColor("#AAFFFF"));
      scene->addItem(current_group->leader_poly);
      scene->changeGroup(current_group->leader_poly, current_group);
      current_group->lockChild(current_group->leader_poly);
      ui_widget.newPolylineButton->setEnabled(false);
    }
    current_group->generator_is_created = !current_group->generator_is_created;
    current_group->min_dist = -1;
  }

  void add_surface()
  {
    if(!(current_group->generator_poly && current_group->leader_poly))
      return;
    Q_FOREACH(int id, hidden_planes)
    {
      if(scene->item(id))
        scene->item(id)->setVisible(true);
    }
    hidden_planes.clear();
    mode = ADD_POINT_AND_DEFORM;
    ui_widget.UndoButton->setEnabled(false);
    clear_redo();
    current_group->operations_done.clear();
    //create points
    std::vector<Point_3> points;
    std::vector<Point_3>& g_polyline = current_group->generator_poly->polylines.back();
    std::vector<Point_3>& l_polyline = current_group->leader_poly->polylines.back();
    std::vector<Point_3> control_pos;

    //Only work if the polylines intersect the planes at most once.
    boost::optional<boost::variant<Point_3, Kernel::Segment_3, Kernel::Line_3> > o;
    current_group->g_id = -1; current_group->l_id = -1;
    CGAL::Oriented_side first_side= current_group->g_plane->oriented_side(l_polyline[0]);
    bool need_split_l = true;
    //find the closest generator's end point to the l_plane
    Point_3 g_closest_to_plane;
    double sq_dist = CGAL::squared_distance(g_polyline.front(), *current_group->l_plane);
    if(CGAL::squared_distance(g_polyline.back(), *current_group->l_plane) <sq_dist)
      g_closest_to_plane = g_polyline.back();
    else
      g_closest_to_plane = g_polyline.front();

    for(std::size_t i=1; i< l_polyline.size(); ++i)
    {
      if(current_group->g_plane->oriented_side(l_polyline[i]) !=
         first_side)
      {
        if(current_group->g_plane->oriented_side(l_polyline[i]) == CGAL::ON_ORIENTED_BOUNDARY)
        {
          need_split_l = false;
        }
        current_group->l_id = i;
        break;
      }
    }
    if(current_group->l_id == -1)
    {
      Kernel::Vector_3 f_diff = l_polyline.front()-g_closest_to_plane;
      Kernel::Vector_3 b_diff = l_polyline.back()-g_closest_to_plane;
      if(f_diff.squared_length()< b_diff.squared_length())
      {
        current_group->l_id =0;
      }
      else
      {
        current_group->l_id = (int)l_polyline.size();
      }
    }
    first_side= current_group->l_plane->oriented_side(g_polyline[0]);
    for(std::size_t i=1; i< g_polyline.size(); ++i)
    {
      if(current_group->l_plane->oriented_side(g_polyline[i]) !=
         first_side)
      {
        current_group->g_id = i;
        if(current_group->l_plane->oriented_side(g_polyline[i]) != CGAL::ON_ORIENTED_BOUNDARY)
          o = *intersection(
                Kernel::Segment_3(g_polyline[i-1], g_polyline[i]),*current_group->l_plane);
        break;
      }
    }
    if(current_group->g_id == -1)
    {
      o =*intersection(
            Kernel::Line_3(g_polyline[0], g_polyline[1])
          ,*current_group->l_plane);
      Kernel::Vector_3 diff = boost::get<Point_3>(*o)-g_polyline[0];
      double sq_dist = diff.squared_length();
      o =*intersection(
            Kernel::Line_3(g_polyline[g_polyline.size()-1], g_polyline[g_polyline.size()-2])
          ,*current_group->l_plane);
      diff = boost::get<Point_3>(*o)-g_polyline[g_polyline.size()-1];

      if(sq_dist < diff.squared_length())
      {
        current_group->g_id =0;
        o =*intersection(
              Kernel::Line_3(g_polyline[0], g_polyline[1])
            ,*current_group->l_plane);

      }
      else
      {
        current_group->g_id = (int)g_polyline.size();

        o = *intersection(
              Kernel::Line_3(g_polyline[current_group->g_id-1], g_polyline[current_group->g_id-2])
            ,*current_group->l_plane);

      }
    }
    //get the intersection point between the generator and the leader's plane
    Point_3 p;
    if(o!=boost::none)
      p = boost::get<Point_3>(*o);

    g_polyline.insert(g_polyline.begin()+current_group->g_id, p);
    current_group->generator_poly->invalidateOpenGLBuffers();
    current_group->generator_poly->itemChanged();
    //compute the points
    for(std::size_t i=0; i< g_polyline.size(); ++i)
    {
      Kernel::Vector_3 offset = g_polyline[i]-p;
      for(std::size_t j=0; j<l_polyline.size(); ++j)
      {
        if(j==static_cast<std::size_t>(current_group->l_id))
        {
          if(need_split_l)
            points.push_back(g_polyline[i]);
          control_pos.push_back(points.back());
        }

        points.push_back(l_polyline[j]+offset);
        if(i == static_cast<std::size_t>(current_group->g_id))
          control_pos.push_back(points.back());
      }
      if(static_cast<std::size_t>(current_group->l_id) == l_polyline.size())
      {
        points.push_back(g_polyline[i]);
        control_pos.push_back(points.back());
      }
    }

    current_group->initial_mesh_item = new Scene_points_with_normal_item();
    BOOST_FOREACH(Point_3 p, points)
        current_group->initial_mesh_item->point_set()->insert(p);
    current_group->initial_mesh_item->setName("Initial mesh");
    current_group->initial_mesh_item->setVisible(false);
    scene->addItem(current_group->initial_mesh_item);
    scene->changeGroup(current_group->initial_mesh_item, current_group);
    current_group->lockChild(current_group->initial_mesh_item);
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
          current_group->control_points.push_back(vit);
          control_pos.erase(control_pos.begin()+i);
          break;
        }
        ++i;
      }
      if(control_pos.empty())
        break;
    }
    current_group->control_limit = (int)current_group->control_points.size();
    current_group->surface = new Scene_polyhedron_item(polyhedron);
    current_group->surface->setName("Surface");
    scene->addItem(current_group->surface);
    scene->changeGroup(current_group->surface, current_group);
    current_group->lockChild(current_group->surface);
    current_group->control_points_item = new Scene_points_with_normal_item();
    Q_FOREACH(Polyhedron::Vertex_iterator vit, current_group->control_points)
    {
      current_group->control_points_item->point_set()->insert(vit->point());
    }
    current_group->control_points_item->setName("Fixed Points");
    current_group->control_points_item->setColor(QColor(Qt::red));
    scene->addItem(current_group->control_points_item);
    scene->changeGroup(current_group->control_points_item, current_group);
    current_group->lockChild(current_group->control_points_item);
    ui_widget.createSurfaceButton->setText("Remesh");
    disconnect(ui_widget.createSurfaceButton, &QPushButton::clicked,
               this, &SurfaceFromPickedPointsPlugin::add_surface);
    connect(ui_widget.createSurfaceButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::remesh);
    ui_widget.edgeSpinBox->setEnabled(true);
    ui_widget.regenButton->setEnabled(true);
    ui_widget.smoothButton->setEnabled(true);
  }

  void finish()
  {
    mode = IDLE;
    ui_widget.UndoButton->setEnabled(false);
    ui_widget.createSurfaceButton->setEnabled(false);
    ui_widget.regenButton->setEnabled(false);
    ui_widget.smoothButton->setEnabled(false);
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
    current_group = NULL;
    if(!current_group)
      return;
    current_group->operations_done.clear();
    clear_redo();
    disconnect(current_group, &SurfaceGroup::aboutToBeDestroyed,
               this, &SurfaceFromPickedPointsPlugin::finish);
  }

  void cancel()
  {
    is_editing = false;
    if(mode != ADD_POLYLINE && mode != ADD_POINT_AND_DEFORM)
      return;
    ui_widget.redoButton->setEnabled(true);
    if(mode == ADD_POLYLINE)
    {
      Scene_polylines_item* current_polyline = NULL;
      bool plane_is_generator = false;
      if(current_group->generator_is_created)
      {
        current_polyline = current_group->generator_poly;
        plane_is_generator = true;
      }
      else
      {
        current_polyline = current_group->leader_poly;
      }

      std::vector<Point_3>& polyline = current_polyline->polylines.back();
      if(polyline.size() > 0)
      {
        current_group->points_stack.push_back(polyline.back());
        polyline.pop_back();
        current_polyline->invalidateOpenGLBuffers();
        current_polyline->itemChanged();
      }
      if(polyline.empty())
      {
        Q_FOREACH(int id, hidden_planes)
        {
          if(scene->item(id))
            scene->item(id)->setVisible(true);
        }
        hidden_planes.clear();
        if(plane_is_generator)
        {
          delete current_group->g_plane;
          current_group->g_plane = NULL;
          current_group->generator_is_created = true;
        }
        else
        {
          delete current_group->l_plane;
          current_group->l_plane = NULL;
        }
        ui_widget.redoButton->setEnabled(false);
      }
    }
    else if(mode == ADD_POINT_AND_DEFORM)
    {
      if(current_group->operations_done.back() == 0)
      {
        if(current_group->ordered_control_points.size()==0)
          return;
        int i=0;
        Q_FOREACH(Polyhedron::Vertex_handle vh, current_group->control_points)
        {
          if(vh->point() == current_group->ordered_control_points.back())
          {
            current_group->control_points.erase(current_group->control_points.begin()+i);
            break;
          }
          ++i;
        }
        current_group->points_stack.push_back(current_group->ordered_control_points.back());
        current_group->ordered_control_points.pop_back();
        current_group->planes_stack.push_back(current_group->control_points_planes.back());
        current_group->control_points_planes.pop_back();
        current_group->control_points_item->point_set()->clear();
        Q_FOREACH(Polyhedron::Vertex_iterator vit, current_group->control_points)
        {
          current_group->control_points_item->point_set()->insert(vit->point());
        }
        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
      }
      else if(current_group->operations_done.back() == 1)
      {
        if(current_group->repoints_stack.size()==0)
          return;
        Point_3 point = current_group->repoints_stack.back();
        for(
            Polyhedron::Vertex_iterator vit = current_group->surface->polyhedron()->vertices_begin();
            vit != current_group->surface->polyhedron()->vertices_end();
            ++vit)
        {
          if(vit->point() == point)
          {
            current_group->control_points.push_back(vit);
            break;
          }
        }
        current_group->ordered_control_points.push_back(point);
        current_group->control_points_planes.push_back(current_group->replanes_stack.back());
        current_group->replanes_stack.pop_back();
        current_group->repoints_stack.pop_back();

        current_group->control_points_item->point_set()->insert(point);
        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
      }
      else if(current_group->operations_done.back() == 2)
      {
        //case point is on polyline
        if(current_group->edit_planes_stack.back() == Kernel::Plane_3(1,1,1,0) &&
           current_group->edit_planes_stack[current_group->edit_planes_stack.size()-2] == Kernel::Plane_3(1,1,1,0))
        {
          Point_3 first = current_group->edit_points_stack.back();
          current_group->reedit_points_stack.push_back(first);
          current_group->edit_points_stack.pop_back();
          Point_3 second = current_group->edit_points_stack.back();
          current_group->reedit_points_stack.push_back(second);
          current_group->edit_points_stack.pop_back();
          current_group->reedit_planes_stack.push_back(Kernel::Plane_3(1,1,1,0));
          current_group->reedit_planes_stack.push_back(Kernel::Plane_3(1,1,1,0));
          current_group->edit_planes_stack.pop_back();
          current_group->edit_planes_stack.pop_back();
         ////////////////////////////////////
          int id = -1, i = 0;
          std::vector<Point_3> &l_poly = current_group->leader_poly->polylines.back();
          std::vector<Point_3> &g_poly = current_group->generator_poly->polylines.back();
          Q_FOREACH(Point_3 cp, l_poly)
          {
            if(cp == first)
            {
              id = i;
              break;
            }
            ++i;
          }
          if(id>-1)
          {
            *(l_poly.begin()+id) = second;
            current_group->leader_poly->invalidateOpenGLBuffers();
            current_group->leader_poly->itemChanged();
          }
          else
          {
            i = 0;
            Q_FOREACH(Point_3 cp, g_poly)
            {
              if(cp == first)
              {
                id = i;
                break;
              }
              ++i;
            }
            if(id >-1)
            {
              *(g_poly.begin()+id) = second;
              current_group->generator_poly->invalidateOpenGLBuffers();
              current_group->generator_poly->itemChanged();
            }
            else
            {
              std::cerr<<"cannot find point to edit back."<<std::endl;
              return;
            }
          }
          Point_set_3<Kernel>::iterator pit;
          for (pit = current_group->control_points_item->point_set()->begin();
               pit != current_group->control_points_item->point_set()->end();
               ++pit)
          {
            if(current_group->control_points_item->point_set()->point(*pit) == first)
            {
              current_group->control_points_item->point_set()->select(pit);
              current_group->control_points_item->point_set()->delete_selection();
              current_group->control_points_item->point_set()->insert(second);
              break;
            }
          }
          minEnergy();
          //////////////////////////////////
        }
        //case point is simple control_point
        else
        {
          Point_3 first = current_group->edit_points_stack.back();
          current_group->reedit_points_stack.push_back(first);
          current_group->edit_points_stack.pop_back();
          Point_3 second = current_group->edit_points_stack.back();
          current_group->reedit_points_stack.push_back(second);
          current_group->edit_points_stack.pop_back();

          Kernel::Plane_3 first_plane = current_group->edit_planes_stack.back();
          current_group->reedit_planes_stack.push_back(first_plane);
          current_group->edit_planes_stack.pop_back();
          Kernel::Plane_3 second_plane = current_group->edit_planes_stack.back();
          current_group->reedit_planes_stack.push_back(second_plane);
          current_group->edit_planes_stack.pop_back();

          int id = -1;
          Q_FOREACH(Point_3 cp, current_group->ordered_control_points)
          {
            ++id;
            if(cp == first)
            {
              break;
            }
          }
          current_group->ordered_control_points[id] = second;
          current_group->control_points_planes[id] = second_plane;

          current_group->control_points_item->point_set()->clear();
          Point_set_3<Kernel>::iterator pit;
          for (pit = current_group->control_points_item->point_set()->begin();
               pit != current_group->control_points_item->point_set()->end();
               ++pit)
          {
            if(current_group->control_points_item->point_set()->point(*pit) == first)
            {
              current_group->control_points_item->point_set()->select(pit);
              current_group->control_points_item->point_set()->delete_selection();
              current_group->control_points_item->point_set()->insert(second);
              break;
            }
          }
          minEnergy();
        }

        current_group->control_points_item->point_set()->clear();
        Q_FOREACH(Polyhedron::Vertex_iterator vit, current_group->control_points)
        {
          current_group->control_points_item->point_set()->insert(vit->point());
        }
        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
      }
      else if(current_group->operations_done.back() == 3)
      {
        Point_3 point = current_group->extend_points_stack.back();
        current_group->extend_points_stack.pop_back();
        int id = -1, i = 0;
        std::vector<Point_3> &l_poly = current_group->leader_poly->polylines.back();
        std::vector<Point_3> &g_poly = current_group->generator_poly->polylines.back();
        Q_FOREACH(Point_3 cp, l_poly)
        {
          if(cp == point)
          {
            id = i;
            break;
          }
          ++i;
        }
        if(id>-1)
        {
          l_poly.erase(l_poly.begin()+id);
          double dist = Kernel::Vector_3(l_poly.front(), point).squared_length();
          if(Kernel::Vector_3(l_poly.back(), point).squared_length() > dist)
          {
            if(current_group->l_id > 0)
              --current_group->l_id;
          }
          current_group->leader_poly->invalidateOpenGLBuffers();
          current_group->leader_poly->itemChanged();
        }
        else
        {
          i = 0;
          Q_FOREACH(Point_3 cp, g_poly)
          {
            if(cp == point)
            {
              id = i;
              break;
            }
            ++i;
          }
          if(id >-1)
          {
            g_poly.erase(g_poly.begin()+id);
            double dist = Kernel::Vector_3(g_poly.front(), point).squared_length();
            if(Kernel::Vector_3(g_poly.back(), point).squared_length() > dist)
            {
              if(current_group->g_id>0)
                --current_group->g_id;
            }
            current_group->generator_poly->invalidateOpenGLBuffers();
            current_group->generator_poly->itemChanged();
          }
          else
          {
            std::cerr<<"cannot find point to erase."<<std::endl;
            return;
          }
        }

        Point_set_3<Kernel>::iterator pit;
        for (pit = current_group->control_points_item->point_set()->begin();
             pit != current_group->control_points_item->point_set()->end();
             ++pit)
        {
          if(current_group->control_points_item->point_set()->point(*pit) == point)
          {
            current_group->control_points_item->point_set()->select(pit);
            current_group->control_points_item->point_set()->delete_selection();
            break;
          }
        }
        current_group->control_limit--;
        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
        minEnergy();
      }
      //No redo for an extension, so don't push it into the redo-list
      if(current_group->operations_done.back() != 3)
         current_group->operations_redone.push_back(current_group->operations_done.back());
      current_group->operations_done.pop_back();
      if(current_group->operations_done.size() == 0)
        ui_widget.UndoButton->setEnabled(false);
    }
  }

  void redo()
  {
    ui_widget.UndoButton->setEnabled(true);
    if(mode == ADD_POLYLINE)
    {
      if(current_group->points_stack.size() == 0)
        return;
      Point_3 point = current_group->points_stack.back();
      Scene_polylines_item* current_polyline = NULL;
      QDoubleSpinBox* current_spin = ui_widget.edgeSpinBox;
      if(current_group->generator_is_created)
      {
        current_polyline = current_group->generator_poly;
      }
      else
      {
        current_polyline = current_group->leader_poly;
      }

      std::vector<Point_3>& polyline = current_polyline->polylines.back();
      polyline.push_back(point);
      if(polyline.size() >= 2)
      {
        Kernel::Vector_3 v = polyline[polyline.size()-1] - polyline[polyline.size()-2];
        double dist = CGAL::sqrt(v.squared_length());
        if(current_group->min_dist == -1 || dist<current_group->min_dist)
        {
          current_group->min_dist = dist;
          current_spin->setValue(current_group->min_dist/2.0);
        }
      }
      current_group->points_stack.pop_back();
      current_polyline->invalidateOpenGLBuffers();
      current_polyline->itemChanged();
      if(current_group->points_stack.size() == 0)
        ui_widget.redoButton->setEnabled(false);
    }
    else if (mode == ADD_POINT_AND_DEFORM)
    {
      if(current_group->operations_redone.back() == 0)
      {
        if(current_group->points_stack.size() == 0)
          return;
        Point_3 point = current_group->points_stack.back();
        for(
            Polyhedron::Vertex_iterator vit = current_group->surface->polyhedron()->vertices_begin();
            vit != current_group->surface->polyhedron()->vertices_end();
            ++vit)
        {
          if(vit->point() == point)
          {
            current_group->control_points.push_back(vit);
            break;
          }
        }
        current_group->ordered_control_points.push_back(point);
        current_group->control_points_planes.push_back(current_group->planes_stack.back());
        current_group->planes_stack.pop_back();
        current_group->points_stack.pop_back();

        current_group->control_points_item->point_set()->insert(point);
        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
      }
      else if (current_group->operations_redone.back() == 1)
      {
        int i=0;
        Q_FOREACH(Polyhedron::Vertex_handle vh, current_group->control_points)
        {
          if(vh->point() == current_group->ordered_control_points.back())
          {
            current_group->control_points.erase(current_group->control_points.begin()+i);
            break;
          }
          ++i;
        }
        current_group->repoints_stack.push_back(current_group->ordered_control_points.back());
        current_group->ordered_control_points.pop_back();
        current_group->replanes_stack.push_back(current_group->control_points_planes.back());
        current_group->control_points_planes.pop_back();
        current_group->control_points_item->point_set()->clear();
        Q_FOREACH(Polyhedron::Vertex_iterator vit, current_group->control_points)
        {
          current_group->control_points_item->point_set()->insert(vit->point());
        }
        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
      }
      else if(current_group->operations_redone.back() == 2)
      {
        //case point is on polyline
        if(current_group->reedit_planes_stack.back() == Kernel::Plane_3(1,1,1,0) &&
           current_group->reedit_planes_stack[current_group->reedit_planes_stack.size()-2] == Kernel::Plane_3(1,1,1,0))
        {
          Point_3 first = current_group->reedit_points_stack.back();
          current_group->edit_points_stack.push_back(first);
          current_group->reedit_points_stack.pop_back();
          Point_3 second = current_group->reedit_points_stack.back();
          current_group->edit_points_stack.push_back(second);
          current_group->reedit_points_stack.pop_back();
          current_group->edit_planes_stack.push_back(Kernel::Plane_3(1,1,1,0));
          current_group->edit_planes_stack.push_back(Kernel::Plane_3(1,1,1,0));
          current_group->reedit_planes_stack.pop_back();
          current_group->reedit_planes_stack.pop_back();
         ////////////////////////////////////
          int id = -1, i = 0;
          std::vector<Point_3> &l_poly = current_group->leader_poly->polylines.back();
          std::vector<Point_3> &g_poly = current_group->generator_poly->polylines.back();
          Q_FOREACH(Point_3 cp, l_poly)
          {
            if(cp == first)
            {
              id = i;
              break;
            }
            ++i;
          }
          if(id>-1)
          {
            *(l_poly.begin()+id) = second;
            current_group->leader_poly->invalidateOpenGLBuffers();
            current_group->leader_poly->itemChanged();
          }
          else
          {
            i = 0;
            Q_FOREACH(Point_3 cp, g_poly)
            {
              if(cp == first)
              {
                id = i;
                break;
              }
              ++i;
            }
            if(id >-1)
            {
              *(g_poly.begin()+id) = second;
              current_group->generator_poly->invalidateOpenGLBuffers();
              current_group->generator_poly->itemChanged();
            }
            else
            {
              std::cerr<<"cannot find point to edit back."<<std::endl;
              return;
            }
          }
          Point_set_3<Kernel>::iterator pit;
          for (pit = current_group->control_points_item->point_set()->begin();
               pit != current_group->control_points_item->point_set()->end();
               ++pit)
          {
            if(current_group->control_points_item->point_set()->point(*pit) == first)
            {
              current_group->control_points_item->point_set()->select(pit);
              current_group->control_points_item->point_set()->delete_selection();
              current_group->control_points_item->point_set()->insert(second);
              break;
            }
          }
          minEnergy();
          //////////////////////////////////
        }
        //case point is simple control_point
        else
        {
          Point_3 first = current_group->reedit_points_stack.back();
          current_group->edit_points_stack.push_back(first);
          current_group->reedit_points_stack.pop_back();
          Point_3 second = current_group->reedit_points_stack.back();
          current_group->edit_points_stack.push_back(second);
          current_group->reedit_points_stack.pop_back();

          Kernel::Plane_3 first_plane = current_group->reedit_planes_stack.back();
          current_group->edit_planes_stack.push_back(first_plane);
          current_group->reedit_planes_stack.pop_back();
          Kernel::Plane_3 second_plane = current_group->reedit_planes_stack.back();
          current_group->edit_planes_stack.push_back(second_plane);
          current_group->reedit_planes_stack.pop_back();

          int id = -1;
          Q_FOREACH(Point_3 cp, current_group->ordered_control_points)
          {
            ++id;
            if(cp == first)
            {
              break;
            }
          }
          current_group->ordered_control_points[id] = second;
          current_group->control_points_planes[id] = second_plane;

          current_group->control_points_item->point_set()->clear();
          Point_set_3<Kernel>::iterator pit;
          for (pit = current_group->control_points_item->point_set()->begin();
               pit != current_group->control_points_item->point_set()->end();
               ++pit)
          {
            if(current_group->control_points_item->point_set()->point(*pit) == first)
            {
              current_group->control_points_item->point_set()->select(pit);
              current_group->control_points_item->point_set()->delete_selection();
              current_group->control_points_item->point_set()->insert(second);
              break;
            }
          }
          minEnergy();
        }

        current_group->control_points_item->point_set()->clear();
        Q_FOREACH(Polyhedron::Vertex_iterator vit, current_group->control_points)
        {
          current_group->control_points_item->point_set()->insert(vit->point());
        }
        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
      }

      current_group->operations_done.push_back(current_group->operations_redone.back());
      current_group->operations_redone.pop_back();
      if(current_group->operations_redone.size() == 0)
        ui_widget.redoButton->setEnabled(false);
    }
  }

  void clear_redo()
  {
    ui_widget.redoButton->setEnabled(false);
    current_group->points_stack.clear();
    current_group->planes_stack.clear();
    current_group->reedit_planes_stack.clear();
    current_group->reedit_points_stack.clear();
    current_group->operations_redone.clear();

  }

  void setEdgeSize(double d) { current_group->edgeSize = d; }

  //remesh the polyhedron to optimize the surface
  void remesh()
  {
    if(!current_group->surface )
      return;
    current_group->last_remesh = current_group->edgeSize;
    if(current_group->edgeSize == 0)
      return;
    Polyhedron* poly = current_group->surface->polyhedron();
    Vertex_set is_constrained_set;
    Q_FOREACH(Polyhedron::Vertex_handle vh, current_group->control_points)
      is_constrained_set.insert(vh);

    Is_constrained_map vcm(&is_constrained_set);
    CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(*poly),
                                                       current_group->edgeSize,
                                                       *poly,
                                                       CGAL::Polygon_mesh_processing::parameters::vertex_is_constrained_map(vcm)
                                                       .number_of_relaxation_steps(100));
    current_group->control_points.clear();
    Q_FOREACH(Polyhedron::Vertex_handle vh, is_constrained_set)
      current_group->control_points.push_back(vh);

    current_group->surface->invalidateOpenGLBuffers();
    current_group->surface->itemChanged();
  }

  void minEnergy()
  {
    current_group->control_points.clear();
    //create points
    std::vector<Point_3> points;
    std::vector<Point_3>& g_polyline = current_group->generator_poly->polylines.back();
    std::vector<Point_3>& l_polyline = current_group->leader_poly->polylines.back();
    std::vector<Point_3> control_pos;
    //compute the points
    for(std::size_t i=0; i< g_polyline.size(); ++i)
    {
      Kernel::Vector_3 offset = g_polyline[i]-g_polyline[current_group->g_id];
      for(std::size_t j=0; j<l_polyline.size(); ++j)
      {
        if(j==static_cast<std::size_t>(current_group->l_id))
        {
          points.push_back(g_polyline[i]);
          control_pos.push_back(points.back());
        }

        points.push_back(l_polyline[j]+offset);
        if(i == static_cast<std::size_t>(current_group->g_id))
          control_pos.push_back(points.back());
      }
      if(static_cast<std::size_t>(current_group->l_id) == l_polyline.size())
      {
        points.push_back(g_polyline[i]);
        control_pos.push_back(points.back());
      }
    }

    //update Polyhedron item

    //re-create initial polyhedron.
    Polyhedron *polyhedron = current_group->surface->polyhedron();
    polyhedron->clear();
    Build_polyhedron<Polyhedron::HalfedgeDS> builder(points, (int)g_polyline.size(), (int)l_polyline.size()+1);
    polyhedron->delegate(builder);
    // Init the indices of the halfedges and the vertices.
    set_halfedgeds_items_id(*polyhedron);

    //re-compute control_points
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
          current_group->control_points.push_back(vit);
          control_pos.erase(control_pos.begin()+i);
          break;
        }
        ++i;
      }
      if(control_pos.empty())
        break;
    }
    //remesh
    if(current_group->last_remesh > 0)
    {
      Vertex_set is_constrained_set;
      Q_FOREACH(Polyhedron::Vertex_handle vh, current_group->control_points)
        is_constrained_set.insert(vh);

      Is_constrained_map vcm(&is_constrained_set);
      CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(*polyhedron),
                                                         current_group->last_remesh,
                                                         *polyhedron,
                                                         CGAL::Polygon_mesh_processing::parameters::vertex_is_constrained_map(vcm));

      current_group->control_points.clear();
      Q_FOREACH(Polyhedron::Vertex_handle vh, is_constrained_set)
        current_group->control_points.push_back(vh);
    }
    //deform
    typedef CGAL::AABB_halfedge_graph_segment_primitive<Polyhedron> HGSP;
    typedef CGAL::AABB_traits<Kernel, HGSP>                         AABB_traits;
    typedef CGAL::AABB_tree<AABB_traits>                            AABB_tree;
    //Slice the Polyhedron along the picked plane
    int plane_id = 0;
    Q_FOREACH(Point_3 ctrl_p, current_group->ordered_control_points)
    {
      Kernel::Plane_3 &plane = current_group->control_points_planes[plane_id++];
      std::vector<std::vector<Point_3> > slices;
      AABB_tree tree(edges(*polyhedron).first, edges(*polyhedron).second, *polyhedron);
      CGAL::Polygon_mesh_slicer<Polyhedron, Kernel> slicer_aabb(*polyhedron, tree);
      slicer_aabb(plane, std::back_inserter(slices));
      if(slices.empty())
      {
        messageInterface->warning("An error has occured.");
        std::pair<Point_3, AABB_tree::Primitive_id> res=tree.closest_point_and_primitive(ctrl_p);
        current_group->control_points.push_back(CGAL::Euler::split_edge(halfedge(res.second, *polyhedron), *polyhedron)->vertex());
        current_group->control_points.back()->point() = res.first;
        continue;
      }
      //find the closest slice
      boost::tuple<double, int, int> min_squared_dist(-1,-1, -1);
      for(std::size_t i = 0; i<slices.size(); ++i)
      {
        for(std::size_t j = 0; j<slices[i].size()-1; ++j)
        {
          Kernel::Segment_3 segment(slices[i][j], slices[i][j+1]);
          double sq_dist = CGAL::squared_distance(ctrl_p, segment);
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
      Polyhedron::Halfedge_handle h1(halfedge(pid1, *polyhedron));
      Polyhedron::Halfedge_handle h2(halfedge(pid2, *polyhedron));
      //find the triangle that contains those two edges
      Polyhedron::Facet_handle closest_triangle = h1->facet();
      if(!(closest_triangle == h2->facet()
           || closest_triangle == h2->opposite()->facet()))
      {
        closest_triangle = h1->opposite()->facet();
      }
      if(closest_triangle == NULL)
      {
        BOOST_FOREACH(Polyhedron::Facet_handle f1, CGAL::faces_around_target(h1, *polyhedron))
        {
          BOOST_FOREACH(Polyhedron::Facet_handle f2, CGAL::faces_around_target(h2, *polyhedron))
          {
            if(f2==f1 && f2 != NULL)
            {
              closest_triangle = f1;
              break;
            }
          }
        }
        if(closest_triangle == NULL)
        {
          messageInterface->error("Cannot find the closest triangle.");
          return ;
        }
      }
      // add triangle's center to the mesh
      double x(0), y(0), z(0);
      Polyhedron::Halfedge_around_facet_circulator hafc = closest_triangle->facet_begin();
      Polyhedron::Halfedge_around_facet_circulator end = hafc;
      CGAL_For_all(hafc, end)
      {
        x+=hafc->vertex()->point().x(); y+=hafc->vertex()->point().y(); z+=hafc->vertex()->point().z();
      }
      Polyhedron::Halfedge_handle center = CGAL::Euler::add_center_vertex(closest_triangle->facet_begin(), *polyhedron);
      center->vertex()->point() = Point_3(x/3.0, y/3.0, z/3.0);
      current_group->control_points.push_back(center->vertex());
    }
    // Init the indices of the halfedges and the vertices.
    set_halfedgeds_items_id(*polyhedron);

    Surface_mesh_deformation deform_mesh(*polyhedron);
    //Define the ROI
    polyhedron->normalize_border();
    for(Polyhedron::Vertex_iterator vit = polyhedron->vertices_begin();
        vit != polyhedron->vertices_end();
        ++vit)
    {
      deform_mesh.insert_roi_vertex(vit);
    }
    //add the control points
    deform_mesh.insert_control_vertices(current_group->control_points.begin(), current_group->control_points.end());
    //deform
    bool is_matrix_factorization_OK = deform_mesh.preprocess();
    if(!is_matrix_factorization_OK){
      std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
      return;
    }
    int ctrl_id = current_group->control_limit;
    Q_FOREACH(Point_3 ctrl_p, current_group->ordered_control_points)
    {
      Surface_mesh_deformation::Point constrained_pos(ctrl_p.x(), ctrl_p.y(), ctrl_p.z());
      deform_mesh.set_target_position(current_group->control_points[ctrl_id++], constrained_pos);
    }
    deform_mesh.deform(100, 1e-8);

    current_group->surface->invalidateOpenGLBuffers();
    current_group->surface->itemChanged();
  }

  void checkEdit(int id)
  {
    SurfaceGroup* group = qobject_cast<SurfaceGroup*>(scene->item(id));
    if(!group)
    {
      Scene_item* item = scene->item(id);
      if(!item)
      {
        return;
      }
      else
        group = qobject_cast<SurfaceGroup*>(item->parentGroup());
    }
    if(!group)
    {
      ui_widget.editButton->setEnabled(false);
    }
    else
      ui_widget.editButton->setEnabled(true);
  }

  void edit()
  {
    SurfaceGroup* group = qobject_cast<SurfaceGroup*>(scene->item(scene->mainSelectionIndex()));
    if(!group)
    {
      Scene_polyhedron_item* poly = qobject_cast<Scene_polyhedron_item*>(scene->item(scene->mainSelectionIndex()));
      if(!poly)
      {
        messageInterface->information("Please select a rift");
        return;
      }
      else
        group = qobject_cast<SurfaceGroup*>(poly->parentGroup());
      if(!group)
      {
        messageInterface->information("Please select a rift");
        return;
      }
    }
    mode = ADD_POINT_AND_DEFORM;
    current_group = group;
    Q_FOREACH(SurfaceGroup* sgroup, surface_groups)
    {
      if(sgroup != current_group)
      {
        sgroup->setVisible(false);
      }
    }
    ui_widget.newPolylineButton->setEnabled(false);
    ui_widget.regenButton->setEnabled(true);
    ui_widget.smoothButton->setEnabled(true);
    ui_widget.createSurfaceButton->setText("Remesh");
    disconnect(ui_widget.createSurfaceButton, &QPushButton::clicked,
               this, &SurfaceFromPickedPointsPlugin::add_surface);
    connect(ui_widget.createSurfaceButton, &QPushButton::clicked,
            this, &SurfaceFromPickedPointsPlugin::remesh);
    ui_widget.createSurfaceButton->setEnabled(true);

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
  SurfaceGroup* current_group;
  Polyhedron::Vertex_handle sel_handle;
  bool is_editing;
  bool is_selecting;
  std::vector<SurfaceGroup*> surface_groups;
  bool find_plane(QMouseEvent* e, Kernel::Plane_3& plane);
  std::vector<int> hidden_planes;
  void extendSurface(const qglviewer::Vec& p, int checked)
  {
    Point_3 new_point = Point_3(0,0,0);
    Polyhedron* poly = current_group->surface->polyhedron();
    poly->normalize_border();
    Point_3 point(p.x, p.y, p.z);
    if(checked !=1)
    {
      //report the vector between the closest point of the bordure and the picked point
      // at the right end of the generator.
      std::vector<Point_3>& l_polyline = current_group->leader_poly->polylines.back();
      double dist = Kernel::Vector_3(l_polyline.front(), point).squared_length();
      if(Kernel::Vector_3(l_polyline.back(), point).squared_length() > dist)
      {
        new_point = current_group->l_plane->projection(point);
        l_polyline.insert(
              l_polyline.begin(),
              new_point);
        if(current_group->l_id > 0)
          current_group->l_id++;
      }
      else
      {
        new_point = current_group->l_plane->projection(point);
        l_polyline.insert(
              l_polyline.end(),
              new_point);
      }
      current_group->extend_points_stack.push_back(new_point);
      current_group->leader_poly->invalidateOpenGLBuffers();
      current_group->leader_poly->itemChanged();
      current_group->control_limit++;
      current_group->control_points_item->point_set()->insert(new_point);
      current_group->operations_done.push_back(3);
    }
    if(checked !=0)
    {

      //report the vector between the closest point of the bordure and the picked point
      // at the right end of the generator.
      std::vector<Point_3>& g_polyline = current_group->generator_poly->polylines.back();
      double dist = Kernel::Vector_3(g_polyline.front(), point).squared_length();
      if(Kernel::Vector_3(g_polyline.back(), point).squared_length() > dist)
      {
        new_point = current_group->g_plane->projection(point);
        g_polyline.insert(
              g_polyline.begin(),
              new_point);
        if(current_group->g_id > 0)
          current_group->g_id++;
      }
      else
      {
        new_point = current_group->g_plane->projection(point);
        g_polyline.insert(
              g_polyline.end(),
              new_point);
      }
      current_group->extend_points_stack.push_back(new_point);
      current_group->generator_poly->invalidateOpenGLBuffers();
      current_group->generator_poly->itemChanged();
      current_group->control_limit++;
      current_group->control_points_item->point_set()->insert(new_point);
      current_group->operations_done.push_back(3);
    }


    current_group->control_points_item->invalidateOpenGLBuffers();
    current_group->control_points_item->itemChanged();
    minEnergy();
  }

  /*!
   * \brief checkExtend distinguishes the 3 cases when extending the surface
   * \param point the new point
   * \param projGf the projection of the first point of the generator in g_plane
   * \param projGl the projection of the last point of the generator in g_plane
   * \param projLf the projection of the first point of the leader in l_plane
   * \param projLl the projection of the last point of the leader in l_plane
   * \param g_plane the plane in which the generator was picked
   * \param l_plane the plane in which the leader was picked
   * \return 0, 1 or 2
   *  2   |      0      | 2
   *___________________________
   *      |             |
   *      |             |
   *      |             |
   *   1  |      3      | 1
   *      |             |
   *      |             |
   *__________________________
   *      |             |
   *   2  |      0      | 2
   */
  int checkExtend(Kernel::Point_3& point)
  {
    bool is_in_g_bounds = false;
    bool is_in_l_bounds = false;
    Kernel::Point_2 projGf(current_group->g_plane->to_2d(current_group->generator_poly->polylines.back().front())),
        projGl(current_group->g_plane->to_2d(current_group->generator_poly->polylines.back().back())),
        projLf(current_group->l_plane->to_2d(current_group->leader_poly->polylines.back().front())),
        projLl(current_group->l_plane->to_2d(current_group->leader_poly->polylines.back().back()));
    //Find the varying coord in g_plane
    int varCoord = 0;
    double variation = CGAL::abs(projGf.x() - projGl.x());
    if(CGAL::abs(projGf.y() - projGl.y()) > variation)
      varCoord = 1;
    //first check
    Kernel::Point_2 projP = current_group->g_plane->to_2d(point);
    if(projP[varCoord] >= (std::min)(projGf[varCoord], projGl[varCoord]) &&
       projP[varCoord] <= (std::max)(projGf[varCoord], projGl[varCoord]))
    {
      is_in_g_bounds = true;
    }

    //Find the varying coord in l_plane
    varCoord = 0;
    variation = CGAL::abs(projLf.x() - projLl.x());
    if(CGAL::abs(projLf.y() - projLl.y()) > variation)
      varCoord = 1;
    //first check
    projP = current_group->l_plane->to_2d(point);
    if(projP[varCoord] >= (std::min)(projLf[varCoord], projLl[varCoord]) &&
       projP[varCoord] <= (std::max)(projLf[varCoord], projLl[varCoord]))
    {
      is_in_l_bounds = true;
    }

    if(!is_in_g_bounds && is_in_l_bounds)
      return 1;
    if(is_in_g_bounds && !is_in_l_bounds)
      return 0;
    if(!is_in_g_bounds && !is_in_l_bounds)
      return 2;
    //if(is_in_g_bounds && is_in_l_bounds)
    return 3;

  }
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
  if(event->type() == QEvent::KeyPress
     && static_cast<QKeyEvent*>(event)->key()==Qt::Key_E)
  {
    is_selecting= true;
  }
  else if(event->type() == QEvent::KeyRelease
          && static_cast<QKeyEvent*>(event)->key()==Qt::Key_E)
  {
    is_selecting= false;
  }
  if(event->type()==QEvent::MouseButtonPress)
  {
    QMouseEvent* e= static_cast<QMouseEvent*>(event);
    if(e->modifiers() == Qt::NoModifier &&
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
      if(!found)
        return false;
      if(is_selecting)
      {
        if(mode != ADD_POINT_AND_DEFORM)
          //return false;
          if(object == mw)
          {
            viewer->setFocus();
            //return false;
          }
        Point_3 p(point.x, point.y, point.z);
        double min_dist = -1;
        Q_FOREACH(Polyhedron::Vertex_handle vh, current_group->control_points)
        {
          double dist = Kernel::Vector_3(vh->point(), p).squared_length();
          if(min_dist == -1 || dist < min_dist)
          {
            min_dist = dist;
            sel_handle = vh;
            is_editing = true;
          }
        }
        Point_set_3<Kernel>::iterator pit;
        current_group->control_points_item->point_set()->unselect_all();
        for (pit = current_group->control_points_item->point_set()->begin();
             pit != current_group->control_points_item->point_set()->end();
             ++pit)
        {

          if(current_group->control_points_item->point_set()->point(*pit) == sel_handle->point())
          {
            current_group->control_points_item->point_set()->select(pit);
            break;
          }
        }

        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
        is_selecting = false;
        return false;

      }
    }
    else if(e->modifiers() == Qt::ShiftModifier &&
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
      if(!found)
        return false;
      switch(mode)
      {
      case ADD_POLYLINE:
      {
        clear_redo();
        ui_widget.UndoButton->setEnabled(true);
        Scene_polylines_item* current_polyline = NULL;
        QDoubleSpinBox* current_spin = ui_widget.edgeSpinBox;
        if(current_group->generator_is_created)
        {
          current_polyline = current_group->generator_poly;
          if(!current_group->g_plane)
          {
            current_group->g_plane = new Kernel::Plane_3();
            if(!find_plane(e, *current_group->g_plane))
            {
              delete current_group->g_plane;
              current_group->g_plane = NULL;
              return false;
            }
          }
          else
          {
            //project point on plane
            if ( !current_group->g_plane->has_on(Point_3(point.x, point.y, point.z)))
            {
              qglviewer::Vec pos = viewer->camera()->position();
              Kernel::Line_3 ray(Point_3(pos.x, pos.y, pos.z), Point_3(point.x, point.y, point.z));
              Point_3 res = boost::get<Point_3>(*intersection(*current_group->g_plane, ray));
              point = qglviewer::Vec(res.x(), res.y(), res.z());
            }
          }
          if(current_group->generator_poly->polylines.back().size() >=1)
            ui_widget.newPolylineButton->setEnabled(true);
        }
        else if(current_group->leader_poly)
        {
          current_polyline = current_group->leader_poly;
          if(!current_group->l_plane)
          {
            current_group->l_plane = new Kernel::Plane_3();
            if(!find_plane(e, *current_group->l_plane))
            {
              delete current_group->l_plane;
              current_group->l_plane = NULL;
              return false;
            }
          }
          else
          {
            //project point on plane
            if ( !current_group->l_plane->has_on(Point_3(point.x, point.y, point.z)))
            {
              qglviewer::Vec pos = viewer->camera()->position();
              Kernel::Line_3 ray(Point_3(pos.x, pos.y, pos.z), Point_3(point.x, point.y, point.z));
              Point_3 res = boost::get<Point_3>(*intersection(*current_group->l_plane, ray));
              point = qglviewer::Vec(res.x(), res.y(), res.z());
            }
          }
          if(current_group->leader_poly->polylines.back().size() >=1)
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
          std::vector<Point_3>& polyline = current_polyline->polylines.back();
          polyline.push_back(Point_3(point.x, point.y, point.z));
          if(polyline.size() >= 2)
          {
            Kernel::Vector_3 v = polyline[polyline.size()-1] - polyline[polyline.size()-2];
            double dist = CGAL::sqrt(v.squared_length());
            if(current_group->min_dist == -1 || dist<current_group->min_dist)
            {
              current_group->min_dist = dist;
              current_spin->setValue(current_group->min_dist/2.0);
            }
          }
          current_polyline->invalidateOpenGLBuffers();
          current_polyline->itemChanged();
        break;
      }
      case ADD_POINT_AND_DEFORM:
      {
        ui_widget.UndoButton->setEnabled(true);
        typedef CGAL::AABB_halfedge_graph_segment_primitive<Polyhedron> HGSP;
        typedef CGAL::AABB_traits<Kernel, HGSP>                         AABB_traits;
        typedef CGAL::AABB_tree<AABB_traits>                            AABB_tree;
        if(!current_group->surface)
          return false;
        Kernel::Plane_3 plane;
        if(!find_plane(e,plane))
          return false;
        //project point on plane
        if ( !plane.has_on(Point_3(point.x, point.y, point.z)))
        {
          qglviewer::Vec pos = viewer->camera()->position();
          Kernel::Line_3 ray(Point_3(pos.x, pos.y, pos.z), Point_3(point.x, point.y, point.z));
          Point_3 res = boost::get<Point_3>(*intersection(plane, ray));
          point = qglviewer::Vec(res.x(), res.y(), res.z());
        }
        Polyhedron &polyhedron = *current_group->surface->polyhedron();
        // Init the indices of the halfedges and the vertices.
        set_halfedgeds_items_id(polyhedron);
        if(!is_editing)
        {
          clear_redo();
          Kernel::Point_3 p(point.x, point.y, point.z);
          //Slice the Polyhedron along the picked plane
          std::vector<std::vector<Point_3> > slices;
          AABB_tree tree(edges(polyhedron).first, edges(polyhedron).second, polyhedron);
          int checked = checkExtend(p);
          if(checked !=3)
          {
            extendSurface(point, checked);
            return false;
          }
          CGAL::Polygon_mesh_slicer<Polyhedron, Kernel> slicer_aabb(polyhedron, tree);
          slicer_aabb(plane, std::back_inserter(slices));
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
          if(closest_triangle == NULL)
          {
            BOOST_FOREACH(Polyhedron::Facet_handle f1, CGAL::faces_around_target(h1, polyhedron))
            {
              BOOST_FOREACH(Polyhedron::Facet_handle f2, CGAL::faces_around_target(h2, polyhedron))
              {
                if(f2==f1 && f2 != NULL)
                {
                  closest_triangle = f1;
                  break;
                }
              }
            }
            if(closest_triangle == NULL)
            {
              messageInterface->error("Cannot find the closest triangle.");
              return false;
            }
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
          current_group->control_points.push_back(center->vertex());
          //add the control points
          deform_mesh.insert_control_vertices(current_group->control_points.begin(), current_group->control_points.end());
          //deform
          bool is_matrix_factorization_OK = deform_mesh.preprocess();
          if(!is_matrix_factorization_OK){
            std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
            return false;
          }
          Surface_mesh_deformation::Point constrained_pos(point.x, point.y, point.z);
          current_group->control_points_planes.push_back(plane);
          current_group->ordered_control_points.push_back(Point_3(point.x, point.y, point.z));
          deform_mesh.set_target_position(target(center, polyhedron), constrained_pos);
          deform_mesh.deform(3, 1e-4);
          //update the item

          current_group->control_points_item->point_set()->insert(constrained_pos);
          current_group->surface->invalidateOpenGLBuffers();
          current_group->surface->itemChanged();
          current_group->operations_done.push_back(0);
        }
        else
        {
          int id = -1, i = 0;
          std::vector<Point_3> &l_poly = current_group->leader_poly->polylines.back();
          std::vector<Point_3> &g_poly = current_group->generator_poly->polylines.back();
          Q_FOREACH(Point_3 cp, l_poly)
          {
            if(cp == sel_handle->point())
            {
              id = i;
              break;
            }
            ++i;
          }
          if(id>-1)
          {
            current_group->edit_points_stack.push_back(*(l_poly.begin()+id));
            *(l_poly.begin()+id) = Point_3(point.x, point.y, point.z);
            current_group->edit_points_stack.push_back(*(l_poly.begin()+id));
            current_group->edit_planes_stack.push_back(Kernel::Plane_3(1,1,1,0));
            current_group->edit_planes_stack.push_back(Kernel::Plane_3(1,1,1,0));
            current_group->leader_poly->invalidateOpenGLBuffers();
            current_group->leader_poly->itemChanged();
          }
          else
          {
            i = 0;
            Q_FOREACH(Point_3 cp, g_poly)
            {
              if(cp == sel_handle->point())
              {
                id = i;
                break;
              }
              ++i;
            }
            if(id >-1)
            {
              current_group->edit_points_stack.push_back(*(g_poly.begin()+id));
              *(g_poly.begin()+id) = Point_3(point.x, point.y, point.z);
              current_group->edit_points_stack.push_back(*(g_poly.begin()+id));
              current_group->edit_planes_stack.push_back(Kernel::Plane_3(1,1,1,0));
              current_group->edit_planes_stack.push_back(Kernel::Plane_3(1,1,1,0));
              current_group->generator_poly->invalidateOpenGLBuffers();
              current_group->generator_poly->itemChanged();
            }
            else
            {
              id = -1;
              Q_FOREACH(Point_3 cp, current_group->ordered_control_points)
              {
                ++id;
                if(cp == sel_handle->point())
                {
                  break;
                }
              }
              current_group->edit_points_stack.push_back(current_group->ordered_control_points[id]);
              current_group->ordered_control_points[id] = Point_3(point.x, point.y, point.z);
              current_group->edit_points_stack.push_back(current_group->ordered_control_points[id]);
              current_group->edit_planes_stack.push_back(current_group->control_points_planes[id]);
              current_group->control_points_planes[id] = plane;
              current_group->edit_planes_stack.push_back(current_group->control_points_planes[id]);
            }
          }
          Point_set_3<Kernel>::iterator pit;
          for (pit = current_group->control_points_item->point_set()->begin();
               pit != current_group->control_points_item->point_set()->end();
               ++pit)
          {
            if(current_group->control_points_item->point_set()->point(*pit) == sel_handle->point())
            {
              current_group->control_points_item->point_set()->delete_selection();
              current_group->control_points_item->point_set()->insert(Point_3(point.x, point.y, point.z));
              break;
            }
          }

          sel_handle->point() = Point_3(point.x, point.y, point.z);
          is_editing = false;
          minEnergy();
          current_group->operations_done.push_back(2);
        }
        current_group->control_points_item->invalidateOpenGLBuffers();
        current_group->control_points_item->itemChanged();
        break;
      }
      default:
        break;
      }
      return true;
    }
    else if(e->modifiers() == Qt::ControlModifier &&
            e->buttons() == Qt::LeftButton)
    {
      if(mode != ADD_POINT_AND_DEFORM)
        return false;

      QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
      if(object == mw)
      {
        viewer->setFocus();
        return false;
      }
      bool found = false;
      qglviewer::Vec p = viewer->camera()->pointUnderPixel(e->pos(), found);
      if(!found)
        return false;

      Point_3 point(p.x, p.y, p.z);
      double min_dist = -1;
      int id(-1), i(0);
      Point_3 target = Point_3(0,0,0);
      Q_FOREACH(Polyhedron::Vertex_handle vh, current_group->control_points)
      {
        double dist = Kernel::Vector_3(vh->point(), point).squared_length();
        if(min_dist == -1 || dist < min_dist)
        {
          min_dist = dist;
          id = i;
          target = vh->point();
        }
        ++i;
      }
      int save_id = id;
      min_dist = -1; id = -1; i = 0;
      Q_FOREACH(Point_3 cp, current_group->leader_poly->polylines.back())
      {
        ++i;
        if(cp == target)
        {
          id = i;
          break;
        }
      }
      if(id >-1)
      {
        return false;
      }
      else
      {
        i = 0;
        Q_FOREACH(Point_3 cp, current_group->generator_poly->polylines.back())
        {
          ++i;
          if(cp == target)
          {
            id = i;
            break;
          }
        }
      }
      if(id >-1)
      {
        return false;
      }

      min_dist = -1; id = -1; i = 0;
      Q_FOREACH(Point_3 cp, current_group->ordered_control_points)
      {
        if(current_group->control_points[save_id]->point() == cp)
        {
          id = i;
          break;
        }
        ++i;
      }
      current_group->control_points.erase(current_group->control_points.begin()+save_id);
      current_group->repoints_stack.push_back(*(current_group->ordered_control_points.begin()+id));
      current_group->ordered_control_points.erase(current_group->ordered_control_points.begin()+id);
      current_group->replanes_stack.push_back(*(current_group->control_points_planes.begin()+id));
      current_group->control_points_planes.erase(current_group->control_points_planes.begin()+id);

      current_group->control_points_item->point_set()->clear();
      Q_FOREACH(Polyhedron::Vertex_iterator vit, current_group->control_points)
      {
        current_group->control_points_item->point_set()->insert(vit->point());
      }
      current_group->control_points_item->invalidateOpenGLBuffers();
      current_group->control_points_item->itemChanged();
      current_group->operations_done.push_back(1);
      return true;
    }
  }
  return false;
}
#include "Surface_from_picked_points_plugin.moc"


