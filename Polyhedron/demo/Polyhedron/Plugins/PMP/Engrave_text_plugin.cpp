//General Plugin Data
#include <CGAL/Three/Polyhedron_demo_plugin_helper.h>
#include <CGAL/Three/Three.h>

#include "ui_Engrave_dock_widget.h"
//Items
#include "Scene_surface_mesh_item.h"
#include "Scene_polyhedron_selection_item.h"
#include "Scene_polylines_item.h"
#include "Messages_interface.h"

#include <CGAL/Surface_mesh_parameterization/Error_code.h>
#include <CGAL/surface_mesh_parameterization.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/extrude.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/double.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

#include <CGAL/linear_least_squares_fitting_2.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/centroid.h>

#include <CGAL/boost/graph/Face_filtered_graph.h>

#include <QPainterPath>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QDialog>

#include <CGAL/Qt/GraphicsViewNavigation.h>

#include <CGAL/Segment_2.h>
#include <CGAL/box_intersection_d.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Cartesian_converter.h>
#include <CGAL/constructions_d.h>

typedef CGAL::Box_intersection_d::Box_with_info_d<double, 2,std::size_t> Box;
using namespace CGAL::Three;
namespace SMP = CGAL::Surface_mesh_parameterization;
typedef EPICK::Point_2                                   Point_2;
typedef EPICK::Point_3                                   Point_3;
typedef CGAL::Exact_predicates_exact_constructions_kernel EPECK;
typedef CGAL::Cartesian_converter<EPECK, EPICK> Exact_to_double;
typedef boost::graph_traits<SMesh>::
edge_descriptor          edge_descriptor;
typedef boost::graph_traits<SMesh>::
halfedge_descriptor      halfedge_descriptor;
typedef boost::graph_traits<SMesh>::
vertex_descriptor        vertex_descriptor;

typedef boost::unordered_set<boost::graph_traits<SMesh>::
face_descriptor>                                         Component;

struct FaceInfo2
{
  FaceInfo2(){}
  int nesting_level;
  bool in_domain(){
    return nesting_level%2 == 1;
  }
};

template<typename PMAP,
         typename NMAP>
struct Bot
{
  Bot(NMAP nmap,
      double d,
      PMAP pmap):d(d),
    pmap(pmap),
    nmap(nmap){}
  template<typename VD, typename T>
  void operator()(const T& v1,VD v2) const
  {
    put(pmap, v2, get(pmap, v2)-d*get(nmap, v1));
  }
  double d;
  PMAP pmap;
  NMAP nmap;
  
};

template<typename PMAP,
         typename NMAP>
struct Top
{
  Top(NMAP nmap,
      PMAP pmap,
      double d):d(d),
    nmap(nmap),
    pmap(pmap){}
  
  template<typename VD, typename T>
  void operator()(const T& v1, VD v2) const
  {
    put(pmap, v2, get(pmap, v2)+d*get(nmap, v1));
  }
  double d;
  NMAP nmap;
  PMAP pmap;
};

typedef EPICK                                                        Gt;
typedef CGAL::Triangulation_vertex_base_2<Gt>                        Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,Gt >    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<Gt,Fbb>          Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                TDS;
typedef CGAL::No_intersection_tag                                   Tag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Gt, TDS, Tag>    CDT;

//Parameterization and text displaying
class ParamItem : public QGraphicsItem
{
public :
  ParamItem(Component* component,
            const std::vector<std::vector<EPICK::Point_2> > &polylines,
            EPICK::Aff_transformation_2 transfo,
            SMesh* graph,
            QRectF brect)
    :
      QGraphicsItem(),
      bounding_rect(brect),
      component(component),
      polylines(polylines),
      graph(graph),
      transfo(transfo){}
  
  ~ParamItem()
  {
    delete component;
  }
  
  QRectF boundingRect() const
  {
    return bounding_rect;
  }
  
  void set_transfo(EPICK::Aff_transformation_2 t){ transfo = t;}
  
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
  {
    QPen pen;
    QBrush brush;
    brush.setColor(QColor(100, 100, 255));
    brush.setStyle(Qt::SolidPattern);
    pen.setColor(Qt::black);
    pen.setWidth(0);
    painter->setPen(pen);
    painter->setBrush(brush);
    SMesh::Property_map<halfedge_descriptor,std::pair<float, float> > uv;
    uv = graph->add_property_map<halfedge_descriptor,std::pair<float, float> >
        ("h:uv",std::make_pair(0.0f,0.0f)).first;
    for( Component::iterator
         fi = component->begin();
         fi != component->end();
         ++fi)
    {
      boost::graph_traits<SMesh>::face_descriptor f(*fi);
      QPointF points[3];
      boost::graph_traits<SMesh>::halfedge_descriptor h = halfedge(f, *graph);;
      points[0] = QPointF(get(uv, h).first, -get(uv, h).second);
      h = next(halfedge(f, *graph), *graph);
      points[1] = QPointF(get(uv, h).first, -get(uv, h).second);
      h = next(next(halfedge(f, *graph), *graph), *graph);
      points[2] = QPointF(get(uv, h).first, -get(uv, h).second);
      painter->drawPolygon(points,3);
    }
    
    pen.setColor(Qt::red);
    pen.setWidth(0);
    painter->setPen(pen);
    for(std::size_t i =0; i<polylines.size(); ++i)
    {
      std::vector<QPointF> points;
      points.reserve(polylines[i].size());
      for(std::size_t j =0; j<polylines[i].size(); ++j)
      {
        Point_2 transfo_point = transfo.transform(polylines[i][j]);
        points.push_back(QPointF(transfo_point.x(),
                                 -transfo_point.y()));
      }
      painter->drawPolyline(points.data(), static_cast<int>(points.size()));
    }
  }
  
private:
  QString texMesh_name;
  QRectF bounding_rect;
  Component* component;
  const std::vector<std::vector<EPICK::Point_2> >& polylines;
  
  SMesh* graph;
  EPICK::Aff_transformation_2 transfo;
};

class Navigation : public CGAL::Qt::GraphicsViewNavigation
{
public:
  Navigation()
    :CGAL::Qt::GraphicsViewNavigation(),
      prev_pos(QPoint(0,0))
  { }
  
protected:
  bool eventFilter(QObject *obj, QEvent *ev)
  {
    QGraphicsView* v = qobject_cast<QGraphicsView*>(obj);
    if(v == NULL) {
      QWidget* viewport = qobject_cast<QWidget*>(obj);
      if(viewport == NULL) {
        return false;
      }
      v = qobject_cast<QGraphicsView*>(viewport->parent());
      if(v == NULL) {
        return false;
      }
    }
    switch(ev->type())
    {
    case QEvent::MouseMove: {
      QMouseEvent* me = static_cast<QMouseEvent*>(ev);
      if(is_dragging)
      {
        qreal dir[2] = {v->mapToScene(me->pos()).x() - prev_pos.x(),
                        v->mapToScene(me->pos()).y() - prev_pos.y()};
        
        v->translate(dir[0],dir[1]);
        v->update();
      }
      prev_pos = v->mapToScene(me->pos());
      break;
    }
      
    case QEvent::MouseButtonPress: {
      is_dragging = true;
      break;
    }
    case QEvent::MouseButtonRelease: {
      is_dragging = false;
      break;
    }
    case QEvent::Wheel: {
      QWheelEvent* event = static_cast<QWheelEvent*>(ev);
      QPointF old_pos = v->mapToScene(event->pos());
      if(event->delta() <0)
        v->scale(1.2, 1.2);
      else
        v->scale(0.8, 0.8);
      QPointF new_pos = v->mapToScene(event->pos());
      QPointF delta = new_pos - old_pos;
      v->translate(delta.x(), delta.y());
      v->update();
      break;
    }
      
    case QEvent::MouseButtonDblClick: {
      v->fitInView(v->scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
      break;
    }
    default:
      CGAL::Qt::GraphicsViewNavigation::eventFilter(obj, ev);
    }
    return false;
  }
private:
  bool is_dragging;
  QPointF prev_pos;
};


class EngraveWidget :
    public QDockWidget,
    public Ui::EngraveWidget
{
public:
  EngraveWidget(QString name, QWidget *parent)
    :QDockWidget(name,parent)
  {
    setupUi(this);
  }
};

class Q_DECL_EXPORT Engrave_text_plugin :
    public QObject,
    public Polyhedron_demo_plugin_helper
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")
  
private:
  typedef CGAL::Surface_mesh_shortest_path_traits<EPICK, SMesh> SP_traits;
  typedef CGAL::Surface_mesh_shortest_path<SP_traits> Surface_mesh_shortest_path;
  typedef Surface_mesh_shortest_path::Face_location Face_location;
  typedef CGAL::AABB_face_graph_triangle_primitive<SMesh> Primitive;
  typedef CGAL::AABB_traits<EPICK, Primitive> Tree_traits;
  typedef CGAL::AABB_tree<Tree_traits> Tree;
  typedef EPICK::Point_3 Point_3;
  Messages_interface* messages;
  
public :
  
  void init(QMainWindow*,
            CGAL::Three::Scene_interface*,
            Messages_interface* m) Q_DECL_OVERRIDE{
    //get refs
    this->scene = Three::scene();
    this->mw = Three::mainWindow();
    messages = m;
    
    //action
    QAction* actionFitText= new QAction("Fit Text", mw);
    connect(actionFitText, SIGNAL(triggered()),
            this, SLOT(showWidget()));
    _actions << actionFitText;
    //widget
    dock_widget = new EngraveWidget("Engraving", mw);
    dock_widget->setVisible(false); // do not show at the beginning
    addDockWidget(dock_widget);
    connect(dock_widget->visualizeButton, &QPushButton::clicked,
            this, &Engrave_text_plugin::visualize);
    connect(dock_widget->engraveButton, &QPushButton::clicked,
            this, &Engrave_text_plugin::engrave);
    connect(dock_widget->text_meshButton, &QPushButton::clicked,
            this, &Engrave_text_plugin::generateTextItem);
    connect(dock_widget->letter_checkBox, &QCheckBox::toggled,
            this, &Engrave_text_plugin::generateTextItem);
    
    //items
    visu_item = nullptr;
    sel_item = nullptr;
    textMesh = nullptr;
    sm = nullptr;
    
    //transfo
    angle = 0.0;
    scalX=1.0;
    scalY=1.0;
    translation = EPICK::Vector_2(0,0);
    pointsize = 15;
    locked = false;
    connect(dock_widget->text_prec_slider, &QSlider::valueChanged,
            this, [this](){
      pointsize = dock_widget->text_prec_slider->value();
      scene->setSelectedItem(scene->item_id(sel_item));
      visualize();
    });
    connect(dock_widget->scalX_slider, &QSlider::valueChanged,
            this, [this](){
      scalX = dock_widget->scalX_slider->value()/1000.0;
      scene->setSelectedItem(scene->item_id(sel_item));
      visualize();
    });
    connect(dock_widget->scalY_slider, &QSlider::valueChanged,
            this, [this](){
      scalY = dock_widget->scalY_slider->value()/1000.0;
      scene->setSelectedItem(scene->item_id(sel_item));
      visualize();
    });
    
    connect(dock_widget->bot_slider, &QSlider::valueChanged,
            this, [this](){
      if(textMesh)
        generateTextItem();
    });
    
    connect(dock_widget->reset_button, &QPushButton::clicked,
            this, [this](){
      cleanup();
    });
    
    connect(dock_widget->t_up_pushButton, &QPushButton::clicked,
            this, [this](){
      translation += EPICK::Vector_2(0,0.005);
      scene->setSelectedItem(scene->item_id(sel_item));
      visualize();
    });
    
    connect(dock_widget->t_down_pushButton, &QPushButton::clicked,
            this, [this](){
      translation -= EPICK::Vector_2(0,0.005);
      scene->setSelectedItem(scene->item_id(sel_item));
      visualize();
    });
    
    connect(dock_widget->t_right_pushButton, &QPushButton::clicked,
            this, [this](){
      translation += EPICK::Vector_2(0.005,0);
      scene->setSelectedItem(scene->item_id(sel_item));
      visualize();
    });
    
    connect(dock_widget->t_left_pushButton, &QPushButton::clicked,
            this, [this](){
      translation -= EPICK::Vector_2(0.005,0);
      scene->setSelectedItem(scene->item_id(sel_item));
      visualize();
    });
    connect(dock_widget->rot_slider, &QSlider::valueChanged,
            this, [this](){
      if(!locked)
      {
        angle = dock_widget->rot_slider->value() * CGAL_PI/180.0;
        scene->setSelectedItem(scene->item_id(sel_item));
        visualize();
      }
    });
    graphics_scene = new QGraphicsScene(dock_widget);
    dock_widget->graphicsView->setScene(graphics_scene);
    dock_widget->graphicsView->setRenderHints(QPainter::Antialiasing);
    navigation = new Navigation();
    dock_widget->graphicsView->installEventFilter(navigation);
    dock_widget->graphicsView->viewport()->installEventFilter(navigation);
  }
  bool applicable(QAction*) const Q_DECL_OVERRIDE
  {
    return qobject_cast<Scene_polyhedron_selection_item*>
        (scene->item(scene->mainSelectionIndex()));
  }
  QList<QAction*> actions() const Q_DECL_OVERRIDE{
    return _actions;
  }
public Q_SLOTS:
  void showWidget()
  {
    dock_widget->setVisible(!dock_widget->isVisible());
  }
  
  void visualize() {
    if(!sel_item)
      sel_item =
          qobject_cast<Scene_polyhedron_selection_item*>
          (scene->item(scene->mainSelectionIndex()));
    if(!sel_item)
      return;
    if(sel_item->selected_facets.empty())
    {
      cleanup();
      return;
    }
    if(!CGAL::is_closed(*sel_item->polyhedron()))
    {
      cleanup();
      return;
    }
    if(visu_item)
      scene->erase(scene->item_id(visu_item));
    visu_item = nullptr;
    
    if(!sm)
    {
      line_to_letter_ids.clear();
      sm = new SMesh();
      sel_item->export_selected_facets_as_polyhedron(sm);
      SMesh::Halfedge_index hd =
          CGAL::Polygon_mesh_processing::longest_border(*sm).first;
      SMesh::Property_map<SMesh::Vertex_index, EPICK::Point_2> uv_map =
          sm->add_property_map<SMesh::Vertex_index, EPICK::Point_2>("v:uv").first;
      
      // Parameterized bool pmap
      boost::unordered_set<SMesh::Vertex_index> vs;
      SMP::internal::Bool_property_map< boost::unordered_set<SMesh::Vertex_index> > vpm(vs);
      
      // Parameterizer
      SMP::ARAP_parameterizer_3<SMesh> parameterizer;
      
      SMP::Error_code status = parameterizer.parameterize(*sm, hd, uv_map, 
                                                          get(boost::vertex_index, *sm), vpm);
      if(status != SMP::OK) {
        std::cout << "Encountered a problem: " << status << std::endl;
        cleanup();
        return ;
      }
      
      std::cout << "Parameterized with ARAP (SM) computed." << std::endl;
      xmin = std::numeric_limits<double>::max();
      xmax = std::numeric_limits<double>::min();
      ymin = std::numeric_limits<double>::max();
      ymax = std::numeric_limits<double>::min();
      uv_map_3 =
          sm->add_property_map<SMesh::Vertex_index, Point_3>("v:uv3").first;
      for(SMesh::Vertex_index v : sm->vertices())
      {
        uv_map_3[v] = Point_3(uv_map[v][0], uv_map[v]
            [1], 0);
        if(uv_map[v][0] > xmax)
          xmax = uv_map[v][0];
        if(uv_map[v][0] < xmin)
          xmin = uv_map[v][0];
        
        if(uv_map[v][1] > ymax)
          ymax = uv_map[v][1];
        if(uv_map[v][1] < ymin)
          ymin = uv_map[v][1];
      }
      
      CGAL::linear_least_squares_fitting_2(
            uv_map.begin(),
            uv_map.end(),
            bf_line,
            CGAL::Dimension_tag<0>());
      
      EPICK::Vector_2 A(bf_line.to_vector()),
          B(EPICK::Point_2(0,0), 
            EPICK::Point_2(1,0));      
      if (A.x()<0) A=-A;
          angle = std::acos(A.x()/CGAL::sqrt(A.squared_length()));
          if ( A.y()<0 ) angle+=3*CGAL_PI/2.;
          if (angle>2*CGAL_PI) angle-=2*CGAL_PI;
      
      locked = true;
      dock_widget->rot_slider->setSliderPosition(angle*180.0/CGAL_PI);
      locked = false;
    }
    //create Text Polyline
    QFont font;
    font.setPointSize(pointsize);
    QFontMetrics fm(font);
    float last_x(0);
    QString text = dock_widget->lineEdit->text();
    polylines.clear();
    QList<QPolygonF> polys;
    
    
    //Get the id of each letter for each polygon, so we can group up letters 
    //that have several parts, like an `i`, in the uniform letters mode.
    for(int letter_id = 0;
        letter_id < text.length();
        ++letter_id)
    {
      QPainterPath path;
      path.addText(QPoint(xmin+last_x,ymin), font, text.at(letter_id));
      last_x += fm.width(text.at(letter_id));
      last_x += fm.width(" ");
      
      Q_FOREACH(QPolygonF paul, path.toSubpathPolygons())
      {
        polys.append(paul);
        line_to_letter_ids.push_back(letter_id);
      }
    }
    float pxmin(8000),pxmax(-8000),
        pymin(8000), pymax(-8000);
    
    Q_FOREACH(QPolygonF poly, polys){
      Q_FOREACH(QPointF pf, poly)
      {
        EPICK::Point_2 v = EPICK::Point_2(pf.x(),-pf.y());
        if(v.x() < pxmin)
          pxmin = v.x();
        if(v.x() > pxmax)
          pxmax = v.x();
        if(v.y() < pymin)
          pymin = v.y();
        if(v.y() > pymax)
          pymax = v.y();
      }
    }
    //Prepare refining of polylines
    std::vector<EPECK::Segment_2> edges_2d;
    std::vector<Box> boxes_2d;
    SMesh::Property_map<SMesh::Vertex_index, EPICK::Point_2> uv_map =
        sm->property_map<SMesh::Vertex_index, EPICK::Point_2>("v:uv").first;
    Exact_to_double etd;
    std::size_t i=0;
    BOOST_FOREACH(edge_descriptor ed, edges(*sm))
    {
      EPECK::Segment_2 seg(
            EPECK::Point_2(uv_map[source(ed, *sm)][0]
          , uv_map[source(ed, *sm)][1]),
          EPECK::Point_2(uv_map[target(ed, *sm)][0]
          , uv_map[target(ed, *sm)][1]));
      edges_2d.push_back(seg);
      boxes_2d.push_back(Box(seg.bbox(), i++));
    }
    
    // build AABB-tree for face location queries
    Tree aabb_tree(faces(*sm).first, faces(*sm).second, *sm, uv_map_3);
    
    Q_FOREACH(QPolygonF poly, polys){
      polylines.push_back(std::vector<EPICK::Point_2>());
      Q_FOREACH(QPointF pf, poly)
      {
        Point_2 v = EPICK::Point_2(pf.x(),-pf.y());
        Point_2 new_point(v.x()*(xmax-xmin)/(pxmax-pxmin) +xmin,
                          v.y()*(ymax-ymin)/(pymax-pymin)+ymin
                          );
        polylines.back().push_back(new_point);
      }
    }
    visu_item = new Scene_polylines_item;
    
    
    // compute 3D coordinates
    transfo =
        EPICK::Aff_transformation_2(CGAL::TRANSLATION, 
                                    EPICK::Vector_2((xmax-xmin)/2+xmin,
                                                    (ymax-ymin)/2+ymin)+ translation)
        * EPICK::Aff_transformation_2(CGAL::ROTATION,sin(angle), cos(angle))
        * EPICK::Aff_transformation_2(scalX, 0.0,0.0,scalY)
        * EPICK::Aff_transformation_2(CGAL::TRANSLATION,
                                      EPICK::Vector_2(-(xmax-xmin)/2-xmin,
                                                      -(ymax-ymin)/2-ymin));
    BOOST_FOREACH(const std::vector<EPICK::Point_2>& polyline, polylines)
    {
      visu_item->polylines.push_back(std::vector<Point_3>());
      EPICK::Point_2 last_point;
      BOOST_FOREACH(const EPICK::Point_2& p, polyline)
      {
        std::vector<Point_2> to_insert;
        EPICK::Point_2 p_2 = transfo.transform(p);
        if(visu_item->polylines.back().size() > 0)
        {
          EPECK::Segment_2 query(
                EPECK::Point_2(last_point.x(),
                               last_point.y()), EPECK::Point_2(p_2.x(), 
                                                               p_2.y()));
          Box query_box = Box(query.bbox(), -1);
          std::vector<std::size_t> results;
          CGAL::box_intersection_d(&query_box, &query_box+1,
                                   boxes_2d.begin(),boxes_2d.end(),
                                   [&results](const Box&, 
                                   const Box& b)
          {
            results.push_back(b.info());
          });
          BOOST_FOREACH(std::size_t id, results)
          {
            //test intersection between query and segments_2d[id]:
            typedef CGAL::cpp11::result_of<EPECK::Intersect_2(EPECK::Segment_2,
                                                              EPECK::Segment_2)>::type result_type;
            result_type result = CGAL::intersection(query, edges_2d[id]);
            if(result)
              if (const EPECK::Point_2* p = boost::get<EPECK::Point_2>(&*result)) {
                to_insert.push_back(Point_2(etd(p->x()), etd(p->y())));
              }
          }
          std::sort(to_insert.begin(), to_insert.end(),
                    [last_point](const Point_2& a, const Point_2& b){
            return CGAL::squared_distance(last_point, a) < CGAL::squared_distance(last_point, b);
          });
        }
        to_insert.push_back(p_2);
        last_point = p_2;
        BOOST_FOREACH(const Point_2& p, to_insert)
        {
          Face_location loc = Surface_mesh_shortest_path::locate(
                Point_3(p.x(), p.y(), 0),
                aabb_tree, *sm, uv_map_3);
          visu_item->polylines.back().push_back(
                Surface_mesh_shortest_path::point(loc.first, loc.second,  *sm, sm->points()));
        }
      }
    }
    visu_item->setName("Text");
    visu_item->setColor(QColor(Qt::red));
    scene->addItem(visu_item);
    dock_widget->engraveButton->setEnabled(true);
    dock_widget->text_meshButton->setEnabled(true);
    
    if(graphics_scene->items().empty())
    {
      Component* component = new Component();
      face_iterator bfit;
      for(bfit = faces(*sm).begin();
          bfit != faces(*sm).end();
          ++bfit)
      {
        component->insert(*bfit);
      }
      SMesh::Property_map<halfedge_descriptor,std::pair<float, float> > uv;
      uv = sm->add_property_map<halfedge_descriptor,std::pair<float, float> >(
            "h:uv",std::make_pair(0.0f,0.0f)).first;
      SMesh::Halfedge_iterator it;
      for(it = sm->halfedges_begin();
          it != sm->halfedges_end();
          ++it)
      {
        halfedge_descriptor hd(*it);
        EPICK::FT u = uv_map[target(hd, *sm)].x();
        EPICK::FT v = uv_map[target(hd, *sm)].y();
        put(uv, *it, std::make_pair(static_cast<float>(u),static_cast<float>(v)));
      }
      
      //ParamItem does not take ownership of text_mesh_bottom
      ParamItem *param_item= new ParamItem(component, polylines, transfo, sm,
                                           QRectF(QPointF(xmin, -ymax), QPointF(xmax, -ymin)));
      graphics_scene->addItem(param_item);
      dock_widget->graphicsView->fitInView(param_item->boundingRect(), Qt::KeepAspectRatio);
    }
    else
    {
      ParamItem* param_item = static_cast<ParamItem*>(graphics_scene->items().first());
      param_item->set_transfo(transfo);
      dock_widget->graphicsView->activateWindow();
      graphics_scene->update();
    }
    // dock_widget->visualizeButton->setEnabled(false);
  }
  
  void create_text_mesh(SMesh& text_mesh)
  {
    if(!visu_item)
      return;
    if(!sel_item)
      return;
    if(sel_item->selected_facets.empty())
      return;
    if(!CGAL::is_closed(*sel_item->polyhedron()))
      return;
    CDT cdt;
    //polylines is duplicated so the transformation is only performed once
    std::vector<std::vector<EPICK::Point_2> > local_polylines;
    //Prepare refining of polylines
    std::vector<EPECK::Segment_2> edges_2d;
    std::vector<Box> boxes_2d;
    SMesh::Property_map<SMesh::Vertex_index, EPICK::Point_2> uv_map =
        sm->property_map<SMesh::Vertex_index, EPICK::Point_2>("v:uv").first;
    Exact_to_double etd;
    std::size_t i=0;
    BOOST_FOREACH(edge_descriptor ed, edges(*sm))
    {
      EPECK::Segment_2 seg(
            EPECK::Point_2(uv_map[source(ed, *sm)][0]
          , uv_map[source(ed, *sm)][1]),
          EPECK::Point_2(uv_map[target(ed, *sm)][0]
          , uv_map[target(ed, *sm)][1]));
          edges_2d.push_back(seg);
          boxes_2d.push_back(Box(seg.bbox(), i++));
    }
    typedef SMesh::Property_map<face_descriptor, EPICK::Vector_3> FPMAP;
    FPMAP fnormals =
        sm->add_property_map<face_descriptor,
        EPICK::Vector_3 >("f:normal").first;
    if(!dock_widget->letter_checkBox->isChecked())
        CGAL::Polygon_mesh_processing::compute_face_normals(*sm, fnormals);
    try{
      for(std::size_t i = 0;
          i < polylines.size(); ++i)
      {
        std::vector<Point_2>points;
        Point_2 last_point;
        for(std::size_t j = 0; j< polylines[i].size(); ++j)
        {
          std::vector<Point_2> to_insert;
          Point_2 p = polylines[i][j];
          p = transfo.transform(p);
          if(points.size() > 0)
          {
            EPECK::Segment_2 query(
                  EPECK::Point_2(last_point.x(),
                                 last_point.y()), EPECK::Point_2(p.x(), 
                                                                 p.y()));
            Box query_box = Box(query.bbox(), -1);
            std::vector<std::size_t> results;
            CGAL::box_intersection_d(&query_box, &query_box+1,
                                     boxes_2d.begin(),boxes_2d.end(),
                                     [&results](const Box&, 
                                     const Box& b)
            {
              results.push_back(b.info());
            });
            BOOST_FOREACH(std::size_t id, results)
            {
              //test intersection between query and segments_2d[id]:
              typedef CGAL::cpp11::result_of<EPECK::Intersect_2(EPECK::Segment_2,
                                                                EPECK::Segment_2)>::type result_type;
              result_type result = CGAL::intersection(query, edges_2d[id]);
              if(result)
                if (const EPECK::Point_2* ep = boost::get<EPECK::Point_2>(&*result)) {
                  Point_2 lepouaingue(etd(ep->x()), etd(ep->y()));
                  if(!dock_widget->letter_checkBox->isChecked()){
                    edge_descriptor ed = edge_descriptor(id);
                    face_descriptor fa(sm->face(sm->halfedge(ed,0))),
                        fb(sm->face(sm->halfedge(ed,1)));
                    EPICK::Vector_3 fan = get(fnormals, fa), 
                        fbn = get(fnormals, fb),
                        normal = fan+fbn;
                    normal/=CGAL::sqrt(normal.squared_length());
                    p2_normal_map[lepouaingue] = normal;
                  }
                  to_insert.push_back(lepouaingue);
                }
            }
            std::sort(to_insert.begin(), to_insert.end(),
                      [last_point](const Point_2& a,
                      const Point_2& b){
              return CGAL::squared_distance(last_point, a) < CGAL::squared_distance(last_point, b);
            });
          }
          last_point = p;
          to_insert.push_back(p);
          BOOST_FOREACH(const Point_2& ip, to_insert)
          {
            points.push_back(ip);
            point_to_letter_map[ip] = line_to_letter_ids[i];
          }
        }
        cdt.insert_constraint(points.begin(),points.end());
        local_polylines.push_back(points);
      }
    }catch(std::runtime_error&)
    {
      QApplication::restoreOverrideCursor();
      throw;
    }
    if (cdt.dimension()!=2){
      QApplication::restoreOverrideCursor();
      std::cout << "Triangulation is not of dimension 2" << std::endl;
      return;
    }
    CGAL::Bbox_2 bbox= CGAL::bbox_2(local_polylines.front().begin(),
                                    local_polylines.front().end(),
                                    EPICK());
    Q_FOREACH(const std::vector<EPICK::Point_2>& points,
              local_polylines)
    {
      bbox += CGAL::bbox_2(points.begin(), points.end(), EPICK());
    }
    mark_nested_domains(cdt);
    
    SMesh text_mesh_bottom;
    typedef SMesh::Property_map<vertex_descriptor, EPICK::Vector_3> NPMAP;
    NPMAP vnormals =
        text_mesh_bottom.add_property_map<vertex_descriptor,
        EPICK::Vector_3 >("v:normal").first;
    cdt2_to_face_graph(cdt,
                       text_mesh_bottom);
    SMesh::Property_map<vertex_descriptor, std::size_t> 
        letter_id_map = text_mesh_bottom.property_map<vertex_descriptor,
        std::size_t>("v:letter_id").first;
    typedef boost::property_map<SMesh, CGAL::vertex_point_t>::type VPMap;
    if(dock_widget->letter_checkBox->isChecked()){
      //foreach CC
      std::vector<EPICK::Vector_3> letter_normals;
      letter_normals.resize(line_to_letter_ids.back()+1);
      CGAL::Polygon_mesh_processing::compute_vertex_normals(text_mesh_bottom, vnormals);
      //compute the average normal for the cc give it to every vertex
      
      BOOST_FOREACH(vertex_descriptor vd, vertices(text_mesh_bottom))
      {
        //todo: could use not-uniform results for possibly better results ?
        letter_normals[get(letter_id_map, vd)] += 
            get(vnormals, vd);
      }
      BOOST_FOREACH(vertex_descriptor vd, vertices(text_mesh_bottom))
      {
        
        EPICK::Vector_3 normal = letter_normals[get(letter_id_map, vd)];
        put(vnormals, vd, normal /= CGAL::sqrt(normal.squared_length()));
      }
    }
    Bot<VPMap, NPMAP> bot(vnormals, dock_widget->bot_slider->value()/100000.0,
                          get(CGAL::vertex_point, text_mesh));
    Top<VPMap, NPMAP> top(vnormals, get(CGAL::vertex_point, text_mesh), 
                          0.001);
    PMP::extrude_mesh(text_mesh_bottom, text_mesh, bot, top);
    p2_normal_map.clear();
    point_to_letter_map.clear();
  }
  
  void engrave() {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    SMesh text_mesh_complete;
    create_text_mesh(text_mesh_complete);
    
    if (PMP::does_self_intersect(text_mesh_complete))
    {
      QApplication::restoreOverrideCursor();
      messages->information("Error: text mesh self-intersects!");
      return;
    }
    
    SMesh result;
    CGAL::copy_face_graph(*sel_item->polyhedron(), result);
    bool OK = PMP::corefine_and_compute_difference(result, text_mesh_complete, result);
    
    if (!OK)
    {
      QApplication::restoreOverrideCursor();
      messages->information("Error: the output mesh is not manifold!");
      return;
    }
    
    CGAL::Polygon_mesh_processing::triangulate_faces(result);
    Scene_surface_mesh_item* result_item = new Scene_surface_mesh_item(
          result);
    scene->addItem(result_item);
    graphics_scene->clear();
    cleanup();
    if(textMesh)
    {
     textMesh->setVisible(false);
    }
    QApplication::restoreOverrideCursor();
    dock_widget->engraveButton->setEnabled(false);
    dock_widget->text_meshButton->setEnabled(false);
    dock_widget->visualizeButton->setEnabled(true);
  }
  
  void generateTextItem()
  {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    
    if(textMesh)
    {
      textMesh->face_graph()->clear();
      create_text_mesh(*textMesh->face_graph());
      textMesh->invalidateOpenGLBuffers();
    }
    else
    {
      SMesh text_mesh;
      create_text_mesh(text_mesh);
      textMesh = new Scene_surface_mesh_item(text_mesh);
      connect(textMesh, &Scene_surface_mesh_item::aboutToBeDestroyed,
              this, [this](){
        textMesh = nullptr;});
      textMesh->setName("Extruded Text");
      scene->addItem(textMesh);
    }
    QApplication::restoreOverrideCursor();
  }
  
  void closure()Q_DECL_OVERRIDE
  {
    dock_widget->hide();
  }
  
private:
  template <class CDT>
  void
  mark_domains(CDT& ct,
               typename CDT::Face_handle start,
               int index,
               std::list<typename CDT::Edge>& border )
  {
    if(start->info().nesting_level != -1){
      return;
    }
    std::list<typename CDT::Face_handle> queue;
    queue.push_back(start);
    while(! queue.empty()){
      typename CDT::Face_handle fh = queue.front();
      queue.pop_front();
      if(fh->info().nesting_level == -1){
        fh->info().nesting_level = index;
        for(int i = 0; i < 3; i++){
          typename CDT::Edge e(fh,i);
          typename CDT::Face_handle n = fh->neighbor(i);
          if(n->info().nesting_level == -1){
            if(ct.is_constrained(e)) border.push_back(e);
            else queue.push_back(n);
          }
        }
      }
    }
  }
  
  
  template <class CDT>
  void
  mark_nested_domains(CDT& cdt)
  {
    for(typename CDT::All_faces_iterator it = cdt.all_faces_begin(); it !=
        cdt.all_faces_end(); ++it){
      it->info().nesting_level = -1;
    }
    std::list<typename CDT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while(! border.empty()){
      typename CDT::Edge e = border.front();
      border.pop_front();
      typename CDT::Face_handle n = e.first->neighbor(e.second);
      if(n->info().nesting_level == -1){
        mark_domains(cdt, n, e.first->info().nesting_level+1, border);
      }
    }
  }
  
  template <class CDT>
  void cdt2_to_face_graph(const CDT& cdt,
                          SMesh& tm)
  {
    //to fill the vertex_normal_map of tm, we use the normals of the faces of sm.
    typedef SMesh::Property_map<vertex_descriptor, EPICK::Vector_3> NPMAP;
    typedef SMesh::Property_map<vertex_descriptor, std::size_t> LetterIdMap;
    typedef SMesh::Property_map<face_descriptor, EPICK::Vector_3> FPMAP;
    FPMAP fnormals =
        sm->property_map<face_descriptor,
        EPICK::Vector_3 >("f:normal").first;
    NPMAP vnormals =
        tm.property_map<vertex_descriptor,
        EPICK::Vector_3 >("v:normal").first;
    
    LetterIdMap letter_id_map = tm.add_property_map<vertex_descriptor,
        std::size_t>("v:letter_id").first;
    
    Tree aabb_tree(faces(*sm).first, faces(*sm).second, *sm, uv_map_3);
    typedef typename boost::graph_traits<SMesh>::vertex_descriptor vertex_descriptor;
    
    typedef std::map<typename CDT::Vertex_handle, vertex_descriptor> Map;
    Map descriptors;
    for (typename CDT::Finite_faces_iterator fit=cdt.finite_faces_begin(),
         fit_end=cdt.finite_faces_end();
         fit!=fit_end; ++fit)
    {
      if (!fit->info().in_domain()) continue;
      CGAL::cpp11::array<vertex_descriptor,3> vds;
      for(int i=0; i<3; ++i)
      {
        typename Map::iterator it;
        bool insert_ok;
        boost::tie(it,insert_ok) =
            descriptors.insert(std::make_pair(fit->vertex(i),vertex_descriptor()));
        if (insert_ok){
          const EPICK::Point_2& pt=fit->vertex(i)->point();
          Face_location loc = Surface_mesh_shortest_path::locate(
                Point_3(pt.x(), pt.y(), 0),
                aabb_tree, *sm, uv_map_3);
          it->second = add_vertex(Surface_mesh_shortest_path::point(loc.first, loc.second,
                                                                    *sm, sm->points()), tm);
          if(!dock_widget->letter_checkBox->isChecked()){
            if(! p2_normal_map.contains(pt))
            {
              put(vnormals, it->second, get(fnormals, loc.first));
            }
            else{
              put(vnormals, it->second, p2_normal_map[pt]);
            }
          }
          else
          {
            put(letter_id_map, it->second, point_to_letter_map[pt]);
          }
        }
        vds[i]=it->second;
      }
      
      CGAL::Euler::add_face(vds, tm);
    }
  }
  
  void cleanup()
  {
    dock_widget->scalX_slider->setValue(1000);
    dock_widget->scalY_slider->setValue(1000);
    dock_widget->rot_slider->setValue(0);
    translation = EPICK::Vector_2(0,0);
    uv_map_3.reset();
    graphics_scene->clear();
    if(sel_item)
    {
      scene->erase(scene->item_id(sel_item));
      sel_item =nullptr;
    }
    if(sm)
    {
      delete sm;
      sm = nullptr;
    }
    if(visu_item)
    {
      scene->erase(scene->item_id(visu_item));
      visu_item = nullptr;
    }
  }
  
  QList<QAction*> _actions;
  EngraveWidget* dock_widget;
  Scene_polylines_item* visu_item;
  Scene_polyhedron_selection_item* sel_item;
  Scene_surface_mesh_item* textMesh;
  double angle;
  double scalX;
  double scalY;
  EPICK::Vector_2 translation;
  EPICK::Aff_transformation_2 transfo;
  std::vector<std::vector<EPICK::Point_2> > polylines;
  SMesh::Property_map<SMesh::Vertex_index, Point_3> uv_map_3;
  SMesh* sm;
  QMap<EPICK::Point_2, EPICK::Vector_3> p2_normal_map;
  std::vector<std::size_t> line_to_letter_ids;
  QMap<Point_2, std::size_t> point_to_letter_map;
  float xmin, xmax, ymin, ymax;
  int pointsize;
  bool locked;
  EPICK::Line_2 bf_line;
  QGraphicsScene *graphics_scene;
  Navigation* navigation;
};
#include "Engrave_text_plugin.moc"

