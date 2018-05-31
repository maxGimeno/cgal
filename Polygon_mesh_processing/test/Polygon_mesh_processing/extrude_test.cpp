#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/extrude.h>

#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <iostream>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SMesh;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Kernel::Triangle_3 Triangle;

template<typename MAP>
struct Bot
{
  Bot(MAP map):map(map){}
  template<typename VD, typename T>
  void operator()(VD vd, const T&)
  {
    put(map, vd, get(map, vd)+Kernel::Vector_3(-2.0,0.0,1.0));
  }
  MAP map;
  
};

template<typename MAP>
struct Top
{
  Top(MAP map):map(map){}
  
  template<typename VD, typename T>
  void operator()(VD vd, const T&)
  {
    put(map, vd, get(map, vd)+Kernel::Vector_3(0.0,2.0,-1.0));
  }
  
  MAP map;
};

template <typename PolygonMesh, typename OutputIterator>
CGAL::Bbox_3 triangles(const PolygonMesh& mesh,
                       OutputIterator out)
{
  CGAL::Bbox_3 bb;
  typename boost::property_map<PolygonMesh,CGAL::vertex_point_t>::const_type vpm =
      get(CGAL::vertex_point, mesh);
  BOOST_FOREACH(typename boost::graph_traits<PolygonMesh>::face_descriptor fd, faces(mesh)){
    typename boost::graph_traits<PolygonMesh>::halfedge_descriptor hd = halfedge(fd,mesh);
    Triangle t(get(vpm,source(hd,mesh)),
               get(vpm,target(hd,mesh)),
               get(vpm,target(next(hd,mesh),mesh)));
    *out++ = t;
    bb = bb + t.bbox();
  }
  return bb;
}

template <class Mesh>
void test_mesh(const char* filename)
{
  Mesh in, out; 
  std::ifstream input(filename);
  
  if (!input || !(input >> in))
  {
    std::cerr << "Error: cannot read Surface Mesh : " << filename << "\n";
    assert(!CGAL::is_empty(in));
    assert(false);
    return ;
  }
  CGAL::Polygon_mesh_processing::extrude_mesh(in, out, Kernel::Vector_3(0.0, 0.0, -1.0));
  std::ofstream extruded_off("extruded.off");
  extruded_off << out;
  extruded_off.close();  
  out.clear();
  
  typedef typename boost::property_map<Mesh, CGAL::vertex_point_t>::type VPMap;
  Bot<VPMap> bot(get(CGAL::vertex_point, out));
  Top<VPMap> top(get(CGAL::vertex_point, out));
  CGAL::Polygon_mesh_processing::extrude_mesh(in, out, bot, top);
  std::ofstream gen_extruded_off("gen_extruded.off");
  gen_extruded_off << out;
  gen_extruded_off.close();
  std::cerr << "All done." << std::endl;
}


void get_rotation(Kernel::FT angle, Kernel::Vector_3 axis, Kernel::Aff_transformation_3& rot)
{
  //create matrix of the rotation
  Kernel::RT c = cos(angle),
      s = sin(angle),
      ux(axis.x()),uy(axis.y()),uz(axis.z());
  Kernel::RT matrix[12] =
  {
    ux*ux*(1-c)+c, ux*uy*(1-c)-uz*s, ux*uz*(1-c)+uy*s, 0,
    ux*uy*(1-c)+uz*s, uy*uy*(1-c)+c, uy*uz*(1-c)-ux*s, 0,
    ux*uz*(1-c)-uy*s, uy*uz*(1-c)+ux*s, uz*uz*(1-c)+c, 0
  };
  rot = Kernel::Aff_transformation_3(matrix[0],matrix[1],matrix[2],
      matrix[3],matrix[4],matrix[5],
      matrix[6],matrix[7],matrix[8],
      matrix[9],matrix[10],matrix[11]);
}

bool get_transfo(const Kernel::Point_3& p1,
                 const Kernel::Point_3& p2,
                 const Kernel::Point_3& p3,
                 Kernel::Aff_transformation_3& rot,
                 Kernel::FT& angle,
                 bool split_angle = false)
{
  if(CGAL::collinear(p1,p2,p3)){
    angle = 0;
    return false;}
  //find the axis of the rotation:  
  Kernel::Vector_3 vec1(p1,p2), vec2(p2,p3);
  if(vec1.squared_length() < 0.00000000001
     || vec2.squared_length() < 0.00000000001
     || Kernel::Vector_3(p1,p3).squared_length() < 0.00000000001){
    angle = 0;
    return false;
  }
  Kernel::Vector_3 axis = CGAL::cross_product(vec1, vec2);
  axis /= CGAL::approximate_sqrt(axis.squared_length());
  //find the angle of the rotation:
  angle = 180-CGAL::approximate_dihedral_angle(
        p2, p2+axis,
        p1, p3);
  angle = split_angle ? angle * CGAL_PI/360.0
                      : angle * CGAL_PI/180;
  
  get_rotation(angle, axis, rot);
  return true;
}

template <class Mesh>
void test_mesh_2(const char* filename, const char* filename2)
{
  typedef typename boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
  typedef typename boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
  typedef typename boost::graph_traits<Mesh>::edge_descriptor edge_descriptor;
  typedef typename boost::graph_traits<Mesh>::face_descriptor face_descriptor;
  
  Mesh in, out; 
  std::ifstream input(filename);
  
  if (!input || !(input >> in))
  {
    std::cerr << "Error: cannot read Surface Mesh : " << filename << "\n";
    assert(!CGAL::is_empty(in));
    assert(false);
    return ;
  }
  //create guide
  std::ifstream ifs(filename2);
  if(!ifs) {
    std::cerr << "Error! Cannot open file " << filename2 << std::endl;
    return ;
  }
  int counter = 0;
  std::size_t n;
  std::vector<Kernel::Point_3> polyline;
  ifs >> n;
  ++counter;
  polyline.reserve(n);
  while(n--){
    Kernel::Point_3 p;
    ifs >> p;
      polyline.push_back(p);
    if(!ifs.good()) return;
  }
  if(ifs.bad() || ifs.fail()) return;
  
  //use to find best fitting plane of top.
  Kernel::Plane_3 plane;
  std::list<Triangle> triangles;
  ::triangles(in, std::back_inserter(triangles));
  Kernel::Point_3 centroid;
  CGAL::linear_least_squares_fitting_3(
        triangles.begin(),triangles.end(),plane,centroid,CGAL::Dimension_tag<2>());
  //used to rotate correctly : must translate to origin, 
  //rotate and re-translate back to original position. 
  //{
  Kernel::Vector_3 recenter_vector(centroid, Kernel::Point_3(0,0,0));
  Kernel::Aff_transformation_3 recenter1(CGAL::Translation(), recenter_vector);
  Kernel::Aff_transformation_3 recenter2(CGAL::Translation(), -recenter_vector);
  
  //}
  //copy bot in out
  std::vector<std::pair<vertex_descriptor, vertex_descriptor> > bot_v2v;
  std::vector<std::pair<halfedge_descriptor, halfedge_descriptor> > bot_h2h;
  std::vector<std::pair<face_descriptor, face_descriptor> > bot_f2f;
  CGAL::copy_face_graph(in, out, std::back_inserter(bot_v2v),
                        std::back_inserter(bot_h2h), std::back_inserter(bot_f2f));
  CGAL::Polygon_mesh_processing::reverse_face_orientations(out);
  //copy top in out
  std::vector<std::pair<vertex_descriptor, vertex_descriptor> > top_v2v;
  std::vector<std::pair<halfedge_descriptor, halfedge_descriptor> > top_h2h;
  std::vector<std::pair<face_descriptor, face_descriptor> > top_f2f;
  CGAL::copy_face_graph(in, out, std::inserter(top_v2v, top_v2v.end()),
                        std::inserter(top_h2h, top_h2h.end()), std::back_inserter(top_f2f));
  CGAL_assertion(vertices(out).size() == 2*vertices(in).size());
  typename boost::property_map<Mesh, CGAL::vertex_point_t>::type vpm =
      get(CGAL::vertex_point, out);
  Kernel::Vector_3 trans_vector(centroid, polyline[0]);
  Kernel::Aff_transformation_3 trans(CGAL::Translation(),trans_vector);
  Kernel::Aff_transformation_3 rot;
  //get rotation to orient ortho to first section
  double angle = 0.0f;
  double temp_angle;
  double test = CGAL::scalar_product(plane.orthogonal_vector(), 
  CGAL::Polygon_mesh_processing::compute_face_normal(top_f2f[0].second, out));
  if(test<0)
    plane = Kernel::Plane_3(centroid, -plane.orthogonal_vector());
  Kernel::Point_3 first_point = polyline[0]+plane.orthogonal_vector();
  
  bool do_rotate = get_transfo(
         first_point, polyline[0], polyline[1], rot, temp_angle, false);
  //if the angle is 0, then do_rotate is false but we need to flip bot and top
  if(!do_rotate 
     && CGAL::scalar_product(plane.orthogonal_vector(), 
                            Kernel::Vector_3(polyline[0], polyline[1])) > 0)
  {
    std::vector<face_descriptor> top_f, bot_f;
    for(std::size_t i = 0; i< bot_f2f.size(); ++i)
    {
      top_f.push_back(top_f2f[i].second);
      bot_f.push_back(bot_f2f[i].second);
    }
  CGAL::Polygon_mesh_processing::reverse_face_orientations(top_f, out);
  CGAL::Polygon_mesh_processing::reverse_face_orientations(bot_f, out);
  }
  //place top and bot at the beginning of the guide
  CGAL_assertion(top_v2v.size() == vertices(in).size());
  for(std::size_t i = 0; i < top_v2v.size(); ++i)
  {
    vertex_descriptor v1 = top_v2v[i].second;
    vertex_descriptor v2 = bot_v2v[i].second;
    if(do_rotate)
    {
      put(vpm, v1, get(vpm, v1).transform(recenter1));
      put(vpm, v1, get(vpm, v1).transform(rot));
      put(vpm, v1, get(vpm, v1).transform(recenter2));
      put(vpm, v2, get(vpm, v2).transform(recenter1));
      put(vpm, v2, get(vpm, v2).transform(rot));
      put(vpm, v2, get(vpm, v2).transform(recenter2));
    }
    put(vpm, v1, get(vpm, v1).transform(trans));    
    put(vpm, v2, get(vpm, v2).transform(trans));    
  }
  std::vector<Kernel::Vector_3> sections;
  for(std::size_t i = 0; i< polyline.size()-1; ++i)
  {
    sections.push_back(Kernel::Vector_3(polyline[i], polyline[i+1]));
  }
  // collect border halfedges for the creation of the triangle strip
  std::vector<halfedge_descriptor> border_hedges;
  std::vector<halfedge_descriptor> offset_border_hedges;
  std::map<halfedge_descriptor,halfedge_descriptor> h2h;//border to offset map
  std::map<halfedge_descriptor, halfedge_descriptor> left_h2h; // input h -> last_border ;
  std::map<halfedge_descriptor, halfedge_descriptor> right_h2h; // input h <- last_border ;
  //first section: start with top.
  do_rotate = get_transfo(polyline[0], polyline[1], polyline[2], rot, temp_angle, true);
  angle += 2*temp_angle;
  
  recenter_vector = Kernel::Vector_3(polyline[0], Kernel::Point_3(0,0,0)) ;
  Kernel::Vector_3 original_recenter_v = recenter_vector;
  recenter1 = Kernel::Aff_transformation_3(CGAL::Translation(), recenter_vector);
  recenter2 = Kernel::Aff_transformation_3(CGAL::Translation(), -recenter_vector);
  trans = Kernel::Aff_transformation_3(CGAL::Translation(), sections[0]);
  for(std::size_t i = 0; i< top_h2h.size(); ++i)
  {
    halfedge_descriptor input_h = top_h2h[i].first;
    if( CGAL::is_border(input_h, in) )
    {
      border_hedges.push_back(top_h2h[i].second);
      vertex_descriptor s,t;
      s = add_vertex(out);
      t = add_vertex(out);
      edge_descriptor new_e = add_edge(out);
      set_target(halfedge(new_e, out), t, out);
      set_target(opposite(halfedge(new_e, out),out), s, out);
      put(vpm, s, get(vpm, source(input_h, out)));
      put(vpm, t, get(vpm, target(input_h, out)));
      offset_border_hedges.push_back(halfedge(new_e, out));
      h2h[top_h2h[i].second]=halfedge(new_e, out);
      left_h2h[input_h] = halfedge(new_e, out);
      right_h2h[halfedge(new_e, out)] = input_h;
      if(do_rotate){
        put(vpm, s, get(vpm, s).transform(recenter1));
        put(vpm, s, get(vpm, s).transform(rot));
        put(vpm, s, get(vpm, s).transform(recenter2));
        
        put(vpm, t, get(vpm, t).transform(recenter1));
        put(vpm, t, get(vpm, t).transform(rot));
        put(vpm, t, get(vpm, t).transform(recenter2));
      }
      put(vpm, s, get(vpm, s).transform(trans));
      put(vpm, t, get(vpm, t).transform(trans));
      CGAL_assertion(CGAL::is_border(border_hedges.back(), out));
      CGAL_assertion(CGAL::is_border(offset_border_hedges.back(), out));
    }
  }
  //set connectivity
  for(std::size_t j = 0; j < border_hedges.size(); ++j)
  {
    halfedge_descriptor h1 = border_hedges[j];
    halfedge_descriptor h2 = offset_border_hedges[j];
    set_next(h2, h2h[next(h1, out)],out);
    set_next(opposite(h2,out),  h2h[next(opposite(h1, out), out)],out);
  }
  //add strip
  CGAL::Polygon_mesh_processing::extrude_impl::create_strip(border_hedges, offset_border_hedges, out, false);

  //inside sections
  Kernel::Aff_transformation_3 prev_rot = rot;
  for(std::size_t i = 1; i< polyline.size()-2; ++i)
  {
    border_hedges.clear();
    std::swap(border_hedges, offset_border_hedges);
    offset_border_hedges.clear();
    h2h.clear();
    do_rotate = get_transfo(polyline[i], polyline[i+1], polyline[i+2], rot, temp_angle, true);
    angle += 2*temp_angle;
    trans      = Kernel::Aff_transformation_3(CGAL::Translation(), sections[i]);
    recenter_vector-=sections[i-1];
    recenter1=Kernel::Aff_transformation_3(CGAL::Translation(), recenter_vector);
    recenter2=Kernel::Aff_transformation_3(CGAL::Translation(), -recenter_vector);
    for(std::size_t j = 0; j< border_hedges.size(); ++j)
    {
      halfedge_descriptor h = border_hedges[j];
      vertex_descriptor s,t;
      s = add_vertex(out);
      t = add_vertex(out);
      halfedge_descriptor newh = halfedge(add_edge(out), out);
      set_target(newh, s, out);
      set_target(opposite(newh,out), t, out);
      put(vpm, s, get(vpm, source(h, out)));
      put(vpm, t, get(vpm, target(h, out)));
      offset_border_hedges.push_back(newh);
      h2h[h]=newh;
      //move border
      if(do_rotate)
      {
        put(vpm, s, get(vpm, s).transform(recenter1));
        put(vpm, s, get(vpm, s).transform(prev_rot)); //second half of the previous rotation
        put(vpm, s, get(vpm, s).transform(rot));
        put(vpm, s, get(vpm, s).transform(recenter2));
        
        put(vpm, t, get(vpm, t).transform(recenter1));;
        put(vpm, t, get(vpm, t).transform(prev_rot)); //second half of the previous rotation
        put(vpm, t, get(vpm, t).transform(rot));
        put(vpm, t, get(vpm, t).transform(recenter2));
      }
      put(vpm, s, get(vpm, s).transform(trans));
      put(vpm, t, get(vpm, t).transform(trans));
      
      CGAL_assertion(CGAL::is_border(border_hedges.back(), out));
      CGAL_assertion(CGAL::is_border(offset_border_hedges.back(), out));
      //replace h by newh in the total map
      halfedge_descriptor input_h = right_h2h[h];
      left_h2h[input_h] = newh;
      right_h2h[newh] = input_h;
    }
    prev_rot = rot;
    //set connectivity
    for(std::size_t j = 0; j < border_hedges.size(); ++j)
    {
      halfedge_descriptor h1 = border_hedges[j];
      halfedge_descriptor h2 = offset_border_hedges[j];
      set_next(h2, h2h[next(h1, out)],out);
      set_next(opposite(h2,out),  h2h[next(opposite(h1, out), out)],out);
    }
    //add strip
    CGAL::Polygon_mesh_processing::extrude_impl::create_strip(border_hedges, offset_border_hedges, out, false);
  }
  
  //last section:
  std::size_t len = polyline.size()-1;
  std::swap(border_hedges, offset_border_hedges);
  offset_border_hedges.clear();
  h2h.clear();
  recenter1=Kernel::Aff_transformation_3(CGAL::Translation(), original_recenter_v);
  recenter2=Kernel::Aff_transformation_3(CGAL::Translation(), -original_recenter_v);
  get_rotation(angle, CGAL::cross_product(sections[len-2], sections[len-1]), rot);
  trans_vector = Kernel::Vector_3(0,0,0);
  for(std::size_t i = 0; i < sections.size(); ++i)
  {
    trans_vector += sections[i];
  }
  trans      = Kernel::Aff_transformation_3(CGAL::Translation(),trans_vector);
  //connect borders
  for(std::size_t i = 0; i< bot_h2h.size(); ++i)
  {
    halfedge_descriptor h = bot_h2h[i].first;
    if( CGAL::is_border(h, in) )
    {
      offset_border_hedges.push_back(bot_h2h[i].second);
    }
  }
  //move bot
  for(std::size_t i = 0; i< bot_v2v.size(); ++i)
  {
    vertex_descriptor v = bot_v2v[i].second;
    if(do_rotate)
    {
      put(vpm, v, get(vpm, v).transform(recenter1));
      put(vpm, v, get(vpm, v).transform(rot)); 
      put(vpm, v, get(vpm, v).transform(recenter2));
    }
    put(vpm, v, get(vpm, v).transform(trans));
  }
  CGAL::Polygon_mesh_processing::extrude_impl::create_strip(border_hedges, offset_border_hedges, out);
  
  std::ofstream extruded_off("/home/gimeno/Bureau/coude.off");
  extruded_off << out;
  extruded_off.close();  
  
  std::cerr << "All done." << std::endl;
}

int main(int argc, char* argv[])
{
  const char* filename = (argc > 1) ? argv[1] : "data/quad.off";
  const char* filename2 = (argc > 2) ? argv[2] : "data/spring.polylines.txt";
  
  test_mesh<SMesh>(filename);
  test_mesh<Polyhedron>(filename);
  
  test_mesh_2<SMesh>(filename, filename2);
  return 0;
}
