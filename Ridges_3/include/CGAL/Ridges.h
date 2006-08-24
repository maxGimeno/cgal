#ifndef _RIDGE_3_H_
#define _RIDGE_3_H_

#include <utility>
#include <list>
#include <map>

#include <CGAL/basic.h>
#include <CGAL/Min_sphere_d.h>
#include <CGAL/Optimisation_d_traits_3.h>

//note : one has to orient monge normals according to mesh normals to
//define min/max curv

CGAL_BEGIN_NAMESPACE
 
enum Ridge_type {NONE=0, BLUE_RIDGE, RED_RIDGE, CREST, 
		 BLUE_ELLIPTIC_RIDGE, BLUE_HYPERBOLIC_RIDGE, BLUE_CREST, 
		 RED_ELLIPTIC_RIDGE, RED_HYPERBOLIC_RIDGE, RED_CREST};

//---------------------------------------------------------------------------
//Ridge_line : a connected sequence of edges of a Poly crossed by a
//ridge (with a barycentric coordinate to compute the crossing point),
//with a Ridge_type and weigths : strength and sharpness. Note
//sharpness is only available (more precisely only meaningful : for
//Tag_3 it keeps its initial value 0) is the Ridge_approximation has
//been computed with the Tag_order Tag_4.
//--------------------------------------------------------------------------
template < class Poly > class Ridge_line
{
public:
  typedef typename Poly::Traits::FT                 FT;
  typedef typename Poly::Traits::Vector_3           Vector_3;
  typedef typename Poly::Traits::Point_3            Point_3;
  typedef typename Poly::Halfedge_handle            Halfedge_handle;
  typedef std::pair< Halfedge_handle, FT>           ridge_he;

  const Ridge_type line_type() const {return m_line_type;}
  Ridge_type& line_type() {return m_line_type;}

  const FT strength() const {return m_strength;}
  FT& strength() {return m_strength;}

  const FT sharpness() const {return m_sharpness;}
  FT& sharpness() {return m_sharpness;}

  const std::list<ridge_he>* line() const { return &m_line;}
  std::list<ridge_he>* line() { return &m_line;}

  //constructor
  Ridge_line();
  
  /* The output is : line_type, strength, sharpness, list of points of
     the polyline. An insert operator << is also available.
   */
  void dump_4ogl(std::ostream& out_stream) const ;
  void dump_verbose(std::ostream& out_stream) const ;

protected:
  //one of BLUE_ELLIPTIC_RIDGE, BLUE_HYPERBOLIC_RIDGE, BLUE_CREST,
  //RED_ELLIPTIC_RIDGE, RED_HYPERBOLIC_RIDGE or RED_CREST
  Ridge_type m_line_type;  
  std::list<ridge_he> m_line;
  FT m_strength;// = integral of ppal curvature along the line
  FT m_sharpness;// = (integral of second derivative of curvature
		 // along the line) divided by the size of the model
		 // (which is the radius of the smallest enclosing
		 // ball)
};

//--------------------------------------------------------------------------
// IMPLEMENTATION OF Ridge_line members
//--------------------------------------------------------------------------

 //constructor
template < class Poly >
Ridge_line<Poly>::
Ridge_line() : m_strength(0.), m_sharpness(0.)  {}
   

template < class Poly >
void Ridge_line<Poly>::
dump_4ogl(std::ostream& out_stream) const
{
  out_stream << line_type() << " "
	     << strength() << " "
	     << sharpness() << " ";

  typename std::list<ridge_he >::const_iterator
    iter = line()->begin(), 
    ite =  line()->end();
  for (;iter!=ite;iter++){
    //he: p->q, r is the crossing point
    Point_3 p = iter->first->opposite()->vertex()->point(),
      q = iter->first->vertex()->point();
    Vector_3 r = (p-CGAL::ORIGIN)*iter->second +
      (q-CGAL::ORIGIN)*(1-iter->second); 
    out_stream << " " << r ;	
  }
  out_stream  << std::endl;  
}

//verbose output
template < class Poly >
void Ridge_line<Poly>::
dump_verbose(std::ostream& out_stream) const
{
  out_stream << "Line type is : " << line_type() << std::endl
	     << "Strength is :  " << strength() << std::endl
	     << "Sharpness is : " << sharpness() << std::endl
	     << "Polyline point coordinates are : " << std::endl;

  typename std::list<ridge_he>::const_iterator
    iter = line()->begin(), 
    ite =  line()->end();
  for (;iter!=ite;iter++){
    //he: p->q, r is the crossing point
    Point_3 p = iter->first->opposite()->vertex()->point(),
      q = iter->first->vertex()->point();
    Vector_3 r = (p-CGAL::ORIGIN)*iter->second +
      (q-CGAL::ORIGIN)*(1-iter->second); 
    out_stream << r << std::endl;	
  }
}

template <class Poly>
std::ostream& 
operator<<(std::ostream& out_stream, const Ridge_line<Poly>& ridge_line)
{
  ridge_line.dump_verbose(out_stream);
  return out_stream;
}

//---------------------------------------------------------------------------
//Differential_quantities
//--------------------------------------------------------------------------
template <class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap>
  class Differential_quantities {
 public :
  Vertex2FTPropertyMap  vertex2k1_pm, vertex2k2_pm,
    vertex2b0_pm, vertex2b3_pm,
    vertex2P1_pm, vertex2P2_pm;
  Vertex2VectorPropertyMap  vertex2d1_pm, vertex2d2_pm;
};



//---------------------------------------------------------------------------
//Ridge_approximation
//--------------------------------------------------------------------------
template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
  class Ridge_approximation
{
 public:  
  typedef typename Poly::Traits::FT        FT;
  typedef typename Poly::Traits::Vector_3  Vector_3;
  typedef typename Poly::Vertex_handle     Vertex_handle;
  typedef typename Poly::Halfedge_handle   Halfedge_handle;
  typedef typename Poly::Facet_handle      Facet_handle;
  typedef typename Poly::Facet_iterator    Facet_iterator;

  typedef std::pair< Halfedge_handle, FT>  ridge_he;
  typedef Ridge_line<Poly>                 Ridge_line;

  //are ridges tagged as elliptic or hyperbolic using 3rd or 4th order
  //differential quantitities?
  //with tag_3 P1 and P2 are not used and the sharpness is not defined.
  enum Tag_order {Tag_3 = 3, Tag_4 = 4};
  
  Ridge_approximation(Poly &P,
		      Vertex2FTPropertyMap vertex2k1_pm, Vertex2FTPropertyMap vertex2k2_pm,
		      Vertex2FTPropertyMap vertex2b0_pm, Vertex2FTPropertyMap vertex2b3_pm,
		      Vertex2FTPropertyMap vertex2P1_pm, Vertex2FTPropertyMap vertex2P2_pm,
		      Vertex2VectorPropertyMap vertex2d1_pm, Vertex2VectorPropertyMap vertex2d2_pm);
  OutputIt compute_all_ridges(OutputIt it, Tag_order ord = Tag_3);
  
  //Find BLUE_RIDGE, RED_RIDGE or CREST ridges iterate on P facets,
  //find a non-visited, regular (i.e. if there is a coherent
  //orientation of ppal dir at the facet vertices), 2Xing triangle,
  //follow non-visited, regular, 2Xing triangles in both sens to
  //create a Ridge line.  Each time an edge is added the strength and
  //sharpness(if Tag_4) are updated.
  void compute_ridges(Ridge_type r_type, 
		      OutputIt ridge_lines_it,
		      Tag_order ord = Tag_3);

 protected:
  Poly* P;
  FT model_size;//radius of the smallest enclosing sphere of the Poly
		//used to make the sharpness scale independant and iso indep
  //tag to visit faces
  struct Facet_cmp{ //comparison is wrt facet addresses
    bool operator()(Facet_handle a,  Facet_handle b) const{
      return &*a < &*b;
    }
  };
  typedef std::map<Facet_handle, bool, Facet_cmp> Facet2bool_map_type;
  Facet2bool_map_type is_visited_map;

  //Property maps
  Vertex2FTPropertyMap k1, k2, b0, b3, P1, P2;
  Vertex2VectorPropertyMap d1, d2;

  //is a facet crossed by a BLUE, RED or CREST ridge? if so, return
  //the crossed edges and more precise type from BLUE_ELLIPTIC_RIDGE,
  //BLUE_HYPERBOLIC_RIDGE, BLUE_CREST, RED_ELLIPTIC_RIDGE,
  //RED_HYPERBOLIC_RIDGE, RED_CREST or NONE
  Ridge_type facet_ridge_type(Facet_handle f, 
			      Halfedge_handle& he1, 
			      Halfedge_handle& he2,
			      Ridge_type r_type,
			      Tag_order ord);
  
  //is an edge crossed by a BLUE/RED ridge? (color is BLUE_RIDGE or
  //RED_RIDGE ).  As we only test edges of regular triangles, the ppal
  //direction at endpoints d_p and d_q cannot be orthogonal. If both
  //extremalities vanish, we consider no crossing occurs. If only one
  //of them vanishes, we consider it as an positive infinitesimal and
  //apply the general rule. The general rule is that for both
  //non-vanishing extremalities, a crossing occurs if their sign
  //differ; Assuming the accute rule to orient the ppal directions,
  //there is a crossing iff d_p.d_q * b_p*b_q < 0
  void xing_on_edge(Halfedge_handle he, 
		    bool& is_crossed, 
		    Ridge_type color);
 
  //for the computation with tag_order == 3 only
  //for a ridge segment [r1,r2] in a triangle (v1,v2,v3), let r = r2 -
  //r1 and normalize, the projection of a point p on the line (r1,r2)
  //is pp=r1+tr, with t=(p-r1)*r then the vector v starting at p is
  //pointing to the ridge line (r1,r2) if (pp-p)*v >0. Return the sign
  //of b, for a ppal direction pointing to the ridge segment,
  //appearing at least at two vertices of the facet.
  //
  // for color = BLUE_RIDGE, sign = 1 if BLUE_ELLIPTIC_RIDGE, -1 if
  // BLUE_HYPERBOLIC_RIDGE 
  //
  // for color = RED_RIDGE, sign = -1 if RED_ELLIPTIC_RIDGE, 1 if
  // RED_HYPERBOLIC_RIDGE
  int b_sign_pointing_to_ridge(Vertex_handle v1, 
			       Vertex_handle v2,
			       Vertex_handle v3,
			       Vector_3 r1, Vector_3 r2, 
			       Ridge_type color);

  //a ridge line begins with a segment in a triangle given by the 2 he
  //crossed
  void init_ridge_line(Ridge_line* ridge_line, 
		       Halfedge_handle h1, Halfedge_handle h2, 
		       Ridge_type r_type,
		       Tag_order ord);
  //When the line is extended with a he, the bary coord of the
  //crossing point is computed, the pair (he,coord) is added and the
  //weigths are updated 
  void addback(Ridge_line* ridge_line, Halfedge_handle he, 
	       Ridge_type r_type, Tag_order ord);
  void addfront(Ridge_line* ridge_line, Halfedge_handle he,
		Ridge_type r_type, Tag_order ord);

  //compute the barycentric coordinate of the xing point (blue or red)
  //for he: p->q  coord is st xing_point = coord*p + (1-coord)*q
  FT bary_coord(Halfedge_handle he, Ridge_type r_type);
};


// IMPLEMENTATION OF Ridge_approximation members
/////////////////////////////////////////////////////////////////////////////
//contructor
template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
  Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
Ridge_approximation(Poly &P,
		    Vertex2FTPropertyMap vertex2k1_pm, Vertex2FTPropertyMap vertex2k2_pm,
		    Vertex2FTPropertyMap vertex2b0_pm, Vertex2FTPropertyMap vertex2b3_pm,
		    Vertex2FTPropertyMap vertex2P1_pm, Vertex2FTPropertyMap vertex2P2_pm,
		    Vertex2VectorPropertyMap vertex2d1_pm, Vertex2VectorPropertyMap vertex2d2_pm)
: P(&P), k1(vertex2k1_pm), k2(vertex2k2_pm), b0(vertex2b0_pm), b3(vertex2b3_pm), 
  P1(vertex2P1_pm), P2(vertex2P2_pm), d1(vertex2d1_pm), d2(vertex2d2_pm)
{
  //init the is_visited_map and check that the mesh is a triangular one.
  Facet_iterator itb = P.facets_begin(), ite = P.facets_end();
  for(;itb!=ite;itb++) {
    is_visited_map[itb] = false;
    CGAL_precondition( itb->is_triangle() );
  }

  CGAL::Min_sphere_d<CGAL::Optimisation_d_traits_3<typename Poly::Traits> > 
    min_sphere(P.points_begin(), P.points_end());
  model_size = min_sphere.squared_radius();
  //maybe better to use CGAL::Min_sphere_of_spheres_d ?? but need to create spheres?
}



template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
OutputIt Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
compute_all_ridges(OutputIt it, Tag_order ord)
{
  compute_ridges(BLUE_RIDGE, it, ord);
  compute_ridges(RED_RIDGE, it, ord);
  compute_ridges(CREST, it, ord);
  return it;
}

template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
  void Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
compute_ridges(Ridge_type r_type, OutputIt ridge_lines_it, Tag_order ord)
{
  CGAL_precondition( (r_type == BLUE_RIDGE) || (r_type == RED_RIDGE) || (r_type == CREST) );

  //reinit the is_visited_map
  Facet_iterator itb = P->facets_begin(), ite = P->facets_end();
  for(;itb!=ite;itb++) is_visited_map[itb] = false;
  
  itb = P->facets_begin();
  for(;itb!=ite;itb++)
    {
      Facet_handle f = itb;
      if (is_visited_map.find(f)->second) continue;
      is_visited_map.find(f)->second = true;
      Halfedge_handle h1, h2, curhe1, curhe2, curhe;
      
      //h1 h2 are the hedges crossed if any, r_type should be
      //BLUE_RIDGE, RED_RIDGE or CREST ; cur_ridge_type should be
      //BLUE_ELLIPTIC_RIDGE, BLUE_HYPERBOLIC_RIDGE, BLUE_CREST,
      //RED_ELLIPTIC_RIDGE, RED_HYPERBOLIC_RIDGE, RED_CREST or NONE
      Ridge_type cur_ridge_type = facet_ridge_type(f,h1,h2,r_type, ord);
      if ( cur_ridge_type == NONE ) continue;
      
      //a ridge_line is begining and stored
      Ridge_line* cur_ridge_line = new Ridge_line();
      init_ridge_line(cur_ridge_line, h1, h2, cur_ridge_type, ord);
      *ridge_lines_it++ = cur_ridge_line;
    
      //next triangle adjacent to h1 (push_front)
      if ( !(h1->is_border_edge()) ) 
	{
	  f = h1->opposite()->facet();
	  curhe = h1;
	  while (cur_ridge_type == facet_ridge_type(f,curhe1,curhe2,
						    r_type, ord))
	    {
	      //follow the ridge from curhe
	      if (is_visited_map.find(f)->second) break;
	      is_visited_map.find(f)->second = true;
	      if (curhe->opposite() == curhe1) curhe = curhe2;
	      else curhe = curhe1;//curhe stays at the ridge extremity
	      addfront(cur_ridge_line, curhe, cur_ridge_type, ord);
	      if ( !(curhe->is_border_edge()) ) f =
						  curhe->opposite()->facet();
	      else break;
	    }
	  //exit from the while if
	  //1. border or already visited (this is a ridge loop)
	  //2. not same type, then do not set visisted cause a BLUE_ELLIPTIC_RIDGE
	  //	  follows a BLUE_HYPERBOLIC_RIDGE
	}

      //next triangle adjacent to h2 (push_back)
      if ( !(h2->is_border_edge()) ) 
	{
	  f = h2->opposite()->facet();
	  curhe = h2;
	  while (cur_ridge_type ==
		 facet_ridge_type(f,curhe1,curhe2,r_type, ord))
	    {
	      //follow the ridge from curhe
	      if (is_visited_map.find(f)->second) break;
	      is_visited_map.find(f)->second = true;
	      if (curhe->opposite() == curhe1) curhe = curhe2;
	      else curhe = curhe1;
	      addback(cur_ridge_line, curhe, cur_ridge_type, ord);
	      if ( !(curhe->is_border_edge()) ) f =
						  curhe->opposite()->facet();
	      else break;
	    }
	} 
    }
}

template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
Ridge_type Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
facet_ridge_type(Facet_handle f, Halfedge_handle& he1, Halfedge_handle&
		 he2, Ridge_type r_type, Tag_order ord)
{
  //polyhedral data
  //we have v1--h1-->v2--h2-->v3--h3-->v1
  Halfedge_handle h1 = f->halfedge(), h2, h3;
  Vertex_handle v1, v2, v3;
  v2 = h1->vertex();
  h2 = h1->next();
  v3 = h2->vertex();
  h3 = h2->next();
  v1 = h3->vertex();

  //check for regular facet
  //i.e. if there is a coherent orientation of ppal dir at the facet vertices
  if ( d1[v1]*d1[v2] * d1[v1]*d1[v3] * d1[v2]*d1[v3] < 0 ) 
    return NONE;
   
  //determine potential crest color
  //BLUE_CREST if |sum(k1)|>|sum(k2)| sum over facet vertices vi
  //RED_CREST if |sum(k1)|<|sum(k2)|
  Ridge_type crest_color = NONE;
  if (r_type == CREST) 
    {
      if ( CGAL::abs(k1[v1]+k1[v2]+k1[v3]) > CGAL::abs(k2[v1]+k2[v2]+k2[v3]) ) 
	crest_color = BLUE_CREST; 
      if ( CGAL::abs(k1[v1]+k1[v2]+k1[v3]) < CGAL::abs(k2[v1]+k2[v2]+k2[v3]) ) 
	crest_color = RED_CREST;
      if ( CGAL::abs(k1[v1]+k1[v2]+k1[v3]) == CGAL::abs(k2[v1]+k2[v2]+k2[v3]) ) 
	return NONE;
    }
  
  //compute Xing on the 3 edges
  bool h1_is_crossed = false, h2_is_crossed = false, h3_is_crossed = false;
  if ( r_type == BLUE_RIDGE || crest_color == BLUE_CREST ) 
    {
      xing_on_edge(h1, h1_is_crossed, BLUE_RIDGE);
      xing_on_edge(h2, h2_is_crossed, BLUE_RIDGE);
      xing_on_edge(h3, h3_is_crossed, BLUE_RIDGE);
    }
  if ( r_type == RED_RIDGE || crest_color == RED_CREST ) 
    {
      xing_on_edge(h1, h1_is_crossed, RED_RIDGE);
      xing_on_edge(h2, h2_is_crossed, RED_RIDGE);
      xing_on_edge(h3, h3_is_crossed, RED_RIDGE);
    }

  //there are either 0 or 2 crossed edges
  if ( !h1_is_crossed && !h2_is_crossed && !h3_is_crossed ) 
    return NONE; 
  if (h1_is_crossed && h2_is_crossed && !h3_is_crossed)
    {
      he1 = h1; 
      he2 = h2;
    }
  if (h1_is_crossed && !h2_is_crossed && h3_is_crossed)
    {
      he1 = h1; 
      he2 = h3;
    }
  if (!h1_is_crossed && h2_is_crossed && h3_is_crossed)
    {
      he1 = h2; 
      he2 = h3;
    }
  //check there is no other case (just on edge crossed)
  CGAL_postcondition ( !( (h1_is_crossed && !h2_is_crossed && !h3_is_crossed)
			  || (!h1_is_crossed && h2_is_crossed && !h3_is_crossed)
			  || (!h1_is_crossed && h2_is_crossed && !h3_is_crossed)) );

  //There is a ridge segment in the triangle, determine its type
  Vertex_handle v_p1 = he1->opposite()->vertex(), v_q1 = he1->vertex(),
    v_p2 = he2->opposite()->vertex(), v_q2 = he2->vertex(); // he1: p1->q1
 
  if ( r_type == BLUE_RIDGE || crest_color == BLUE_CREST ) {
    FT coord1 = CGAL::abs(b0[v_q1]) / ( CGAL::abs(b0[v_p1]) +
					  CGAL::abs(b0[v_q1]) ), 
      coord2 = CGAL::abs(b0[v_q2]) / ( CGAL::abs(b0[v_p2]) +
					 CGAL::abs(b0[v_q2]) ); 
    if ( ord == Tag_3 ) {
      Vector_3 r1 = (v_p1->point()-ORIGIN)*coord1 +
	(v_q1->point()-ORIGIN)*(1-coord1), 
	r2 = (v_p2->point()-ORIGIN)*coord2 +
	(v_q2->point()-ORIGIN)*(1-coord2); 
      int b_sign = b_sign_pointing_to_ridge(v1, v2, v3, r1, r2, BLUE_RIDGE); 
      if (r_type == CREST) {if (b_sign == 1) return BLUE_CREST; 
	else return NONE;} 
      if (b_sign == 1) return BLUE_ELLIPTIC_RIDGE; 
      else return BLUE_HYPERBOLIC_RIDGE; 
    }
    else {//ord == Tag_4, check the sign of the meanvalue of the signs
      //      of P1 at the two crossing points
      FT sign_P1 =  P1[v_p1]*coord1 + P1[v_q1]*(1-coord1) 
	+ P1[v_p2]*coord2 + P1[v_q2]*(1-coord2);
      if (r_type == CREST) {if ( sign_P1 < 0 ) return BLUE_CREST; else return NONE;}
      if ( sign_P1 < 0 ) return BLUE_ELLIPTIC_RIDGE; else return BLUE_HYPERBOLIC_RIDGE;
    }
  }
 
  if ( r_type == RED_RIDGE || crest_color == RED_CREST ) {
    FT coord1 = CGAL::abs(b3[v_q1]) / ( CGAL::abs(b3[v_p1]) +
					  CGAL::abs(b3[v_q1]) ), 
      coord2 = CGAL::abs(b3[v_q2]) / ( CGAL::abs(b3[v_p2]) +
					 CGAL::abs(b3[v_q2]) ); 
    if ( ord == Tag_3 ) {
      Vector_3 r1 = (v_p1->point()-ORIGIN)*coord1 +
	(v_q1->point()-ORIGIN)*(1-coord1), 
	r2 = (v_p2->point()-ORIGIN)*coord2 +
	(v_q2->point()-ORIGIN)*(1-coord2); 
      int b_sign = b_sign_pointing_to_ridge(v1, v2, v3, r1, r2, RED_RIDGE);
      if (r_type == CREST) {if (b_sign == -1) return RED_CREST; else return NONE;} 
      if (b_sign == -1) return RED_ELLIPTIC_RIDGE; else return RED_HYPERBOLIC_RIDGE; 
    } 
    else {//ord == Tag_4, check the sign of the meanvalue of the signs
      //      of P2 at the two crossing points
      FT sign_P2 =  P2[v_p1]*coord1 + P2[v_q1]*(1-coord1) 
	+ P2[v_p2]*coord2 + P2[v_q2]*(1-coord2);
      if (r_type == CREST) {if ( sign_P2 < 0 ) return RED_CREST; else return NONE;}
      if ( sign_P2 < 0 ) return RED_ELLIPTIC_RIDGE; else return RED_HYPERBOLIC_RIDGE;
    } 
  }
  CGAL_postcondition ( "should return before!" == 0);//should return before!
}

template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
void Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
xing_on_edge(Halfedge_handle he, bool& is_crossed, Ridge_type color)
{
  is_crossed = false;
  FT sign;
  FT b_p, b_q; // extremalities at p and q for he: p->q
  Vector_3  d_p = d1[he->opposite()->vertex()],
    d_q = d1[he->vertex()]; //ppal dir
  if ( color == BLUE_RIDGE ) {
    b_p = b0[he->opposite()->vertex()];
    b_q = b0[he->vertex()];
  }
  else {     
    b_p = b3[he->opposite()->vertex()];
    b_q = b3[he->vertex()];
  }
  if ( b_p == 0 && b_q == 0 ) return;
  if ( b_p == 0 && b_q !=0 ) sign = d_p*d_q * b_q;
  if ( b_p != 0 && b_q ==0 ) sign = d_p*d_q * b_p;
  if ( b_p != 0 && b_q !=0 ) sign = d_p*d_q * b_p * b_q;
  if ( sign < 0 ) is_crossed = true;
}


template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
  int Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
  b_sign_pointing_to_ridge(Vertex_handle v1, Vertex_handle v2, Vertex_handle v3,
			   Vector_3 r1, Vector_3 r2, Ridge_type color)
{
  Vector_3 r = r2 - r1, dv1, dv2, dv3;
  FT bv1, bv2, bv3;
  if ( color == BLUE_RIDGE ) {
    bv1 = b0[v1];
    bv2 = b0[v2];
    bv3 = b0[v3];
    dv1 = d1[v1];
    dv2 = d1[v2];
    dv3 = d1[v3];
  }
  else {
    bv1 = b3[v1];
    bv2 = b3[v2];
    bv3 = b3[v3];
    dv1 = d2[v1];
    dv2 = d2[v2];
    dv3 = d2[v3];    
  }
  if ( r != CGAL::NULL_VECTOR ) r = r/CGAL::sqrt(r*r);
  FT sign1, sign2, sign3;
  sign1 = bv1*(r1 - (v1->point()-ORIGIN) + (((v1->point()-ORIGIN)-r1)*r)*r )*dv1;
  sign2 = bv2*(r1 - (v2->point()-ORIGIN) + (((v2->point()-ORIGIN)-r1)*r)*r )*dv2;
  sign3 = bv3*(r1 - (v3->point()-ORIGIN) + (((v3->point()-ORIGIN)-r1)*r)*r )*dv3;
  
  int compt = 0;
  if ( sign1 > 0 ) compt++; else if (sign1 < 0) compt--;
  if ( sign2 > 0 ) compt++; else if (sign2 < 0) compt--;
  if ( sign3 > 0 ) compt++; else if (sign3 < 0) compt--;
  
  if (compt > 0) return 1; else return -1;
}


template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
  void Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
init_ridge_line(Ridge_line* ridge_line, 
		       Halfedge_handle h1, Halfedge_handle h2, 
		       Ridge_type r_type,
		       Tag_order ord)
{
  ridge_line->line_type() = r_type;
  ridge_line->line()->push_back(ridge_he(h1, bary_coord(h1,r_type)));
  addback(ridge_line, h2, r_type, ord);
}

template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
  void Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
addback(Ridge_line* ridge_line, Halfedge_handle he,
	Ridge_type r_type, Tag_order ord)
{
  Halfedge_handle he_cur = ( --(ridge_line->line()->end()) )->first;
  FT coord_cur = ( --(ridge_line->line()->end()) )->second;//bary_coord(he_cur);
  FT coord = bary_coord(he,r_type);
  Vertex_handle v_p = he->opposite()->vertex(), v_q = he->vertex(),
    v_p_cur = he_cur->opposite()->vertex(), v_q_cur = he->vertex(); // he: p->q
  Vector_3 segment = (v_p->point()-ORIGIN)*coord + (v_q->point()-ORIGIN)*(1-coord) - 
    ((v_p_cur->point()-ORIGIN)*coord_cur + (v_q_cur->point()-ORIGIN)*(1-coord_cur));

  FT k1x, k2x; //abs value of the ppal curvatures at the Xing point on he.
  FT k_second = 0; // abs value of the second derivative of the curvature
               // along the line of curvature
  k1x = CGAL::abs(k1[v_p]) * coord + CGAL::abs(k1[v_q]) * (1-coord) ;   
  k2x = CGAL::abs(k2[v_p]) * coord + CGAL::abs(k2[v_q]) * (1-coord) ;   

  if ( (ridge_line->line_type() == BLUE_ELLIPTIC_RIDGE) 
       || (ridge_line->line_type() == BLUE_HYPERBOLIC_RIDGE) 
       || (ridge_line->line_type() == BLUE_CREST) ) {
    ridge_line->strength() += k1x * CGAL::sqrt(segment * segment); 
    if (ord == Tag_4) { 
      if (k1x != k2x) 
	k_second =CGAL::abs(( CGAL::abs(P1[v_p]) * coord + CGAL::abs(P1[v_q]) * (1-coord) )/(k1x-k2x));
      ridge_line->sharpness() += k_second * CGAL::sqrt(segment * segment) * model_size; }
  }
  if ( (ridge_line->line_type() == RED_ELLIPTIC_RIDGE) 
       || (ridge_line->line_type() == RED_HYPERBOLIC_RIDGE) 
       || (ridge_line->line_type() == RED_CREST) ) {
   ridge_line->strength() += k2x * CGAL::sqrt(segment * segment); 
   if (ord == Tag_4) {
     if (k1x != k2x) 
       k_second =CGAL::abs(( CGAL::abs(P2[v_p]) * coord + CGAL::abs(P2[v_q]) * (1-coord) )/(k1x-k2x));
     ridge_line->sharpness() += k_second * CGAL::sqrt(segment * segment) * model_size; }
   } 
  ridge_line->line()->push_back( ridge_he(he, coord));
}



template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
  void Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
addfront(Ridge_line* ridge_line, Halfedge_handle he, 
	 Ridge_type r_type, Tag_order ord)
{
  Halfedge_handle he_cur = ( ridge_line->line()->begin() )->first;
  FT coord_cur = ( ridge_line->line()->begin() )->second;
  FT coord = bary_coord(he,r_type);
  Vertex_handle v_p = he->opposite()->vertex(), v_q = he->vertex(),
    v_p_cur = he_cur->opposite()->vertex(), v_q_cur = he->vertex(); // he: p->q
  Vector_3 segment = (v_p->point()-ORIGIN)*coord + (v_q->point()-ORIGIN)*(1-coord) - 
    ((v_p_cur->point()-ORIGIN)*coord_cur + (v_q_cur->point()-ORIGIN)*(1-coord_cur));

  FT k1x, k2x; //abs value of the ppal curvatures at the Xing point on he.
  FT k_second; // abs value of the second derivative of the curvature
               // along the line of curvature
  k1x = CGAL::abs(k1[v_p]) * coord + CGAL::abs(k1[v_q]) * (1-coord) ;   
  k2x = CGAL::abs(k2[v_p]) * coord + CGAL::abs(k2[v_q]) * (1-coord) ;   

  if ( (ridge_line->line_type() == BLUE_ELLIPTIC_RIDGE) 
       || (ridge_line->line_type() == BLUE_HYPERBOLIC_RIDGE) 
       || (ridge_line->line_type() == BLUE_CREST) ) {
    ridge_line->strength() += k1x * CGAL::sqrt(segment * segment); 
   if (ord == Tag_4) {
     if (k1x != k2x) 
       k_second =CGAL::abs(( CGAL::abs(P1[v_p]) * coord + CGAL::abs(P1[v_q]) * (1-coord) )/(k1x-k2x));
     ridge_line->sharpness() += k_second * CGAL::sqrt(segment * segment) * model_size; }
  }
  if ( (ridge_line->line_type() == RED_ELLIPTIC_RIDGE) 
       || (ridge_line->line_type() == RED_HYPERBOLIC_RIDGE) 
       || (ridge_line->line_type() == RED_CREST) ) {
   ridge_line->strength() += k2x * CGAL::sqrt(segment * segment); 
   if (ord == Tag_4) {
     if (k1x != k2x) 
       k_second =CGAL::abs(( CGAL::abs(P2[v_p]) * coord + CGAL::abs(P2[v_q]) * (1-coord) )/(k1x-k2x));
     ridge_line->sharpness() += k_second * CGAL::sqrt(segment * segment) * model_size; }
   } 
  ridge_line->line()->push_front( ridge_he(he, coord));
}


template < class Poly, class OutputIt, class Vertex2FTPropertyMap, class Vertex2VectorPropertyMap >
typename Poly::Traits::FT Ridge_approximation<Poly, OutputIt, Vertex2FTPropertyMap, Vertex2VectorPropertyMap>::
bary_coord(Halfedge_handle he, Ridge_type r_type)
{
  FT b_p, b_q; // extremalities at p and q for he: p->q
  if ( (r_type == BLUE_ELLIPTIC_RIDGE) 
       || (r_type == BLUE_HYPERBOLIC_RIDGE) 
       || (r_type == BLUE_CREST) ) {
    b_p = b0[he->opposite()->vertex()];
    b_q = b0[he->vertex()];    
  }
  if ( (r_type == RED_ELLIPTIC_RIDGE) 
       || (r_type == RED_HYPERBOLIC_RIDGE) 
       || (r_type == RED_CREST) ) {
    b_p = b3[he->opposite()->vertex()];
    b_q = b3[he->vertex()];    
  }
  //denominator cannot be 0 since there is no crossing when both extremalities are 0
  return CGAL::abs(b_q) / ( CGAL::abs(b_q) + CGAL::abs(b_p) );
}
  

CGAL_END_NAMESPACE

#endif
